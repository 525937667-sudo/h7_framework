// Exercise the application and real tuning/driver code without moving hardware.
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_gm6020_test.h"
#include "app_config.h"
#include "bsp_fdcan.h"
#include "fdcan.h"

// Report test failures to CTest without invoking Windows' abort/debug dialog.
#undef assert
#define assert(condition) do { if (!(condition)) { \
  fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #condition, __FILE__, __LINE__); \
  exit(EXIT_FAILURE); } } while (0)

FdcanObject g_fdcan_object[kFdcanNbr];
FDCAN_HandleTypeDef hfdcan1;
const AppTaskConfig g_app_task_config = {.gimbal_period_ms = 1U};
static uint32_t tick, abort_count, primask;
static uint8_t sent[8];
static unsigned fail_next_tx;
static FdcanRxCallback receive;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t value) { primask = value; }
void FdcanInit(FDCAN_HandleTypeDef *handle, FdcanRxCallback callback)
{
  assert(handle == &hfdcan1);
  receive = callback;
  g_fdcan_object[0].tx_header.DataLength = FDCAN_DLC_BYTES_8;
  g_fdcan_object[0].rx_header.DataLength = FDCAN_DLC_BYTES_8;
}
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle,
                                         uint32_t buffers)
{
  assert(handle == &hfdcan1 && buffers == 0xFFFFFFFFU);
  abort_count++;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                             FDCAN_TxHeaderTypeDef *header,
                                             uint8_t *data)
{
  assert(handle == &hfdcan1 && header->Identifier == 0x1FEU);
  assert(header->DataLength == FDCAN_DLC_BYTES_8);
  for (unsigned i = 2U; i < 8U; i++) assert(data[i] == 0U);
  if (fail_next_tx) { fail_next_tx--; return HAL_ERROR; }
  memcpy(sent, data, 8U);
  return HAL_OK;
}
static void feedback(uint32_t id, uint16_t angle, int16_t rpm)
{
  uint8_t data[8] = {(uint8_t)(angle >> 8), (uint8_t)angle,
                     (uint8_t)((uint16_t)rpm >> 8), (uint8_t)rpm,
                     0U, 0U, 25U, 25U};
  receive(id, data);
}
static void step(int with_feedback)
{
  tick++;
  if (with_feedback) feedback(0x205U, 4096U, 0);
  Gm6020TestLoop();
}
static void settle(void)
{
  for (unsigned i = 0U; i < 110U; i++) step(1);
  assert(g_gm6020_test.ready && g_gm6020_test.online);
}
static void start(void)
{
  settle();
  g_gm6020_test.command = 1U;
  step(1);
  assert(g_gm6020_test.active && g_gm6020_test.fault == 0);
  assert(g_gm6020_tune.tracked);
  assert(fabsf(g_gm6020_tune.executed_current -
               g_gm6020_test.sent_current) < 1e-7f);
  assert(fabsf(g_gm6020_test.sent_current) <= 0.3f);
}
static void zero(void)
{
  assert(sent[0] == 0U && sent[1] == 0U);
  assert(!g_gm6020_test.active);
}

static void identification_settings(void)
{
  static AutotuneObject tune;
  AutotuneConfig config = g_gm6020_tune_config.plant;
  config.identification_window_time = 0.0f;
  config.identification_timeout = 0.0f;
  assert(AutotuneInit(&tune, &config));
  assert(fabsf(tune.sysid.config.window_time - 0.02f) < 1e-7f);
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  for (unsigned i = 0; i < 12100U; i++)
    (void)AutotuneUpdate(&tune, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(tune.state == kAutotuneFailed); // Legacy timeout is still 12 seconds.

  config = g_gm6020_tune_config.plant;
  assert(AutotuneInit(&tune, &config));
  assert(fabsf(tune.sysid.config.window_time - 0.05f) < 1e-7f);
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  (void)AutotuneUpdate(&tune, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(tune.pulse_ticks >= 99U && tune.pulse_ticks <= 275U);
  for (unsigned i = 0; i < 13000U; i++)
    (void)AutotuneUpdate(&tune, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(tune.state == kAutotuneIdentify); // New run may continue past 12 s.
  for (unsigned i = 0; i < 12000U; i++)
    (void)AutotuneUpdate(&tune, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(tune.state == kAutotuneFailed);
  assert(tune.error == kAutotuneInsufficientModel);
  config.identification_window_time = 0.0001f;
  assert(!AutotuneInit(&tune, &config));
  config.identification_window_time = NAN;
  assert(!AutotuneInit(&tune, &config));
  config = g_gm6020_tune_config.plant;
  config.identification_timeout = 3.0f;
  assert(!AutotuneInit(&tune, &config));

  // Keep the physical validity and fit gates; never accept negative damping
  // or lower the fit threshold to turn this board's previous failure green.
  SysidObject sysid = {0};
  SysidModel model;
  SysidConfig sc = {.sampling_time = 0.001f, .window_time = 0.05f,
                    .current_scale = 0.3f, .omega_scale = 8.0f};
  assert(SysidInit(&sysid, &sc));
  sysid.count = 100U;
  sysid.excitation = 1.0f;
  sysid.covariance[0][0] = 0.1f;
  sysid.coefficient[0] = 1.0f;
  sysid.coefficient[1] = -0.1f;
  sysid.energy = 1.0f;
  sysid.residual = 0.1f;
  assert(!SysidGetModel(&sysid, &model));
  sysid.coefficient[1] = 0.1f;
  sysid.residual = 0.3f;
  assert(!SysidGetModel(&sysid, &model));
  sysid.residual = 0.2f;
  assert(SysidGetModel(&sysid, &model));
}

static void quantized_feedback_model(void)
{
  // Known mechanical model with 1 RPM speed quantization and real CAN current
  // quantization. This checks numerical behavior, not a fit of the real motor.
  SysidConfig config = {.sampling_time = 0.001f, .window_time = 0.05f,
                        .current_scale = 0.3f, .omega_scale = 8.0f};
  SysidObject sysid;
  SysidModel model;
  assert(SysidInit(&sysid, &config));
  float omega = 0.0f;
  for (unsigned i = 0U; i < 24000U; i++)
  {
    float t = i * 0.001f;
    float current = 0.12f * sinf(4.0f * t + 0.07f * t * t) +
                    0.02f * sinf(0.7f * t);
    float measured_current = DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020,
        DjiMotorCurrentAmperesToUnits(kDjiMotorGm6020, current));
    float measured_speed = roundf(omega * 30.0f / (float)M_PI) *
                            (float)M_PI / 30.0f;
    (void)SysidUpdate(&sysid, measured_current, measured_speed);
    float direction = fminf(fmaxf(omega / 0.08f, -1.0f), 1.0f);
    omega += 0.001f * (current - 0.01f * omega - 0.04f * direction) / 0.002f;
  }
  assert(SysidGetModel(&sysid, &model));
  assert(fabsf(model.inertia / 0.002f - 1.0f) < 0.3f);
  assert(fabsf(model.viscous / 0.01f - 1.0f) < 0.6f);
}

static void steady_speed_windows(void)
{
  SysidConfig config = {.sampling_time = 0.001f, .window_time = 0.05f,
                        .current_scale = 0.3f, .omega_scale = 8.0f};
  SysidObject sysid;
  SysidModel model;
  assert(SysidInit(&sysid, &config));
  for (unsigned i = 0U; i <= 2500U; i++)
    (void)SysidUpdate(&sysid, 0.0f, 0.0f);
  assert(sysid.count == 0U); // Rest samples do not claim mechanical excitation.

  assert(SysidInit(&sysid, &config));
  for (unsigned i = 0U; i <= 2500U; i++)
    (void)SysidUpdate(&sysid, 0.05f, 2.0f);
  assert(sysid.count >= 49U); // Constant moving speed is useful for drag/friction.
  assert(sysid.covariance[1][1] < 10000.0f);
  assert(sysid.excitation == 0.0f);
  assert(!SysidGetModel(&sysid, &model)); // No dynamics: still cannot identify inertia.
}

static void intermittent_motion_excitation(void)
{
  static AutotuneObject tune;
  AutotuneConfig config = g_gm6020_tune_config.plant;
  assert(AutotuneInit(&tune, &config));
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  float initial = tune.test_amplitude;
  for (unsigned i = 0U; i < 600U; i++)
  {
    float omega = i % 500U < 25U ? 3.0f : 0.0f;
    float request = AutotuneUpdate(&tune, 0.0f, omega, 0.0f, omega);
    assert(fabsf(request) <= config.current_limit);
  }
  assert(tune.excitation_motion_ratio < 0.1f);
  assert(tune.test_amplitude > initial); // A short peak no longer blocks growth.
  for (unsigned i = 0U; i < 3000U; i++)
  {
    float omega = i % 500U < 25U ? 3.0f : 0.0f;
    float request = AutotuneUpdate(&tune, 0.0f, omega, 0.0f, omega);
    assert(fabsf(request) <= config.current_limit);
  }
  assert(tune.test_amplitude <= 0.7f * config.current_limit);

  AutotuneCancel(&tune);
  assert(AutotuneInit(&tune, &config));
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  for (unsigned i = 0U; i < 600U; i++)
  {
    float omega = i % 500U < 25U ? 7.0f : 0.0f;
    (void)AutotuneUpdate(&tune, 0.0f, omega, 0.0f, omega);
  }
  assert(tune.test_amplitude < initial); // High peak still takes priority.

  AutotuneCancel(&tune);
  assert(AutotuneInit(&tune, &config));
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  for (unsigned i = 0U; i < 600U; i++)
  {
    float omega = i % 500U < 25U ? 5.0f : 0.0f;
    (void)AutotuneUpdate(&tune, 0.0f, omega, 0.0f, omega);
  }
  // Even with a peak below the old 0.8 limit and very little motion, a speed
  // governor intervention must back off rather than increase excitation.
  assert(tune.excitation_motion_ratio < 0.1f);
  assert(tune.test_amplitude < initial);
}

static void independent_current_excitation(void)
{
  static AutotuneObject at_rest, moving;
  AutotuneConfig config = g_gm6020_tune_config.plant;
  assert(config.identification_open_loop);
  assert(AutotuneInit(&at_rest, &config));
  assert(AutotuneInit(&moving, &config));
  assert(AutotuneStart(&at_rest, 0.0f, 0.0f, NULL));
  assert(AutotuneStart(&moving, 0.0f, 0.0f, NULL));
  float a = AutotuneUpdate(&at_rest, 0.0f, 0.0f, 0.0f, 0.0f);
  float b = AutotuneUpdate(&moving, 0.0f, 1.0f, 0.0f, 1.0f);
  assert(a == b); // Independent input is not a function of ordinary speed noise.
  float brake = AutotuneUpdate(&moving, 0.0f, 7.0f, 0.0f, 7.0f);
  assert(fabsf(brake + config.current_limit*7.0f/config.speed_limit) < 1e-6f);
  assert(moving.excitation_braking && moving.pulse_ticks == 0U);
  uint32_t discarded_pulse = moving.random;
  // Dropping below the entry speed must not immediately resume acceleration.
  brake = AutotuneUpdate(&moving, 0.0f, 3.0f, 0.0f, 3.0f);
  assert(brake < 0.0f && moving.excitation_braking);
  for (unsigned i = 0U; i < 10U; i++)
    assert(AutotuneUpdate(&moving, 0.0f, 0.5f, 0.0f, 0.5f) <= 0.0f);
  assert(moving.excitation_braking);
  (void)AutotuneUpdate(&moving, 0.0f, 2.0f, 0.0f, 2.0f); // Reset settling timer.
  for (unsigned i = 0U; i < 25U; i++)
  {
    if (!moving.excitation_braking) break;
    assert(AutotuneUpdate(&moving, 0.0f, 0.0f, 0.0f, 0.0f) == 0.0f);
  }
  assert(!moving.excitation_braking && moving.pulse_ticks == 0U);
  assert(moving.random == discarded_pulse);
  (void)AutotuneUpdate(&moving, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(moving.random != discarded_pulse); // Begin a new pulse after settling.
  assert(AutotuneInit(&moving, &config));
  assert(AutotuneStart(&moving, 0.0f, 0.0f, NULL));
  brake = AutotuneUpdate(&moving, 0.0f, -5.0f, 0.0f, -5.0f);
  assert(brake > 0.0f && moving.excitation_braking); // Symmetric reverse guard.
  config.speed_offset = 1.0f;
  assert(!AutotuneInit(&at_rest, &config)); // Independent mode has no offset servo.
  config.speed_offset = 0.0f;

  config.identification_open_loop = false;
  assert(AutotuneInit(&at_rest, &config));
  assert(AutotuneInit(&moving, &config));
  assert(AutotuneStart(&at_rest, 0.0f, 0.0f, NULL));
  assert(AutotuneStart(&moving, 0.0f, 0.0f, NULL));
  a = AutotuneUpdate(&at_rest, 0.0f, 0.0f, 0.0f, 0.0f);
  b = AutotuneUpdate(&moving, 0.0f, 1.0f, 0.0f, 1.0f);
  assert(fabsf((a - b) - 0.4f * config.current_limit / config.speed_limit) < 1e-6f);
  assert(AutotuneUpdate(&moving, 0.0f, 7.0f, 0.0f, 7.0f) == -config.current_limit);
}

static void simulated_identification(void)
{
  static AutotuneObject tune;
  AutotuneConfig config = g_gm6020_tune_config.plant;
  assert(AutotuneInit(&tune, &config));
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  float omega = 0.0f, current = 0.0f, position = 0.0f;
  for (unsigned i = 0U; i < 24100U && tune.state == kAutotuneIdentify; i++)
  {
    float measured_speed = roundf(omega * 30.0f / (float)M_PI) * (float)M_PI / 30.0f;
    float measured_current = DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020,
        DjiMotorCurrentAmperesToUnits(kDjiMotorGm6020, current));
    float request = AutotuneUpdate(&tune, position, measured_speed,
                                    measured_current, measured_speed);
    assert(fabsf(request) <= config.current_limit);
    AutotuneTrackOutput(&tune, request);
    current += 0.5f * (request - current); // 2 ms current-loop response.
    float direction = fminf(fmaxf(omega / 0.08f, -1.0f), 1.0f);
    omega += 0.001f * (current - 0.01f * omega - 0.04f * direction) / 0.002f;
    position += omega * 0.001f;
  }
  assert(tune.state == kAutotuneSpeed);
  assert(tune.result.model.inertia > 0.0f && tune.result.model.viscous >= 0.0f);
  assert(tune.result.model.fitness >= 0.75f);
}

static void periodic_position_model(void)
{
  static SysidObject sysid;
  SysidModel model;
  SysidConfig config = {.sampling_time = 0.001f, .window_time = 0.05f,
                        .current_scale = 0.3f, .omega_scale = 8.0f,
                        .position_harmonics = {10.0f, 20.0f, 36.0f}};
  for (unsigned method = 0U; method < 2U; method++)
  {
    config.method = (SysidMethod)method;
    assert(SysidInit(&sysid, &config));
    assert(sysid.parameter_count == 10U);
    assert(!SysidUpdate(&sysid, 0.0f, 0.0f)); // Position cannot be silently omitted.
    for (unsigned i = 0U; i <= 24000U; i++)
    {
      float t = i * 0.001f;
      float position = 0.7f*t + 0.4f*sinf(2.0f*t) + 0.3f*sinf(5.0f*t);
      float omega = 0.7f + 0.8f*cosf(2.0f*t) + 1.5f*cosf(5.0f*t);
      float acceleration = -1.6f*sinf(2.0f*t) - 7.5f*sinf(5.0f*t);
      float direction = fminf(fmaxf(omega / 0.08f, -1.0f), 1.0f);
      float current = 0.002f*acceleration + 0.01f*omega + 0.04f*direction +
          0.005f + 0.05f*sinf(10.0f*position) + 0.03f*cosf(20.0f*position) +
          0.04f*sinf(36.0f*position);
      (void)SysidUpdateWithPosition(&sysid, current, omega, position);
    }
    assert(SysidGetModel(&sysid, &model));
    assert(sysid.validation_count >= 20U);
    assert(fabsf(model.inertia / 0.002f - 1.0f) < 0.1f);
    assert(fabsf(model.viscous / 0.01f - 1.0f) < 0.1f);
    assert(fabsf(model.position_sine[0] - 0.05f) < 0.005f);
    assert(fabsf(model.position_cosine[1] - 0.03f) < 0.005f);
    assert(model.fitness > 0.95f);
    sysid.position_max = sysid.position_min + 0.01f;
    assert(!SysidGetModel(&sysid, &model)); // A tiny arc cannot identify the map.
  }
  config.position_harmonics[1] = 10.0f;
  assert(!SysidInit(&sysid, &config));
  config.position_harmonics[1] = NAN;
  assert(!SysidInit(&sysid, &config));
  config.position_harmonics[0] = 0.0f;
  config.position_harmonics[1] = 20.0f;
  assert(!SysidInit(&sysid, &config));
  static AutotuneObject tune;
  AutotuneConfig plant = g_gm6020_tune_config.plant;
  plant.adaptive = true;
  assert(!AutotuneInit(&tune, &plant)); // No position input in normal adaptation.
}

static SysidModel validation_model(void)
{
  return (SysidModel){.inertia = 0.000124464f, .viscous = 0.00242655f,
      .friction = 0.036651f, .bias = -0.0028207f, .fitness = 0.89f,
      .position_harmonics = {10.0f, 20.0f, 36.0f},
      .position_sine = {0.0231381f, -0.0337501f, -0.0296692f},
      .position_cosine = {0.00217589f, 0.0154985f, -0.0218472f}};
}

static void integrate_validation_motor(const SysidModel *model, float executed,
                                       float *position, float *omega, float *current)
{
  // The near-zero friction time constant is below 1 ms in this fixture.
  // Integrate the plant at 100 us while keeping the controller at 1 ms;
  // a single 1 ms Euler step creates an artificial zero-speed limit cycle.
  for (unsigned j = 0U; j < 10U; j++)
  {
    *current += 0.05f*(executed-*current);
    float load = SysidLoadCurrent(model, *position, *omega, 0.08f);
    *omega += 0.0001f*(*current-model->viscous*(*omega)-load)/model->inertia;
    *position += 0.0001f*(*omega);
  }
}

static void load_compensated_validation(void)
{
  static AutotuneObject tune;
  SysidModel model = validation_model();
  float expected = model.bias + model.friction + model.position_cosine[0] +
      model.position_cosine[1] + model.position_cosine[2];
  assert(fabsf(SysidLoadCurrent(&model, 0.0f, 1.0f, 0.08f)-expected) < 1e-6f);
  assert(isnan(SysidLoadCurrent(&model, 0.0f, 0.0f, 0.0f)));
  SysidModel invalid = model;
  invalid.position_harmonics[0] = NAN;
  assert(isnan(SysidLoadCurrent(&invalid, 0.0f, 1.0f, 0.08f)));
  AutotuneConfig unsupported = g_gm6020_tune_config.plant;
  unsupported.optimize = true;
  assert(!AutotuneInit(&tune, &unsupported));
  assert(AutotuneInit(&tune, &g_gm6020_tune_config.plant));
  DobObject dob = {.is_enabled = true};
  assert(!AutotuneStart(&tune, 0.0f, 0.0f, &dob));
  for (unsigned enabled = 0U; enabled < 2U; enabled++)
  {
    AutotuneConfig config = g_gm6020_tune_config.plant;
    config.load_compensation = enabled != 0U;
    assert(AutotuneInit(&tune, &config));
    tune.result.model = model;
    tune.result.bandwidth = 20.0f;
    tune.test_amplitude = 0.96f;
    PidConfig speed = {.kp = 2.0f*20.0f*model.inertia-model.viscous,
                      .ki = model.inertia*20.0f*20.0f,
                      .sampling_time = 0.001f, .output_max = 0.3f,
                      .is_antiwindup_enabled = true};
    speed.integral_max = speed.output_max / speed.ki;
    PidInit(&tune.speed_pid, &speed);
    tune.state = kAutotuneSpeed; // Fixture: known valid model, begin validation.
    float position = -1.365f, omega = 0.0f, current = 0.0f;
    for (unsigned i = 0U; i < 60000U && AutotuneIsActive(&tune); i++)
    {
      float measured_speed = roundf(omega*30.0f/(float)M_PI)*(float)M_PI/30.0f;
      float request = AutotuneUpdate(&tune, position, measured_speed, current, measured_speed);
      assert(fabsf(request) <= config.current_limit);
      float executed = DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020,
          DjiMotorCurrentAmperesToUnits(kDjiMotorGm6020, request));
      AutotuneTrackOutput(&tune, executed);
      integrate_validation_motor(&model, executed, &position, &omega, &current);
    }
    printf("load compensation %u: state=%d rms=%f\n", enabled, tune.state, tune.result.rms_error);
    if (enabled)
    {
      assert(tune.state == kAutotuneComplete);
      assert(tune.result.rms_error < 0.55f);
    }
    else assert(tune.attempt > 0U); // Weak response triggers a gain retry.
  }
  static MotorTuneObject motor_tune;
  assert(MotorTuneInit(&motor_tune, &g_gm6020_tune_config));
  assert(MotorTuneStart(&motor_tune, 0.0f, 0.0f));
  motor_tune.baseline.state = kAutotuneFailed;
  motor_tune.baseline.error = kAutotuneTrackingError;
  assert(MotorTuneUpdate(&motor_tune, 0.0f, 0.0f, 0.0f) == 0.0f);
  assert(motor_tune.error == kMotorTuneBaselineValidationFailed);
}

static void validation_retry_direction(void)
{
  static AutotuneObject tune;
  SysidModel model = validation_model();
  for (unsigned amplified = 0U; amplified < 2U; amplified++)
  {
    AutotuneConfig config = g_gm6020_tune_config.plant;
    config.load_compensation = false;
    assert(AutotuneInit(&tune, &config));
    tune.result.model = model;
    tune.result.bandwidth = 20.0f;
    tune.test_amplitude = 0.96f;
    PidConfig speed = {.kp = 0.01f, .ki = 0.05f, .sampling_time = 0.001f,
                      .output_max = 0.3f, .integral_max = 6.0f};
    PidInit(&tune.speed_pid, &speed);
    tune.state = kAutotuneSpeed;
    for (unsigned i = 0U; i < 4000U && tune.attempt == 0U; i++)
    {
      float measured = amplified ? 2.5f*0.96f*sinf(5.0f*(i+1U)*0.001f) : 0.0f;
      float request = AutotuneUpdate(&tune, 0.0f, measured, 0.0f, measured);
      AutotuneTrackOutput(&tune, request);
    }
    assert(tune.attempt == 1U);
    assert(fabsf(tune.validation_frequency-5.0f) < 1e-5f);
    assert(fabsf(tune.result.bandwidth-(amplified ? 12.0f : 32.0f)) < 1e-4f);
    float wn = tune.result.bandwidth;
    assert(fabsf(tune.speed_pid.kp -
        (2.0f*config.speed_damping*wn*model.inertia-model.viscous)) < 1e-6f);
    assert(fabsf(tune.speed_pid.ki-model.inertia*wn*wn) < 1e-6f);
  }

  // Zero retains the original unity-damping design; reject invalid settings.
  AutotuneConfig config = g_gm6020_tune_config.plant;
  const float invalid[] = {-1.0f, 0.1f, 3.1f, NAN};
  for (unsigned i = 0U; i < sizeof(invalid)/sizeof(invalid[0]); i++)
  {
    config.speed_damping = invalid[i];
    assert(!AutotuneInit(&tune, &config));
  }
  config.speed_damping = 0.0f;
  assert(AutotuneInit(&tune, &config));
  tune.result.model = model;
  tune.result.bandwidth = 20.0f;
  tune.test_amplitude = 0.96f;
  PidConfig speed = {.kp = 0.01f, .ki = 0.05f, .sampling_time = 0.001f,
                    .output_max = 0.3f, .integral_max = 6.0f};
  PidInit(&tune.speed_pid, &speed);
  tune.state = kAutotuneSpeed;
  for (unsigned i = 0U; i < 4000U && tune.attempt == 0U; i++)
  {
    float request = AutotuneUpdate(&tune, 0.0f, 0.0f, 0.0f, 0.0f);
    AutotuneTrackOutput(&tune, request);
  }
  assert(tune.attempt == 1U);
  assert(fabsf(tune.speed_pid.kp-(2.0f*32.0f*model.inertia-model.viscous)) < 1e-6f);
}

static void pid_integral_survives_quantized_feedback(void)
{
  PidObject pid;
  PidConfig config = {.kp = 0.02f, .ki = 0.5f, .sampling_time = 0.001f,
      .output_max = 0.3f, .integral_max = 0.6f, .is_antiwindup_enabled = true};
  PidInit(&pid, &config);
  pid.error_sum = 0.2f; // 0.1 A holds a steady load; no output saturation.
  for (unsigned i = 0U; i < 1000U; i++)
  {
    float output = PidCalculate(&pid, 1.0f, i%2U ? 1.05f : 0.95f);
    assert(fabsf(output) < config.output_max);
  }
  assert(fabsf(pid.error_sum-0.2f) < 1e-4f);
  // Conditional integration must still block further windup at the limit.
  pid.error_sum = config.integral_max;
  for (unsigned i = 0U; i < 100U; i++)
    assert(PidCalculate(&pid, 100.0f, 0.0f) == config.output_max);
  assert(pid.error_sum == config.integral_max);
  PidTrackOutput(&pid, -0.1f, 50.0f);
  assert(pid.error_sum < config.integral_max);

  PidInit(&pid, &config);
  pid.error_sum = 0.2f;
  pid.previous_error = 0.05f;
  PidTuneObject optimizer;
  PidTuneConfig optimization = {.model = {.inertia=0.001f, .viscous=0.01f},
      .sampling_time=0.001f, .feedback_scale=1.0f};
  assert(PidTuneInit(&optimizer, &optimization));
  assert(PidTuneStart(&optimizer, &pid));
  (void)PidTuneCalculate(&optimizer, &pid, 1.0f, 1.05f);
  assert(fabsf(pid.error_sum-0.2f) < 1e-6f);
  assert(fabsf(optimizer.integral_sensitivity[1]+0.2f) < 1e-6f);
}

static void low_speed_settings(void)
{
  static MotorTuneObject motor;
  MotorTuneConfig motor_config = g_gm6020_tune_config;
  const float invalid[] = {-0.1f, 0.01f, 0.51f, NAN};
  for (unsigned i = 0U; i < sizeof(invalid)/sizeof(invalid[0]); i++)
  {
    motor_config.low_speed_ratio = invalid[i];
    assert(!MotorTuneInit(&motor, &motor_config));
  }
  motor_config.low_speed_ratio = 0.0f;
  assert(MotorTuneInit(&motor, &motor_config));
}

static void full_periodic_tuning(void)
{
  static MotorTuneObject tune;
  SysidModel model = validation_model();
  assert(MotorTuneInit(&tune, &g_gm6020_tune_config));
  assert(MotorTuneStart(&tune, 0.0f, 0.0f));
  float position = 0.0f, omega = 0.0f, current = 0.0f;
  unsigned attempt = 0U;
  for (unsigned i = 0U; i < 600000U && MotorTuneIsActive(&tune); i++)
  {
    float measured_speed = roundf(omega*30.0f/(float)M_PI)*(float)M_PI/30.0f;
    float request = MotorTuneUpdate(&tune, position, measured_speed, current);
    if (attempt != tune.baseline.attempt)
    {
      attempt = tune.baseline.attempt;
      printf("baseline retry %u: rms=%f peak=%f wn=%f\n", attempt,
          tune.baseline.result.rms_error, tune.baseline.result.peak_error,
          tune.baseline.result.bandwidth);
    }
    float limited = fminf(fmaxf(request, -0.3f), 0.3f);
    float executed = DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020,
        DjiMotorCurrentAmperesToUnits(kDjiMotorGm6020, limited));
    if (MotorTuneIsActive(&tune)) assert(MotorTuneTrackOutput(&tune, executed));
    integrate_validation_motor(&model, executed, &position, &omega, &current);
  }
  printf("full tune: state=%d error=%d baseline=%d rms=%f evaluated=%u verified=%u pareto=%u\n",
      tune.state, tune.error, tune.baseline.error, tune.baseline.result.rms_error,
      tune.result.evaluated_count, tune.result.verified_count, tune.result.pareto_count);
  printf("prepare: position=%f omega=%f center=%f elapsed=%f reference=%f output=%f load=%f ready=%f\n",
      position, omega, tune.center, tune.elapsed, tune.speed_pid.reference,
      tune.speed_pid.output, tune.load_current, tune.ready_time);
  printf("gains: wn=%f kp=%f ki=%f M=%f B=%f\n", tune.baseline.result.bandwidth,
      tune.speed_pid.kp, tune.speed_pid.ki, tune.result.model.inertia, tune.result.model.viscous);
  for (unsigned c = 0U; c < 9U; c++)
    printf("candidate %u: state=%d rms=%f overshoot=%f settling=%f settled=%d saturation=%f\n",
        c, tune.result.candidate[c].state, tune.result.candidate[c].training.rms_error,
        tune.result.candidate[c].training.overshoot, tune.result.candidate[c].training.settling_time,
        tune.result.candidate[c].training.settled, tune.result.candidate[c].training.saturation);
  // The estimated model passes baseline tracking, but low-speed quantization
  // and residual load mismatch still violate the candidate settling/overshoot
  // limits. Preserve that rejection rather than exporting a poor controller.
  assert(tune.baseline.state == kAutotuneComplete);
  assert(tune.state == kMotorTuneFailed && tune.error == kMotorTuneNoFeasibleCandidate);
  assert(tune.result.evaluated_count == 8U && tune.result.pareto_count == 0U);
  assert(tune.result.candidate[8].state == kMotorTuneDynamicLimit);
  for (unsigned c = 0U; c < 8U; c++)
  {
    const MotorTuneMetrics *m = &tune.result.candidate[c].training;
    assert(tune.result.candidate[c].rejection_reasons != 0U);
    assert(tune.result.candidate[c].completed_segments < 16U);
    if (!m->settled) assert(m->unstable_segments != 0U);
    assert(m->worst_overshoot_segment < 16U);
  }
  PidConfig speed;
  SysidModel exported;
  assert(!MotorTuneExportController(&tune, 0U, &speed, NULL, &exported));
  // Completed-result fixture checks the export contract independently of
  // whether this deliberately demanding plant meets the search constraints.
  tune.state = kMotorTuneComplete;
  tune.result.pareto_count = 1U;
  tune.result.pareto[0] = 8U;
  assert(!MotorTuneExport(&tune, 0U, &speed, NULL)); // FF cannot be silently lost.
  assert(MotorTuneExportController(&tune, 0U, &speed, NULL, &exported));
  assert(exported.position_harmonics[0] == 10.0f);
}

static void guard_fixture(MotorTuneObject *tune)
{
  assert(MotorTuneInit(tune, &g_gm6020_tune_config));
  assert(MotorTuneStart(tune, 0.0f, 0.0f));
  // Isolate a closed-loop search response from identification/return motion.
  tune->state = kMotorTuneSearch;
  tune->speed_amplitude = 2.2f; // Keep +/-4 rad/s within this fixture's envelope.
  tune->result.model = validation_model();
  tune->result.candidate_count = 1U;
  tune->result.candidate[0].bandwidth = 48.0f;
  PidConfig speed = {.kp=0.06f, .ki=0.2f, .sampling_time=0.001f,
      .output_max=0.3f, .integral_max=1.5f, .is_antiwindup_enabled=true};
  PidInit(&tune->speed_pid, &speed);
  tune->baseline.speed_pid = tune->speed_pid;
  PidConfig position = {.kp=3.0f, .sampling_time=0.001f, .output_max=8.0f};
  PidInit(&tune->baseline.position_pid, &position);
  tune->baseline.result.position_kp = 3.0f;
  tune->result.candidate[1].speed = speed;
  tune->result.candidate[1].position = position;
}

static void early_candidate_rejection(void)
{
  static MotorTuneObject tune;
  for (unsigned negative = 0U; negative < 2U; negative++)
  {
    guard_fixture(&tune);
    tune.speed_amplitude = 1.8f;
    tune.result.candidate_count = 2U;
    tune.speed_pid.reference = negative ? -1.8f : 1.8f;
    float omega = negative ? -5.5f : 5.5f; // Below the 9.6 rad/s hard limit.
    float request = MotorTuneUpdate(&tune, 0.0f, omega, negative ? -0.25f : 0.25f);
    assert(MotorTuneIsActive(&tune) && tune.state == kMotorTunePrepare);
    assert(tune.error == kMotorTuneNoError && tune.candidate_index == 1U);
    assert(tune.prepare_braking && tune.prepare_reference == 0.0f);
    assert(fabsf(tune.speed_pid.error_sum) < 1e-6f); // No transient candidate current.
    assert(request*omega < 0.0f); // Validated baseline already brakes this cycle.
    assert(tune.result.candidate[0].state == kMotorTuneRejected);
    assert(tune.result.candidate[0].rejection_reasons & kMotorTuneRejectTrackingEnvelope);
    assert(tune.result.pareto_count == 0U);
    assert(isfinite(tune.result.candidate[0].training.rms_error));
    assert(MotorTuneTrackOutput(&tune, fminf(fmaxf(request,-0.3f),0.3f)));
    for (unsigned i = 0U; i < 360U; i++)
    {
      request = MotorTuneUpdate(&tune, 0.0f, 0.0f, 0.0f);
      assert(MotorTuneTrackOutput(&tune, fminf(fmaxf(request,-0.3f),0.3f)));
    }
    assert(tune.state == kMotorTuneSearch && tune.candidate_index == 1U);
  }
  guard_fixture(&tune);
  tune.result.candidate_count = 2U;
  tune.tick = tune.segment_ticks;
  tune.samples = tune.segment_ticks;
  tune.metrics.overshoot = 0.5f;
  float request = MotorTuneUpdate(&tune, 0.2f, 0.0f, 0.25f);
  assert(MotorTuneTrackOutput(&tune, fminf(fmaxf(request,-0.3f),0.3f)));
  float previous_reference = 0.0f;
  for (unsigned i = 0U; i < 300U; i++)
  {
    request = MotorTuneUpdate(&tune, 0.2f, 0.0f, 0.0f);
    assert(tune.state == kMotorTunePrepare);
    if (i < 40U) assert(tune.prepare_braking && tune.prepare_reference == 0.0f);
    assert(tune.prepare_reference <= 0.0f);
    assert(fabsf(tune.prepare_reference-previous_reference) <= 0.00081f);
    previous_reference = tune.prepare_reference;
    assert(MotorTuneTrackOutput(&tune, fminf(fmaxf(request,-0.3f),0.3f)));
  }
  assert(!tune.prepare_braking && tune.prepare_reference < 0.0f);
  guard_fixture(&tune);
  tune.result.candidate_count = 2U;
  tune.tick = tune.segment_ticks;
  tune.samples = tune.segment_ticks;
  tune.metrics.overshoot = 0.5f; // Irreversible failure at the first segment end.
  (void)MotorTuneUpdate(&tune, 0.0f, 0.0f, 0.0f);
  assert(tune.state == kMotorTunePrepare && tune.candidate_index == 1U);
  assert(tune.result.candidate[0].completed_segments == 1U);
  assert(tune.result.candidate[0].rejection_reasons & kMotorTuneRejectOvershoot);
}

static void oscillation_protection(void)
{
  static MotorTuneObject tune;
  for (unsigned mode = 0U; mode < 3U; mode++)
  {
    guard_fixture(&tune);
    unsigned i;
    for (i = 0U; i < 600U && MotorTuneIsActive(&tune); i++)
    {
      float omega = mode == 0U ? (i/5U%2U ? -4.0f : 4.0f) :
                    mode == 1U ? (i%2U ? -0.10472f : 0.10472f) :
                                 (i/100U%2U ? -4.0f : 4.0f);
      float request = MotorTuneUpdate(&tune, 0.0f, omega, 0.0f);
      if (MotorTuneIsActive(&tune)) assert(MotorTuneTrackOutput(&tune, request));
      else assert(request == 0.0f);
    }
    if (mode == 0U)
    {
      assert(i <= 60U && tune.error == kMotorTuneOscillationDetected);
      assert(tune.result.candidate[0].state == kMotorTuneResponseAborted);
      assert(tune.result.pareto_count == 0U && !tune.pending);
      assert(tune.guard_reversals >= 4U && tune.guard_rms_current_slew > 30.0f);
      assert(MotorTuneUpdate(&tune, 0.0f, 0.0f, 0.0f) == 0.0f);
    }
    else assert(MotorTuneIsActive(&tune)); // Quantization and slow changes alone.
  }
  // The proportional-rate screen also constrains baseline retries.
  static AutotuneObject baseline;
  AutotuneConfig config = g_gm6020_tune_config.plant;
  assert(AutotuneInit(&baseline, &config));
  baseline.result.model = validation_model();
  baseline.result.bandwidth = 72.0f;
  baseline.test_amplitude = 0.96f;
  PidConfig speed = {.kp=0.01f, .ki=0.1f, .sampling_time=0.001f,
      .output_max=0.3f, .integral_max=3.0f};
  PidInit(&baseline.speed_pid, &speed);
  baseline.state = kAutotuneSpeed;
  for (unsigned i = 0U; i < 4000U && baseline.attempt == 0U; i++)
    (void)AutotuneUpdate(&baseline, 0.0f, 0.0f, 0.0f, 0.0f);
  assert(baseline.attempt == 1U);
  assert(baseline.speed_pid.kp*config.sampling_time/baseline.result.model.inertia <= 0.30001f);
}

static void recovery_handoff_simulation(void)
{
  static MotorTuneObject tune;
  guard_fixture(&tune);
  tune.speed_amplitude = 1.8f;
  tune.result.candidate_count = 2U;
  SysidModel model = {.inertia=0.0001871141f, .viscous=0.0003671168f,
      .friction=0.05613251f, .bias=0.008484866f, .fitness=0.9019843f,
      .position_harmonics={10.0f,20.0f,36.0f},
      .position_sine={0.022072345f,-0.003503456f,0.03171976f},
      .position_cosine={-0.006340349f,0.007792966f,0.03357516f}};
  tune.result.model = model;
  PidConfig speed = {.kp=0.0355588f, .ki=0.19160487f, .sampling_time=0.001f,
      .output_max=0.3f, .integral_max=0.3f/0.19160487f, .is_antiwindup_enabled=true};
  PidInit(&tune.baseline.speed_pid, &speed);
  tune.baseline.result.bandwidth = 32.0f;
  tune.baseline.result.position_kp = 4.8f;
  tune.baseline.test_amplitude = 0.96f;
  tune.baseline.validation_frequency = 5.0f;
  tune.speed_pid.reference = 0.54f;
  tune.executed_current = 0.25f;
  tune.previous_current = 0.25f;
  float position=0.1879f, omega=5.5f, current=0.25f, peak=omega;
  for (unsigned i=0U; i<20000U; i++)
  {
    float measured=roundf(omega*30.0f/(float)M_PI)*(float)M_PI/30.0f;
    float request=MotorTuneUpdate(&tune,position,measured,current);
    assert(MotorTuneIsActive(&tune));
    float executed=DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020,
        DjiMotorCurrentAmperesToUnits(kDjiMotorGm6020,fminf(fmaxf(request,-0.3f),0.3f)));
    assert(MotorTuneTrackOutput(&tune,executed));
    integrate_validation_motor(&model,executed,&position,&omega,&current);
    peak=fmaxf(peak,fabsf(omega));
    if (tune.state==kMotorTuneSearch && tune.candidate_index==1U) break;
  }
  printf("recovery: phase=%d candidate=%u peak=%f position=%f\n",
      tune.state,tune.candidate_index,peak,position);
  assert(tune.state==kMotorTuneSearch && tune.candidate_index==1U);
  assert(peak<9.6f && fabsf(position)<0.063f);
}

static void full_linear_tuning(void)
{
  static MotorTuneObject tune;
  MotorTuneConfig config = g_gm6020_tune_config;
  memset(config.plant.identification_position_harmonics, 0,
         sizeof(config.plant.identification_position_harmonics));
  config.plant.load_compensation = false;
  assert(MotorTuneInit(&tune, &config));
  assert(MotorTuneStart(&tune, 0.0f, 0.0f));
  SysidModel plant = {.inertia = 0.001f, .viscous = 0.01f, .fitness = 1.0f};
  float position = 0.0f, omega = 0.0f, current = 0.0f;
  for (unsigned i = 0U; i < 600000U && MotorTuneIsActive(&tune); i++)
  {
    float measured = roundf(omega*30.0f/(float)M_PI)*(float)M_PI/30.0f;
    float request = MotorTuneUpdate(&tune, position, measured, current);
    float executed = DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020,
        DjiMotorCurrentAmperesToUnits(kDjiMotorGm6020, fminf(fmaxf(request,-0.3f),0.3f)));
    if (MotorTuneIsActive(&tune)) assert(MotorTuneTrackOutput(&tune, executed));
    integrate_validation_motor(&plant, executed, &position, &omega, &current);
  }
  printf("linear tune: state=%d error=%d evaluated=%u verified=%u pareto=%u\n",
      tune.state, tune.error, tune.result.evaluated_count,
      tune.result.verified_count, tune.result.pareto_count);
  assert(tune.state == kMotorTuneComplete && tune.result.pareto_count > 0U);
  for (unsigned p = 0U; p < tune.result.pareto_count; p++)
  {
    const MotorTuneCandidate *candidate = &tune.result.candidate[tune.result.pareto[p]];
    assert(candidate->state == kMotorTuneVerified);
    assert(candidate->completed_segments == 16U && candidate->rejection_reasons == 0U);
    assert(candidate->training.unstable_segments == 0U);
    assert(candidate->validation.unstable_segments == 0U);
  }
  PidConfig speed;
  assert(MotorTuneExport(&tune, 0U, &speed, NULL));
}

int main(void)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  identification_settings();
  quantized_feedback_model();
  steady_speed_windows();
  intermittent_motion_excitation();
  independent_current_excitation();
  simulated_identification();
  periodic_position_model();
  load_compensated_validation();
  validation_retry_direction();
  pid_integral_survives_quantized_feedback();
  low_speed_settings();
  oscillation_protection();
  early_candidate_rejection();
  recovery_handoff_simulation();
  full_periodic_tuning();
  full_linear_tuning();
  Gm6020TestInit();
  assert(g_gm6020_trace_version == 0x474D0001U);
  step(0);
  zero(); // Boot must not excite the motor.
  g_gm6020_test.command = 1U;
  step(0);
  assert(g_gm6020_test.fault == kGm6020TestNotReady);
  zero();

  feedback(0x206U, 4096U, 0);
  step(0);
  assert(g_gm6020_test.rx_count == 0U);
  feedback(0x205U, 9000U, 0);
  step(0);
  assert(g_gm6020_test.rx_count == 0U);
  g_fdcan_object[0].rx_header.DataLength = 4U;
  feedback(0x205U, 4096U, 0);
  step(0);
  assert(g_gm6020_test.rx_count == 0U);
  g_fdcan_object[0].rx_header.DataLength = FDCAN_DLC_BYTES_8;

  start();
  assert(g_gm6020_test.trace_samples > 0U);
  assert(g_gm6020_trace[0].time_ms == 0U);
  assert(g_gm6020_test.position == 0.0f);
  float elapsed = g_gm6020_tune.baseline.total_time;
  g_gm6020_test.command = 1U;
  step(1);
  assert(g_gm6020_tune.baseline.total_time > elapsed); // No restart in motion.
  g_gm6020_test.command = 2U;
  step(1);
  zero();
  assert(abort_count > 0U && g_gm6020_tune.state == kMotorTuneCancelled);
  uint32_t frozen_count = g_gm6020_trace_count;
  step(1);
  assert(g_gm6020_trace_count == frozen_count);

  start();
  for (unsigned i = 0U; i < 11U; i++) step(0);
  assert(g_gm6020_test.fault == kGm6020TestFeedbackLost);
  zero();
  settle();
  assert(!g_gm6020_test.active); // Reconnection alone must not restart.

  start();
  fail_next_tx = 1U;
  step(1);
  assert(g_gm6020_test.fault == kGm6020TestCanTxFailed);
  assert(g_gm6020_test.tx_error_count > 0U);
  zero();

  start();
  tick += 4U;
  feedback(0x205U, 4096U, 0);
  Gm6020TestLoop();
  assert(g_gm6020_test.fault == kGm6020TestTimingFault);
  zero();

  start();
  tick++;
  feedback(0x205U, 4096U, 1000);
  Gm6020TestLoop();
  assert(g_gm6020_tune.state == kMotorTuneFailed);
  assert(g_gm6020_tune.error == kMotorTuneMotionLimit);
  float terminal_omega = g_gm6020_tune.last_omega;
  assert(terminal_omega > 9.6f);
  zero();
  step(1);
  assert(g_gm6020_tune.last_omega == terminal_omega);
  assert(g_gm6020_test.terminal_omega == terminal_omega);

  start();
  guard_fixture(&g_gm6020_tune);
  for (unsigned i = 0U; i < 70U && g_gm6020_test.active; i++)
  {
    tick++;
    feedback(0x205U, 4096U, i/5U%2U ? -40 : 40);
    Gm6020TestLoop();
  }
  assert(g_gm6020_tune.error == kMotorTuneOscillationDetected);
  zero(); // Application retracts pending CAN output and queues zero immediately.
  settle();
  assert(!g_gm6020_test.active); // No automatic restart after a trip.

  g_gm6020_test.command = 3U;
  step(1);
  assert(!g_gm6020_test.exported);
  // A completed result fixture verifies pareto-index mapping and no motion.
  g_gm6020_tune.state = kMotorTuneComplete;
  g_gm6020_tune.result.pareto_count = 1U;
  g_gm6020_tune.result.pareto[0] = 2U;
  g_gm6020_tune.result.candidate[2].speed.kp = 0.125f;
  g_gm6020_test.pareto_index = 256U;
  g_gm6020_test.command = 3U;
  step(1);
  assert(!g_gm6020_test.exported); // Must not wrap an invalid index to zero.
  g_gm6020_test.pareto_index = 0U;
  g_gm6020_test.command = 3U;
  step(1);
  assert(g_gm6020_test.exported && g_gm6020_speed_config.kp == 0.125f);
  assert(g_gm6020_test.fault == kGm6020TestNoFault);
  zero();
  puts("GM6020 integration checks passed");
  return 0;
}
