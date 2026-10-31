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
  assert(AutotuneInit(&tune, &config));
  assert(fabsf(tune.sysid.config.window_time - 0.05f) < 1e-7f);
  assert(AutotuneStart(&tune, 0.0f, 0.0f, NULL));
  assert(tune.identification_stage == kAutotuneProbe);
  for (unsigned i = 0U; i < 25000U && AutotuneIsActive(&tune); i++)
  {
    float request = AutotuneUpdate(&tune, 0.0f, 0.0f, 0.0f, 0.0f);
    assert(isfinite(request) && fabsf(request) <= config.current_limit);
    AutotuneTrackOutput(&tune, request);
  }
  assert(tune.state == kAutotuneFailed);
  assert(tune.error == kAutotuneInsufficientModel);
  config.identification_window = 0.0001f;
  assert(!AutotuneInit(&tune, &config));
  config.identification_window = NAN;
  assert(!AutotuneInit(&tune, &config));
  config = g_gm6020_tune_config.plant;
  config.identification_timeout = 3.0f;
  assert(!AutotuneInit(&tune, &config));
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

int main(void)
{
  setvbuf(stdout, NULL, _IONBF, 0);
  quantized_feedback_model();
  identification_settings();
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
  float terminal_omega = g_gm6020_test.terminal_omega;
  assert(terminal_omega > 9.6f);
  zero();
  step(1);
  assert(g_gm6020_test.terminal_omega == terminal_omega);

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
