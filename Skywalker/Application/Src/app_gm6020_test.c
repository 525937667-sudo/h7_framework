/**
 * @file app_gm6020_test.c
 * @brief Unloaded GM6020, current mode, ID 1, Miaoboard CAN1 (1 Mbit/s).
 * @details See docs/gm6020-autotune-test.md for debugger commands and results.
 */
#include "app_gm6020_test.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "bsp_fdcan.h"
#include "fdcan.h"

enum
{
  kMotorRxId = 0x205U,
  kMotorTxId = 0x1FEU,
  kFeedbackTimeoutMs = 10U,
  kReadyTimeMs = 100U,
  kTestTimeoutMs = 600000U,
};

// Initial bench settings; change these in source, then rebuild.
// This test tunes the speed PI. Position tuning can be enabled explicitly.
const MotorTuneConfig g_gm6020_tune_config = {
  .plant = {.sampling_time = 0.001f,
            .current_limit = 0.3f,
            .speed_limit = 8.0f,
            .position_min = -12.5663706f,
            .position_max = 12.5663706f,
            .response_time = 0.2f,
            .tune_position = false,
            .identification = kSysidExponential,
            .identification_window_time = 0.05f,
            .identification_timeout = 24.0f,
            .identification_open_loop = true,
            .identification_position_harmonics = {10.0f, 20.0f, 36.0f},
            .load_compensation = true,
            .speed_damping = 3.0f},
  .bandwidth = {.lower = 24.0f, .upper = 72.0f, .count = 3U},
  .damping = {.lower = 1.0f, .upper = 3.0f, .count = 3U},
  .feedforward = {.lower = 0.0f, .upper = 0.0f, .count = 1U},
  .limits = {.rms_error = 0.25f,
             .overshoot = 0.30f,
             .settling_time = 0.5f,
             .saturation = 0.20f},
  .speed_amplitude = 1.8f,
  .position_amplitude = 0.35f,
  .segment_time = 1.0f,
  .settling_band = 0.10f,
  .low_speed_ratio = 0.30f,
};

Gm6020TestStatus g_gm6020_test;
MotorTuneObject g_gm6020_tune;
DjiMotor g_gm6020_motor;
PidConfig g_gm6020_speed_config;
PidConfig g_gm6020_position_config;
SysidModel g_gm6020_load_model;
float g_gm6020_model_origin;
Gm6020TraceSample g_gm6020_trace[GM6020_TRACE_CAPACITY];
uint32_t g_gm6020_trace_count;
volatile uint32_t g_gm6020_trace_version;

// The ISR only publishes raw data. Floating point decoding and all control run
// in the Gimbal task. Copying under PRIMASK prevents a torn feedback sample.
static volatile uint8_t s_rx_data[8];
static volatile uint32_t s_rx_count;
static volatile uint32_t s_rx_time;
static uint32_t s_ready_ticks;
static uint32_t s_last_loop;
static uint32_t s_start_time;
static float s_origin;

static void Gm6020TestReceive(uint32_t id, uint8_t *data)
{
  const FDCAN_RxHeaderTypeDef *header = &g_fdcan_object[0].rx_header;
  if (id != kMotorRxId || header->IdType != FDCAN_STANDARD_ID ||
      header->RxFrameType != FDCAN_DATA_FRAME ||
      header->DataLength != FDCAN_DLC_BYTES_8)
  {
    return;
  }
  // Reject invalid encoder values rather than treating corrupt data as motion.
  if ((((uint32_t)data[0] << 8) | data[1]) > 8191U)
  {
    return;
  }
  for (uint8_t i = 0U; i < 8U; i++)
  {
    s_rx_data[i] = data[i];
  }
  s_rx_time = HAL_GetTick();
  s_rx_count++;
}

static void Gm6020TestFeedback(uint32_t now)
{
  uint8_t data[8];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t count = s_rx_count;
  uint32_t time = s_rx_time;
  for (uint8_t i = 0U; i < 8U; i++)
  {
    data[i] = s_rx_data[i];
  }
  __set_PRIMASK(primask);

  if (count != g_gm6020_test.rx_count)
  {
    DjiMotorUpdate(&g_gm6020_motor, data);
    g_gm6020_test.rx_count = count;
  }
  g_gm6020_test.feedback_age_ms = now - time;
  g_gm6020_test.online = count != 0U && now - time < kFeedbackTimeoutMs;
  g_gm6020_test.position = g_gm6020_motor.state.continuous_angle - s_origin;
  g_gm6020_test.omega = g_gm6020_motor.state.omega;
  g_gm6020_test.current = g_gm6020_motor.state.current;
  if (g_gm6020_test.online && fabsf(g_gm6020_test.omega) < 0.2f)
  {
    if (s_ready_ticks < kReadyTimeMs)
    {
      s_ready_ticks++;
    }
  }
  else
  {
    s_ready_ticks = 0U;
  }
  g_gm6020_test.ready = s_ready_ticks >= kReadyTimeMs;
}

static void Gm6020TestAbortPending(void)
{
  // CAN1 is dedicated to this bench test. Prevent queued nonzero commands from
  // being retransmitted after a stop/fault and a later bus reconnection.
  (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, 0xFFFFFFFFU);
}

static void Gm6020TestFail(Gm6020TestFault fault)
{
  MotorTuneCancel(&g_gm6020_tune);
  Gm6020TestAbortPending();
  g_gm6020_test.fault = fault;
}

static HAL_StatusTypeDef Gm6020TestSend(float current)
{
  uint8_t data[8];
  DjiMotor *motors[] = {&g_gm6020_motor};
  g_gm6020_motor.state.given_current = current;
  DjiMotorPackCurrentGroup(motors, 1U, kMotorTxId, data);
  FDCAN_TxHeaderTypeDef header = g_fdcan_object[0].tx_header;
  header.Identifier = kMotorTxId;
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data);
  if (status == HAL_OK)
  {
    int16_t units = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    g_gm6020_test.sent_current =
        DjiMotorCurrentUnitsToAmperes(kDjiMotorGm6020, units);
  }
  else
  {
    g_gm6020_test.tx_error_count++;
  }
  return status;
}

static void Gm6020TestStartRun(uint32_t now)
{
  if (MotorTuneIsActive(&g_gm6020_tune))
  {
    return; // A second start command must not reset an active experiment.
  }
  if (!g_gm6020_test.ready)
  {
    g_gm6020_test.fault = kGm6020TestNotReady;
    return;
  }
  // Keep the hardware current range and the actual task period consistent with
  // the algorithm. Do not allow a source configuration to silently exceed them.
  if (g_app_task_config.gimbal_period_ms != 1U ||
      g_gm6020_tune_config.plant.sampling_time != 0.001f ||
      g_gm6020_tune_config.plant.current_limit > kDjiGm6020MotorCurrentLimitA ||
      !MotorTuneInit(&g_gm6020_tune, &g_gm6020_tune_config))
  {
    g_gm6020_test.fault = kGm6020TestInvalidConfig;
    return;
  }
  s_origin = g_gm6020_motor.state.continuous_angle;
  g_gm6020_test.position = 0.0f;
  g_gm6020_test.exported = false;
  memset(&g_gm6020_speed_config, 0, sizeof(g_gm6020_speed_config));
  memset(&g_gm6020_position_config, 0, sizeof(g_gm6020_position_config));
  memset(&g_gm6020_load_model, 0, sizeof(g_gm6020_load_model));
  g_gm6020_model_origin = s_origin;
  if (!MotorTuneStart(&g_gm6020_tune, 0.0f, g_gm6020_test.omega))
  {
    g_gm6020_test.fault = kGm6020TestInvalidConfig;
    return;
  }
  g_gm6020_test.fault = kGm6020TestNoFault;
  g_gm6020_test.pareto_index = 0U;
  g_gm6020_test.elapsed_ms = 0U;
  g_gm6020_test.identification_peak_speed = 0.0f;
  g_gm6020_test.identification_peak_current = 0.0f;
  g_gm6020_trace_count = 0U;
  s_start_time = now;
}

void Gm6020TestInit(void)
{
  // Explicitly reference this marker so --gc-sections retains it for export.
  g_gm6020_trace_version = 0x474D0001U;
  PidConfig initial_pid = {.sampling_time = 0.001f, .output_max = 0.3f};
  DjiMotorInit(&g_gm6020_motor, kDjiMotorGm6020, kDjiMotorModeSpeed,
               &initial_pid, NULL);
  g_gm6020_motor.state.id = kMotorRxId; // Driver ID means feedback CAN ID.
  if (!MotorTuneInit(&g_gm6020_tune, &g_gm6020_tune_config))
  {
    g_gm6020_test.fault = kGm6020TestInvalidConfig;
  }
  FdcanInit(&hfdcan1, Gm6020TestReceive);
  s_last_loop = HAL_GetTick();
}

void Gm6020TestLoop(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t delta = now - s_last_loop;
  s_last_loop = now;
  Gm6020TestFeedback(now);

  uint32_t command = g_gm6020_test.command;
  g_gm6020_test.command = kGm6020TestNoCommand;
  if (command == kGm6020TestStop)
  {
    MotorTuneCancel(&g_gm6020_tune);
    Gm6020TestAbortPending();
  }
  else if (command == kGm6020TestStart)
  {
    Gm6020TestStartRun(now);
  }
  else if (command == kGm6020TestExport)
  {
    g_gm6020_test.exported = g_gm6020_test.pareto_index < 256U &&
        MotorTuneExportController(&g_gm6020_tune, (uint8_t)g_gm6020_test.pareto_index,
                                   &g_gm6020_speed_config, &g_gm6020_position_config,
                                   &g_gm6020_load_model);
    if (!g_gm6020_test.exported)
    {
      g_gm6020_test.fault = kGm6020TestExportFailed;
    }
    else if (g_gm6020_test.fault == kGm6020TestExportFailed)
    {
      g_gm6020_test.fault = kGm6020TestNoFault;
    }
  }

  bool was_active = MotorTuneIsActive(&g_gm6020_tune);
  float request = 0.0f;
  if (was_active)
  {
    if (g_gm6020_tune.state == kMotorTuneIdentify)
    {
      uint32_t count = g_gm6020_trace_count;
      uint32_t elapsed = now - s_start_time;
      if (count < GM6020_TRACE_CAPACITY &&
          (count == 0U || elapsed - g_gm6020_trace[count - 1U].time_ms >= 8U))
      {
        g_gm6020_trace[count] = (Gm6020TraceSample){
            .time_ms = elapsed,
            .rx_count = g_gm6020_test.rx_count,
            .feedback_age_ms = g_gm6020_test.feedback_age_ms,
            .position = g_gm6020_test.position,
            .omega = g_gm6020_test.omega,
            .current = g_gm6020_test.current,
            .previous_command_current = g_gm6020_test.sent_current};
        g_gm6020_trace_count = count + 1U;
      }
      g_gm6020_test.identification_peak_speed = fmaxf(
          g_gm6020_test.identification_peak_speed, fabsf(g_gm6020_test.omega));
      g_gm6020_test.identification_peak_current = fmaxf(
          g_gm6020_test.identification_peak_current, fabsf(g_gm6020_test.current));
    }
    g_gm6020_test.elapsed_ms = now - s_start_time;
    if (!g_gm6020_test.online)
    {
      Gm6020TestFail(kGm6020TestFeedbackLost);
    }
    else if (delta > 3U)
    {
      Gm6020TestFail(kGm6020TestTimingFault);
    }
    else if (now - s_start_time >= kTestTimeoutMs)
    {
      Gm6020TestFail(kGm6020TestTimeout);
    }
    else if (delta != 0U)
    {
      request = MotorTuneUpdate(&g_gm6020_tune, g_gm6020_test.position,
                                 g_gm6020_test.omega, g_gm6020_test.current);
    }
    else
    {
      return; // Do not integrate twice in the same millisecond after an overrun.
    }
  }
  if (!isfinite(request))
  {
    Gm6020TestFail(kGm6020TestInvalidOutput);
    request = 0.0f;
  }
  if (!MotorTuneIsActive(&g_gm6020_tune))
  {
    request = 0.0f;
    if (was_active)
    {
      Gm6020TestAbortPending();
    }
  }
  g_gm6020_test.requested_current = request;
  float limit = g_gm6020_tune_config.plant.current_limit;
  float executed = fminf(fmaxf(request, -limit), limit);
  if (Gm6020TestSend(executed) != HAL_OK)
  {
    if (MotorTuneIsActive(&g_gm6020_tune))
    {
      Gm6020TestFail(kGm6020TestCanTxFailed);
    }
    Gm6020TestAbortPending();
    (void)Gm6020TestSend(0.0f);
  }
  else if (MotorTuneIsActive(&g_gm6020_tune))
  {
    // Track the quantized command actually queued on CAN, never the measured
    // current or the unquantized request. A TX failure cancels the test instead.
    (void)MotorTuneTrackOutput(&g_gm6020_tune, g_gm6020_test.sent_current);
  }

  g_gm6020_test.active = MotorTuneIsActive(&g_gm6020_tune);
  g_gm6020_test.state = g_gm6020_tune.state;
  g_gm6020_test.error = g_gm6020_tune.error;
  g_gm6020_test.identification_error = g_gm6020_tune.baseline.error;
  g_gm6020_test.evaluated_count = g_gm6020_tune.result.evaluated_count;
  g_gm6020_test.pareto_count = g_gm6020_tune.result.pareto_count;
  g_gm6020_test.model_windows = g_gm6020_tune.baseline.sysid.count;
  g_gm6020_test.model_fitness = g_gm6020_tune.baseline.result.model.fitness;
  g_gm6020_test.model_inertia = g_gm6020_tune.baseline.result.model.inertia;
  g_gm6020_test.model_viscous = g_gm6020_tune.baseline.result.model.viscous;
  if (g_gm6020_tune.baseline.state == kAutotuneIdentify)
    g_gm6020_test.identification_current = g_gm6020_tune.baseline.test_amplitude;
  g_gm6020_test.trace_samples = g_gm6020_trace_count;
  g_gm6020_test.identification_motion_ratio =
      g_gm6020_tune.baseline.excitation_motion_ratio;
  g_gm6020_test.model_validation_windows = g_gm6020_tune.baseline.sysid.validation_count;
  g_gm6020_test.model_position_span = g_gm6020_tune.baseline.sysid.position_max -
                                      g_gm6020_tune.baseline.sysid.position_min;
  g_gm6020_test.baseline_state = g_gm6020_tune.baseline.state;
  g_gm6020_test.validation_rms = g_gm6020_tune.baseline.result.rms_error;
  g_gm6020_test.validation_peak = g_gm6020_tune.baseline.result.peak_error;
  bool baseline = g_gm6020_tune.state == kMotorTuneIdentify ||
                  g_gm6020_tune.error == kMotorTuneBaselineValidationFailed;
  g_gm6020_test.speed_reference = baseline ? g_gm6020_tune.baseline.speed_pid.reference
                                           : g_gm6020_tune.speed_pid.reference;
  g_gm6020_test.pid_current = baseline ? g_gm6020_tune.baseline.speed_pid.output
                                       : g_gm6020_tune.speed_pid.output;
  g_gm6020_test.load_current = g_gm6020_tune.load_current;
  g_gm6020_test.candidate_index = g_gm6020_tune.candidate_index;
  g_gm6020_test.segment = g_gm6020_tune.segment;
  g_gm6020_test.guard_state = g_gm6020_tune.guard_state;
  g_gm6020_test.guard_rms_speed_error = g_gm6020_tune.guard_rms_speed_error;
  g_gm6020_test.guard_rms_current_slew = g_gm6020_tune.guard_rms_current_slew;
  g_gm6020_test.guard_reversals = g_gm6020_tune.guard_reversals;
  g_gm6020_test.stop_state = g_gm6020_tune.stop_state;
  g_gm6020_test.prepare_braking = g_gm6020_tune.prepare_braking;
  g_gm6020_test.prepare_reference = g_gm6020_tune.prepare_reference;
  if (!g_gm6020_test.active)
  {
    g_gm6020_test.terminal_position = g_gm6020_tune.last_position;
    g_gm6020_test.terminal_omega = g_gm6020_tune.last_omega;
  }
}
