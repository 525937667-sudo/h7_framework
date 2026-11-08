#ifndef APP_GM6020_TEST_H
#define APP_GM6020_TEST_H

#include "alg_motor_tune.h"
#include "dev_dji_motor.h"

#define GM6020_TRACE_CAPACITY 6400U

// 8 ms samples captured during baseline identification and validation, frozen
// on exit. The command is the previous queued command at feedback sampling time.
typedef struct
{
  uint32_t time_ms;
  uint32_t rx_count;
  uint32_t feedback_age_ms;
  float position;
  float omega;
  float current;
  float previous_command_current;
} Gm6020TraceSample;

typedef enum
{
  kGm6020TestNoCommand = 0,
  kGm6020TestStart = 1,
  kGm6020TestStop = 2,
  kGm6020TestExport = 3,
} Gm6020TestCommand;

typedef enum
{
  kGm6020TestNoFault = 0,
  kGm6020TestNotReady,
  kGm6020TestInvalidConfig,
  kGm6020TestFeedbackLost,
  kGm6020TestCanTxFailed,
  kGm6020TestTimingFault,
  kGm6020TestTimeout,
  kGm6020TestInvalidOutput,
  kGm6020TestExportFailed,
} Gm6020TestFault;

// Debugger entry: write command=1 to start, 2 to stop, 3 to export.
// All motion is owned by the 1 ms Gimbal task. Boot never starts a test.
typedef struct
{
  volatile uint32_t command;
  uint32_t pareto_index; // Index in result.pareto[], not candidate[].
  Gm6020TestFault fault;
  bool online;
  bool ready;
  bool active;
  bool exported;
  uint32_t rx_count;
  uint32_t feedback_age_ms;
  uint32_t elapsed_ms;
  uint32_t tx_error_count;
  float position; // Continuous angle relative to the start of this test, rad.
  float omega;    // rad/s
  float current;  // Measured feedback, A.
  float requested_current;
  float sent_current; // Quantized CAN command, A; not measured current.
  MotorTuneState state;
  MotorTuneError error;
  AutotuneError identification_error;
  uint8_t evaluated_count;
  uint8_t pareto_count;
  uint32_t model_windows;
  float model_fitness;
  float model_inertia;
  float model_viscous;
  float identification_current;      // Current pulse amplitude, A.
  float identification_peak_speed;   // Peak |omega| during identification, rad/s.
  float identification_peak_current; // Peak measured |current|, A.
  uint32_t trace_samples;
  float identification_motion_ratio; // Fraction moving in last 0.5 s interval.
  uint32_t model_validation_windows;
  float model_position_span;
  AutotuneState baseline_state;
  float validation_rms;
  float validation_peak;
  float speed_reference;
  float pid_current;
  float load_current;
  uint32_t candidate_index; // Zero-based candidate index, meaningful in search/validation.
  uint32_t segment;
  MotorTuneState guard_state; // Phase frozen when oscillation protection trips.
  float guard_rms_speed_error;
  float guard_rms_current_slew;
  uint32_t guard_reversals;
  MotorTuneState stop_state;
  float terminal_position;
  float terminal_omega;
  bool prepare_braking;
  float prepare_reference;
} Gm6020TestStatus;

extern Gm6020TestStatus g_gm6020_test;
extern MotorTuneObject g_gm6020_tune;
extern DjiMotor g_gm6020_motor;
extern const MotorTuneConfig g_gm6020_tune_config;
extern PidConfig g_gm6020_speed_config;
extern PidConfig g_gm6020_position_config;
extern SysidModel g_gm6020_load_model;
extern float g_gm6020_model_origin;
extern Gm6020TraceSample g_gm6020_trace[GM6020_TRACE_CAPACITY];
extern uint32_t g_gm6020_trace_count;
extern volatile uint32_t g_gm6020_trace_version;

void Gm6020TestInit(void);
void Gm6020TestLoop(void);

#endif // APP_GM6020_TEST_H
