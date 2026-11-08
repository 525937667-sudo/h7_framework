#ifndef ALG_SYSID_H
#define ALG_SYSID_H

#include <stdbool.h>
#include <stdint.h>

#define SYSID_MAX_POSITION_HARMONICS 3U
#define SYSID_MAX_PARAMETERS (4U + 2U * SYSID_MAX_POSITION_HARMONICS)

// 辨识更新方法
typedef enum
{
  kSysidExponential = 0, // 指数遗忘RLS
  kSysidSubspace,        // 信息子空间遗忘RLS
} SysidMethod;

// 积分机械模型配置结构体
typedef struct
{
  float sampling_time; // 采样时间（s）
  float window_time;   // 积分窗口（s）
  float current_scale; // 电流归一化尺度（A）
  float omega_scale;   // 角速度归一化尺度（rad/s）
  float memory_time;   // 递推记忆时间（s），0表示不遗忘
  SysidMethod method;  // 辨识更新方法
  float position_harmonics[SYSID_MAX_POSITION_HARMONICS]; // 每圈阶次，0禁用
} SysidConfig;

// 电流坐标机械模型结构体
typedef struct
{
  float inertia;  // 等效J/Kt
  float viscous;  // 等效b/Kt
  float friction; // 库仑摩擦电流（A）
  float bias;     // 当前工况恒定负载电流（A）
  float fitness;  // 一步预测拟合度
  uint32_t count; // 有效积分窗口数
  float position_harmonics[SYSID_MAX_POSITION_HARMONICS];
  float position_sine[SYSID_MAX_POSITION_HARMONICS]; // 周期扰动电流系数，A
  float position_cosine[SYSID_MAX_POSITION_HARMONICS];
} SysidModel;

// 固定尺寸递推辨识结构体
typedef struct
{
  SysidConfig config;
  float coefficient[SYSID_MAX_PARAMETERS];
  float covariance[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS];
  double information[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS];
  // Bounded SIFt scratch space belongs to the statically allocated object,
  // keeping expanded matrix calculations off the 1 ms task stack.
  double work_information[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS];
  double work_covariance[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS];
  double work_lower[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS];
  float integral[3U + 2U * SYSID_MAX_POSITION_HARMONICS];
  double validation_information[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS];
  double validation_cross[SYSID_MAX_PARAMETERS];
  double validation_energy;
  uint32_t validation_count;
  uint32_t window_count;
  uint8_t parameter_count;
  float previous_position;
  float position_min;
  float position_max;
  float elapsed;
  float start_omega;
  float previous_omega;
  float previous_current;
  float energy;
  float residual;
  float excitation;
  uint32_t count;
  bool has_sample;
} SysidObject;

bool SysidInit(SysidObject *sysid, const SysidConfig *config);
bool SysidUpdate(SysidObject *sysid, float current, float omega);
bool SysidUpdateWithPosition(SysidObject *sysid, float current, float omega,
                             float position);
bool SysidGetModel(const SysidObject *sysid, SysidModel *model);
// Load feedforward in A, using the same continuous position origin as fitting.
// transition_speed smooths the reference-direction friction term near zero.
// Invalid inputs return NAN; the caller must limit the total PI + load request.
float SysidLoadCurrent(const SysidModel *model, float position,
                       float reference_omega, float transition_speed);

#endif // ALG_SYSID_H
