/**
 * @file app_robot.c
 * @brief 机器人具体业务层实现
 * @attention
 * 机器人业务逻辑、控制策略和任务接口在本文件中实现。
 * app_config.h/.c、app_protocol.h/.c 与本文件平级，配置参数和通信协议。
 * 框架暂不支持的业务功能在本文件中实现。
 * 如果后续需要复用，可以在提出Issue后，进行代码的重构和优化。
 */

#include "app_task.h"

#include "app_config.h"
#include "app_gm6020_test.h"
#include "app_imu.h"
#include "spi.h"

AppImuState g_app_imu;
static uint32_t s_imu_last_sample_ms;
static uint32_t s_manager_period_ms;

/**
 * @brief 初始化管理任务
 * @return 任务周期（ms）
 */
uint32_t ManagerTaskInit(void)
{
  // 1. 任务周期使用ms，滤波器dt使用s；与StartManger的零周期处理一致。
  s_manager_period_ms = g_app_task_config.manager_period_ms;
  if (s_manager_period_ms == 0U)
  {
    s_manager_period_ms = 1U;
  }
  Bmi088ImuConfig config;
  Bmi088ImuConfigInit(&config, (float)s_manager_period_ms * 0.001f);

  // 2. 上电保持板子静止，采600次数据估计陀螺零偏和重力模长。
  config.is_auto_calibrate_enabled = true;
  config.calibration_sample_count = 600U;
  config.calibration_sample_delay_ms = 1U;
  // 默认还会采100次加速度，初始化roll/pitch；yaw以0作为起点。
  // install_matrix=NULL默认沿用传感器XYZ轴；本例不启用加热。

  // 3. 喵板板载BMI088连接SPI2，片选由dev_bmi088.c管理。
  g_app_imu = (AppImuState){0};
  g_app_imu.status = Bmi088ImuInit(&g_bmi088, &hspi2, &config);
  g_app_imu.initialized = g_app_imu.status == kBmi088NoError;
  // 时间基准放在初始化后，避免把标定耗时积分进姿态。
  s_imu_last_sample_ms = HAL_GetTick();
  return s_manager_period_ms;
}

/**
 * @brief 执行管理任务
 */
void ManagerTaskLoop(void)
{
  // 4. 初始化失败时保留错误码，修正问题后复位重新标定。
  if (!g_app_imu.initialized)
  {
    return;
  }

  // 5. 使用实际时间间隔，避免任务被抢占后仍固定按1ms积分。
  uint32_t now_ms = HAL_GetTick();
  uint32_t interval_ms = now_ms - s_imu_last_sample_ms;
  if (interval_ms == 0U)
  {
    return; // osDelayUntil追赶时，不在同一毫秒重复积分。
  }
  s_imu_last_sample_ms = now_ms;
  g_app_imu.sample_interval_ms = interval_ms;
  if (interval_ms > s_manager_period_ms)
  {
    g_app_imu.delayed_count++;
  }
  g_bmi088.imu_ekf.dt = (float)interval_ms * 0.001f;

  // 6. SPI采样 -> 单位/安装方向修正 -> EKF预测与校正 -> 欧拉角。
  g_app_imu.status = Bmi088ImuUpdate(&g_bmi088);
  g_app_imu.valid = g_app_imu.status == kBmi088NoError;
  if (!g_app_imu.valid)
  {
    g_app_imu.error_count++;
    return; // 角度保留上次成功值，调用者必须同时检查valid和时间戳。
  }

  // 7. 算法输出是rad，额外转换为deg，便于在Watch中直接观察。
  const float rad_to_deg = 57.2957795f;
  g_app_imu.angle_deg.roll = g_bmi088.euler.roll * rad_to_deg;
  g_app_imu.angle_deg.pitch = g_bmi088.euler.pitch * rad_to_deg;
  g_app_imu.angle_deg.yaw = g_bmi088.euler.yaw * rad_to_deg;
  g_app_imu.update_count = g_bmi088.update_count;
  g_app_imu.last_success_ms = HAL_GetTick();
}

/**
 * @brief 初始化云台任务
 * @return 任务周期（ms）
 */
uint32_t GimbalTaskInit(void)
{
  return g_app_task_config.gimbal_period_ms;
}

/**
 * @brief 执行云台任务
 */
void GimbalTaskLoop(void)
{
  Gm6020TestLoop();
}

/**
 * @brief 初始化底盘任务
 * @return 任务周期（ms）
 */
uint32_t ChassisTaskInit(void)
{
  return g_app_task_config.chassis_period_ms;
}

/**
 * @brief 执行底盘任务
 */
void ChassisTaskLoop(void)
{
}

/**
 * @brief 初始化发射任务
 * @return 任务周期（ms）
 */
uint32_t ShootTaskInit(void)
{
  return g_app_task_config.shoot_period_ms;
}

/**
 * @brief 执行发射任务
 */
void ShootTaskLoop(void)
{
}
