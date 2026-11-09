#ifndef APP_IMU_H
#define APP_IMU_H

#include "dev_bmi088.h"

// Manager任务发布的学习/调试结果；内部EKF与传感器数据在g_bmi088中。
typedef struct
{
  bool initialized;             // 完成传感器、静止标定和EKF初始化
  bool valid;                   // 最近一次更新成功；不表示绝对航向有效
  Bmi088Status status;          // 最近一次初始化/更新返回值
  Euler angle_deg;              // 仅用于显示，单位deg；控制计算使用rad
  uint32_t update_count;        // 成功更新次数
  uint32_t error_count;         // 运行期更新失败次数
  uint32_t delayed_count;       // 采样间隔超过配置任务周期的次数
  uint32_t sample_interval_ms;  // 最近两次尝试更新之间的实际间隔
  uint32_t last_success_ms;     // 最近成功更新的HAL毫秒时间戳
} AppImuState;

extern AppImuState g_app_imu;

#endif // APP_IMU_H
