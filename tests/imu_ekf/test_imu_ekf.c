#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "alg_imu_ekf.h"

#define CHECK(condition) do { if (!(condition)) { \
  fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #condition, __FILE__, __LINE__); \
  exit(EXIT_FAILURE); } } while (0)

int main(void)
{
  static ImuEkfObject ekf;
  const float gravity = 9.81f;
  const float gyro_zero[3] = {0.0f, 0.0f, 0.0f};
  // 已知倾角的静止重力：检验从错误初值收敛到真实roll/pitch。
  const float roll = 0.35f, pitch = -0.25f;
  const float tilted_accel[3] = {-gravity * sinf(pitch),
      gravity * sinf(roll) * cosf(pitch), gravity * cosf(roll) * cosf(pitch)};
  CHECK(ImuEkfInit(&ekf, 0.001f, gravity) == 0);
  for (unsigned i = 0U; i < 2000U; i++)
    CHECK(ImuEkfStep(&ekf, gyro_zero, tilted_accel) == ARM_MATH_SUCCESS);
  Euler euler;
  ImuEkfGetState(&ekf, &euler, NULL);
  CHECK(fabsf(euler.roll - roll) < 0.005f);
  CHECK(fabsf(euler.pitch - pitch) < 0.005f);

  // 水平匀速绕Z轴旋转，交替1/2ms周期；yaw应为0.5rad/s乘累计时间。
  CHECK(ImuEkfInit(&ekf, 0.001f, gravity) == 0);
  const float level_accel[3] = {0.0f, 0.0f, gravity};
  const float yaw_gyro[3] = {0.0f, 0.0f, 0.5f};
  float elapsed = 0.0f;
  for (unsigned i = 0U; i < 1000U; i++)
  {
    ekf.dt = i % 2U ? 0.002f : 0.001f;
    elapsed += ekf.dt;
    CHECK(ImuEkfStep(&ekf, yaw_gyro, level_accel) == ARM_MATH_SUCCESS);
  }
  ImuEkfGetState(&ekf, &euler, NULL);
  CHECK(fabsf(euler.yaw - 0.5f * elapsed) < 0.001f);
  CHECK(fabsf(euler.roll) < 0.001f && fabsf(euler.pitch) < 0.001f);
  puts("IMU EKF tilt convergence and yaw integration checks passed");
  return 0;
}
