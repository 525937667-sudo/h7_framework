#include "alg_sysid.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 初始化固定最大尺寸的积分辨识器，默认使用四个参数
 * @param sysid 辨识器对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool SysidInit(SysidObject *sysid, const SysidConfig *config)
{
  if (sysid == NULL || config == NULL || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || !isfinite(config->window_time) ||
      config->window_time < config->sampling_time ||
      !isfinite(config->current_scale) || config->current_scale <= 0.0f ||
      !isfinite(config->omega_scale) || config->omega_scale <= 0.0f ||
      !isfinite(config->memory_time) || config->memory_time < 0.0f ||
      (config->method != kSysidExponential && config->method != kSysidSubspace))
  {
    return false;
  }
  uint8_t n = 4U;
  bool ended = false;
  for (uint8_t i = 0U; i < SYSID_MAX_POSITION_HARMONICS; i++)
  {
    float h = config->position_harmonics[i];
    if (!isfinite(h) || h < 0.0f || h > 128.0f || (ended && h != 0.0f))
      return false;
    if (h == 0.0f) { ended = true; continue; }
    for (uint8_t j = 0U; j < i; j++)
      if (h == config->position_harmonics[j]) return false;
    n += 2U;
  }
  memset(sysid, 0, sizeof(*sysid));
  sysid->parameter_count = n;
  sysid->config = *config;
  for (int i = 0; i < n; i++)
  {
    sysid->covariance[i][i] = 10000.0f;
    sysid->information[i][i] = 0.0001;
  }
  return true;
}

/**
 * @brief 计算角速度方向，零速处使用连续过渡
 * @param omega 角速度（rad/s）
 * @param scale 角速度尺度（rad/s）
 * @return 摩擦方向
 */
static float SysidDirection(float omega, float scale)
{
  return fminf(fmaxf(omega / (0.01f * scale), -1.0f), 1.0f);
}

/**
 * @brief 用Cholesky分解计算当前有效维数的信息矩阵逆
 * @param information 信息矩阵
 * @param covariance 协方差矩阵
 * @return 是否为有限正定矩阵
 */
static bool SysidInvertInformation(const double information[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS],
                                   double covariance[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS],
                                   double lower[SYSID_MAX_PARAMETERS][SYSID_MAX_PARAMETERS], uint8_t n)
{
  memset(lower, 0, sizeof(double) * SYSID_MAX_PARAMETERS * SYSID_MAX_PARAMETERS);
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j <= i; j++)
    {
      double value = information[i][j];
      for (int k = 0; k < j; k++)
      {
        value -= lower[i][k] * lower[j][k];
      }
      if (!isfinite(value) || (i == j && value <= 0.0))
      {
        return false;
      }
      lower[i][j] = i == j ? sqrt(value) : value / lower[j][j];
    }
  }
  for (int column = 0; column < n; column++)
  {
    double value[SYSID_MAX_PARAMETERS] = {0};
    for (int i = 0; i < n; i++)
    {
      double sum = i == column ? 1.0 : 0.0;
      for (int j = 0; j < i; j++)
      {
        sum -= lower[i][j] * value[j];
      }
      value[i] = sum / lower[i][i];
    }
    for (int i = n - 1; i >= 0; i--)
    {
      double sum = value[i];
      for (int j = i + 1; j < n; j++)
      {
        sum -= lower[j][i] * value[j];
      }
      value[i] = sum / lower[i][i];
      covariance[i][column] = value[i];
      if (!isfinite(covariance[i][column]))
      {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 按标量观测执行SIFt-RLS，不使用通用SVD
 * @param sysid 辨识器对象指针
 * @param x 归一化回归向量
 * @param error 一步预测误差
 * @param decay 遗忘系数
 * @return 是否更新成功
 */
static bool SysidSubspaceUpdate(SysidObject *sysid, const float x[SYSID_MAX_PARAMETERS],
                                float error, float decay)
{
  uint8_t n = sysid->parameter_count;
  double rx[SYSID_MAX_PARAMETERS] = {0};
  double norm = 0.0;
  double projection = 0.0;
  for (int i = 0; i < n; i++)
  {
    norm += (double)x[i] * x[i];
    for (int j = 0; j < n; j++)
    {
      rx[i] += sysid->information[i][j] * x[j];
    }
    projection += x[i] * rx[i];
  }
  if (norm < 1.0e-8 || !isfinite(projection) || projection <= 0.0)
  {
    return false;
  }
  double (*information)[SYSID_MAX_PARAMETERS] = sysid->work_information;
  double (*covariance)[SYSID_MAX_PARAMETERS] = sysid->work_covariance;
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      information[i][j] = sysid->information[i][j] -
                          (1.0 - decay) * rx[i] * rx[j] / projection +
                          (double)x[i] * x[j];
    }
  }
  if (!SysidInvertInformation(information, covariance, sysid->work_lower, n))
  {
    return false;
  }
  for (int i = 0; i < n; i++)
  {
    double gain = 0.0;
    for (int j = 0; j < n; j++)
    {
      gain += covariance[i][j] * x[j];
    }
    sysid->coefficient[i] += (float)(gain * error);
    for (int j = 0; j < n; j++)
    {
      sysid->covariance[i][j] = (float)covariance[i][j];
    }
  }
  memcpy(sysid->information, information, sizeof(sysid->information));
  return true;
}

/**
 * @brief 用非重叠积分窗口更新电流坐标机械模型
 * @param sysid 辨识器对象指针
 * @param current 实际反馈电流（A）
 * @param omega 与控制器一致的角速度（rad/s）
 * @return 是否完成一个辨识窗口
 */
bool SysidUpdate(SysidObject *sysid, float current, float omega)
{
  if (sysid == NULL || sysid->parameter_count > 4U) return false;
  return SysidUpdateWithPosition(sysid, current, omega, 0.0f);
}

bool SysidUpdateWithPosition(SysidObject *sysid, float current, float omega,
                             float position)
{
  if (sysid == NULL || !isfinite(current) || !isfinite(omega) || !isfinite(position))
  {
    return false;
  }
  if (!sysid->has_sample)
  {
    sysid->has_sample = true;
    sysid->start_omega = omega;
    sysid->previous_omega = omega;
    sysid->previous_current = current;
    sysid->previous_position = position;
    sysid->position_min = position;
    sysid->position_max = position;
    return false;
  }
  uint8_t n = sysid->parameter_count;
  const SysidConfig *config = &sysid->config;
  sysid->position_min = fminf(sysid->position_min, position);
  sysid->position_max = fmaxf(sysid->position_max, position);
  float dt = config->sampling_time;
  sysid->integral[0] += 0.5f * (current + sysid->previous_current) * dt;
  sysid->integral[1] += 0.5f * (omega + sysid->previous_omega) * dt;
  sysid->integral[2] +=
      0.5f *
      (SysidDirection(omega, config->omega_scale) +
       SysidDirection(sysid->previous_omega, config->omega_scale)) *
      dt;
  for (uint8_t h = 0U; 4U + 2U * h < n; h++)
  {
    float order = config->position_harmonics[h];
    sysid->integral[3U + 2U * h] += 0.5f *
        (sinf(order * position) + sinf(order * sysid->previous_position)) * dt;
    sysid->integral[4U + 2U * h] += 0.5f *
        (cosf(order * position) + cosf(order * sysid->previous_position)) * dt;
  }
  sysid->previous_position = position;
  sysid->previous_omega = omega;
  sysid->previous_current = current;
  sysid->elapsed += dt;
  if (sysid->elapsed + 0.25f * dt < config->window_time)
  {
    return false;
  }

  float x[SYSID_MAX_PARAMETERS] = {(omega - sysid->start_omega) / config->omega_scale,
                sysid->integral[1] /
                    (config->omega_scale * config->window_time),
                sysid->integral[2] / config->window_time,
                sysid->elapsed / config->window_time};
  for (uint8_t h = 0U; 4U + 2U * h < n; h++)
  {
    x[4U + 2U * h] = sysid->integral[3U + 2U * h] / config->window_time;
    x[5U + 2U * h] = sysid->integral[4U + 2U * h] / config->window_time;
  }
  float y = sysid->integral[0] / (config->current_scale * config->window_time);
  float prediction = 0.0f;
  float px[SYSID_MAX_PARAMETERS] = {0};
  for (int i = 0; i < n; i++)
  {
    prediction += x[i] * sysid->coefficient[i];
    for (int j = 0; j < n; j++)
    {
      px[i] += sysid->covariance[i][j] * x[j];
    }
  }
  float decay = config->memory_time > 0.0f
                    ? expf(-sysid->elapsed / config->memory_time)
                    : 1.0f;
  decay = fmaxf(decay, 0.98f);
  float denominator = decay;
  for (int i = 0; i < n; i++)
  {
    denominator += x[i] * px[i];
  }
  float error = y - prediction;
  sysid->window_count++;
  // Reserve every fifth window for a fit check, never for coefficient updates.
  // Quadratic statistics evaluate the CURRENT model on held-out observations.
  bool held_out = n > 4U && sysid->count >= 3U * n && sysid->window_count % 5U == 0U;
  if (held_out)
  {
    for (uint8_t i = 0U; i < n; i++)
    {
      sysid->validation_cross[i] += (double)x[i] * y;
      for (uint8_t j = 0U; j < n; j++)
        sysid->validation_information[i][j] += (double)x[i] * x[j];
    }
    sysid->validation_energy += (double)y * y;
    sysid->validation_count++;
    memset(sysid->integral, 0, sizeof(sysid->integral));
    sysid->elapsed = 0.0f;
    sysid->start_omega = omega;
    return true;
  }
  bool updated = false;
  if (config->method == kSysidSubspace)
  {
    updated = SysidSubspaceUpdate(sysid, x, error, decay);
  }
  // Steady moving windows constrain viscous drag and friction even when
  // delta-omega is zero. Gating only on x[0] discards those observations and,
  // with quantized speed, selects samples based on endpoint measurement noise.
  // In the periodic model, holding-position windows also constrain the
  // position-dependent torque. Inertia still needs dynamic excitation,
  // independently checked in SysidGetModel().
  else if ((n > 4U || fabsf(x[0]) > 0.0002f || fabsf(x[1]) > 0.0002f) &&
           isfinite(denominator) &&
           denominator > 1.0e-9f)
  {
    for (int i = 0; i < n; i++)
    {
      sysid->coefficient[i] += px[i] * error / denominator;
      for (int j = i; j < n; j++)
      {
        float value =
            (sysid->covariance[i][j] - px[i] * px[j] / denominator) / decay;
        if (i == j)
        {
          value = fmaxf(value, 1.0e-9f);
        }
        sysid->covariance[i][j] = value;
        sysid->covariance[j][i] = value;
      }
    }
    // 单向运动时摩擦与偏置可能共线，限制未观测方向的协方差增长。
    float scale[SYSID_MAX_PARAMETERS];
    for (int i = 0; i < n; i++)
    {
      scale[i] = sqrtf(fminf(10000.0f / sysid->covariance[i][i], 1.0f));
    }
    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < n; j++)
      {
        sysid->covariance[i][j] *= scale[i] * scale[j];
      }
    }
    updated = true;
  }
  if (updated)
  {
    // 初始收敛窗口不参与一步预测质量统计。
    if (sysid->count >= 12U)
    {
      sysid->energy = decay * sysid->energy + y * y;
      sysid->residual = decay * sysid->residual + error * error;
    }
    sysid->excitation = decay * sysid->excitation + x[0] * x[0];
    sysid->count++;
  }
  memset(sysid->integral, 0, sizeof(sysid->integral));
  sysid->elapsed = 0.0f;
  sysid->start_omega = omega;
  return true;
}

/**
 * @brief 读取具有足够动态激励的机械模型
 * @param sysid 辨识器对象指针
 * @param model 模型输出指针
 * @return 模型是否可用于整定
 */
bool SysidGetModel(const SysidObject *sysid, SysidModel *model)
{
  if (sysid == NULL || model == NULL)
  {
    return false;
  }
  model->inertia = sysid->coefficient[0] * sysid->config.current_scale *
                   sysid->config.window_time / sysid->config.omega_scale;
  model->viscous = sysid->coefficient[1] * sysid->config.current_scale /
                   sysid->config.omega_scale;
  model->friction = sysid->coefficient[2] * sysid->config.current_scale;
  model->bias = sysid->coefficient[3] * sysid->config.current_scale;
  model->fitness = 1.0f - sysid->residual / fmaxf(sysid->energy, 1.0e-12f);
  uint8_t n = sysid->parameter_count;
  bool periodic_valid = true;
  for (uint8_t h = 0U; h < SYSID_MAX_POSITION_HARMONICS; h++)
  {
    model->position_harmonics[h] = sysid->config.position_harmonics[h];
    model->position_sine[h] = sysid->coefficient[4U + 2U * h] * sysid->config.current_scale;
    model->position_cosine[h] = sysid->coefficient[5U + 2U * h] * sysid->config.current_scale;
    periodic_valid = periodic_valid && isfinite(model->position_sine[h]) &&
                     isfinite(model->position_cosine[h]);
  }
  if (n > 4U)
  {
    double residual = sysid->validation_energy;
    for (uint8_t i = 0U; i < n; i++)
    {
      residual -= 2.0 * sysid->coefficient[i] * sysid->validation_cross[i];
      for (uint8_t j = 0U; j < n; j++)
        residual += (double)sysid->coefficient[i] * sysid->coefficient[j] *
                    sysid->validation_information[i][j];
    }
    model->fitness = 1.0f - (float)(fmax(residual, 0.0) /
                                  fmax(sysid->validation_energy, 1.0e-12));
    periodic_valid = periodic_valid && sysid->validation_count >= 2U * n &&
                     sysid->validation_energy > 1.0e-8;
    // Do not infer a global periodic torque map from only a tiny angle arc.
    for (uint8_t h = 0U; h < SYSID_MAX_POSITION_HARMONICS; h++)
      if (sysid->config.position_harmonics[h] > 0.0f &&
          (sysid->position_max - sysid->position_min) *
              sysid->config.position_harmonics[h] < 6.2831853f)
        periodic_valid = false;
  }
  model->count = sysid->count;
  return periodic_valid && model->count >= 10U * n && sysid->excitation > 1.0e-4f &&
         sysid->covariance[0][0] < 100.0f && isfinite(model->inertia) &&
         model->inertia > 0.0f && isfinite(model->viscous) &&
         model->viscous >= 0.0f && isfinite(model->friction) &&
         isfinite(model->bias) && isfinite(model->fitness) &&
         model->fitness >= 0.75f;
}

// Feed forward only load terms; inertia/viscous dynamics remain in the PI plant.
float SysidLoadCurrent(const SysidModel *model, float position,
                       float reference_omega, float transition_speed)
{
  if (model == NULL || !isfinite(position) || !isfinite(reference_omega) ||
      !isfinite(transition_speed) || transition_speed <= 0.0f ||
      !isfinite(model->bias) || !isfinite(model->friction)) return NAN;
  float direction = fminf(fmaxf(reference_omega / transition_speed, -1.0f), 1.0f);
  float current = model->bias + model->friction * direction;
  for (uint8_t h = 0U; h < SYSID_MAX_POSITION_HARMONICS; h++)
  {
    if (!isfinite(model->position_harmonics[h]) || model->position_harmonics[h] < 0.0f ||
        !isfinite(model->position_sine[h]) || !isfinite(model->position_cosine[h]))
      return NAN;
    if (model->position_harmonics[h] > 0.0f)
      current += model->position_sine[h] * sinf(model->position_harmonics[h] * position) +
                 model->position_cosine[h] * cosf(model->position_harmonics[h] * position);
  }
  return current;
}
