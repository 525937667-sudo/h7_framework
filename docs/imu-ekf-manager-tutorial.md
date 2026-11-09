# 喵板板载 BMI088：在 StartManger 中用 EKF 求姿态角

这份文档对应本工程实际接入的代码。先让板子输出角度，再顺着函数调用理解扩展卡尔曼滤波（EKF）的每一步。

本例使用喵板自己的 BMI088：SPI2、加速度计片选 PC0、陀螺仪片选 PC3。任务中自动读取并融合传感器数据，输出横滚角 roll、俯仰角 pitch 和相对偏航角 yaw。BMI088 提供加速度和角速度，角度是 STM32 上的算法估计出来的。

## 1. 先找到改在哪里

| 文件 | 作用 |
| --- | --- |
| [app_robot.c](../Skywalker/Application/Src/app_robot.c) | 本次接入代码；`ManagerTaskInit()` 初始化，`ManagerTaskLoop()` 周期更新，注释按 1～7 编号 |
| [app_imu.h](../Skywalker/Application/Inc/app_imu.h) | 发布 `g_app_imu`，方便看角度、状态、计数和时间戳 |
| [app_task.c](../Skywalker/Application/Src/app_task.c) | 真正的 `StartManger()` 入口和周期调度 |
| [dev_bmi088.c](../Skywalker/Device/Src/dev_bmi088.c) | SPI 读取、标定、单位转换、安装矩阵和 EKF 调用 |
| [alg_imu_ekf.c](../Skywalker/Algorithm/Src/alg_imu_ekf.c) | 六状态 EKF 的预测、校正及欧拉角输出 |

工程的任务入口确实拼作 `StartManger`，不是 `StartManager`。调用关系如下：

```text
StartManger()
  ├─ ManagerTaskInit()                 只执行一次
  │    └─ Bmi088ImuInit()
  │         ├─ 初始化 BMI088
  │         ├─ 初始化 EKF
  │         ├─ 静止标定
  │         └─ 用重力初始化 roll/pitch
  └─ 循环
       ├─ ManagerTaskLoop()
       │    └─ Bmi088ImuUpdate()
       │         ├─ Bmi088Update()     读取与换算传感器数据
       │         ├─ ImuEkfStep()      预测 + 校正
       │         └─ ImuEkfGetState()  取出姿态和欧拉角速度
       └─ osDelayUntil()              等待下一周期
```

因此不需要在 CubeMX 生成的 `Core/Src/freertos.c` 中再写一份循环。框架的强定义 `StartManger()` 会覆盖生成文件里的 weak 定义。

## 2. 下载后怎么看角度

先编译 Debug 固件：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

通过现有调试配置下载 `build/Debug/h7_framework.elf`。上电时将板子放稳，约前一秒不要碰它；初始化包含传感器复位等待、600 次静止标定和 100 次初始姿态采样，实际耗时还受任务调度影响。无需发命令，管理任务会自动初始化和更新。

在 Watch 或实时监视中添加：

| 变量 | 看什么 |
| --- | --- |
| `g_app_imu.initialized` | 传感器和滤波器是否初始化成功 |
| `g_app_imu.valid` | 最近一次尝试更新是否成功 |
| `g_app_imu.status` | BMI088 驱动或 EKF 返回的状态，0 为成功 |
| `g_app_imu.angle_deg.roll` | 横滚角，单位度 |
| `g_app_imu.angle_deg.pitch` | 俯仰角，单位度 |
| `g_app_imu.angle_deg.yaw` | 相对于初始航向的偏航角，单位度 |
| `g_app_imu.update_count` | 是否持续成功更新，正常约每秒增加 1000，需实机确认 |
| `g_app_imu.sample_interval_ms` | 两次尝试更新之间的实际毫秒间隔，目标为 1 |
| `g_app_imu.delayed_count` | 间隔大于配置任务周期的累计次数 |
| `g_app_imu.error_count` | 运行中驱动或 EKF 更新失败的累计次数 |
| `g_app_imu.last_success_ms` | 最后成功更新时间，用于判断结果是否过期 |

先水平放稳，再分别缓慢绕板载传感器 X、Y、Z 方向转动。roll、pitch、yaw 分别对应这些姿态自由度；正方向遵循右手规则，但传感器轴是否等于你想要的车体轴，需要核对板上方向和安装方式。默认沿用传感器轴，不能直接把它们叫作车体“前、左、上”。

`valid` 表示这次计算返回成功，不代表误差一定很小，也不代表拥有绝对航向。若任务停止执行，`valid` 可能仍保留 true，所以还要看时间戳是否持续前进。

## 3. 初始化代码逐步解释

### 第 1 步：让任务周期和滤波时间单位一致

```c
s_manager_period_ms = g_app_task_config.manager_period_ms;
if (s_manager_period_ms == 0U)
{
  s_manager_period_ms = 1U;
}
Bmi088ImuConfig config;
Bmi088ImuConfigInit(&config, (float)s_manager_period_ms * 0.001f);
```

当前 `app_config.c` 中管理任务周期是 1 ms。EKF 的 `dt` 使用秒，因此要乘 `0.001f`。例如 `1 ms = 0.001 s`。把 1 直接当成秒传进去，会让陀螺积分尺度相差 1000 倍。

零周期按 1 ms 处理，与 `StartManger()` 的行为一致。`Bmi088ImuConfigInit()` 先清空配置，再填入默认重力、初始姿态采样和运动加速度低通等参数，避免未初始化字段出现随机值。

本工程 RTOS tick 为 1000 Hz，`osDelayUntil()` 的 1 tick 正好是 1 ms。若以后修改 `configTICK_RATE_HZ`，必须同时检查 `app_task.c` 的毫秒到 tick 换算，本例并未重写调度框架。

### 第 2 步：打开静止标定

```c
config.is_auto_calibrate_enabled = true;
config.calibration_sample_count = 600U;
config.calibration_sample_delay_ms = 1U;
```

真实陀螺仪静止时通常也有非零读数。若不处理，它会被积分成角度漂移。例如零偏为 `0.01 rad/s`，一分钟就可能积累约 `0.6 rad`，即约 34.4° 的误差。

驱动对静止样本求平均，得到 `g_bmi088.gyro_offset[3]`；同时估计重力模长，用来整体修正加速度尺度。这不是完整的多姿态标定，也不保证消除温漂或各轴比例误差。

标定期间用加速度模长波动和陀螺各轴波动检查运动。明显移动可能返回 `kBmi088CalibrateMotionError`。均匀缓慢旋转也可能被错误当成零偏，所以一定要真实静止，不能只依赖波动判据。

默认还启用了 `is_init_euler_from_accel_enabled`。驱动平均 100 次加速度，用下面的关系建立初始倾角：

```text
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
yaw   = 0
```

这能让板子斜放上电时也从接近真实的 roll/pitch 开始。这里假设初始加速度主要来自重力；摇晃或持续加速会破坏这个条件。yaw 设为 0 是指定参考方向，并非测出了正北。

### 第 3 步：初始化板载 BMI088 和 EKF

```c
g_app_imu = (AppImuState){0};
g_app_imu.status = Bmi088ImuInit(&g_bmi088, &hspi2, &config);
g_app_imu.initialized = g_app_imu.status == kBmi088NoError;
s_imu_last_sample_ms = HAL_GetTick();
```

`g_bmi088` 是驱动已经提供的全局对象，里面同时放传感器读数、标定结果和 `imu_ekf`，无需再创建第二个 EKF 对象。`hspi2` 是 CubeMX 已初始化的 SPI2 句柄。

驱动内部通过 PC0/PC3 分别选择加速度计和陀螺仪。当前走阻塞式 SPI 读取，不依赖数据就绪中断，也不需要调用 DMA 接口来启动本例。

时间基准放在整个初始化结束之后，否则第一次更新会把标定花费的时间也当成旋转积分时间。初始化成功后 `valid` 仍为 false，等第一次运行期更新成功才置 true。

本例默认关闭加热，`install_matrix = NULL` 对应单位矩阵。配置结构体在初始化函数内是局部变量，但本例参数在调用期间同步使用，Q/R 与安装矩阵指针也都没有指向局部数组。

## 4. 周期更新代码逐步解释

### 第 4 步：初始化失败时不继续运算

```c
if (!g_app_imu.initialized)
{
  return;
}
```

传感器没找到、寄存器配置失败、静止标定失败时，不应继续把默认零数据当成测量值。查看 `g_app_imu.status` 定位原因，修正后让板子静止并复位，重新完整初始化。

### 第 5 步：测量实际 dt

```c
uint32_t now_ms = HAL_GetTick();
uint32_t interval_ms = now_ms - s_imu_last_sample_ms;
if (interval_ms == 0U)
{
  return;
}
s_imu_last_sample_ms = now_ms;
g_app_imu.sample_interval_ms = interval_ms;
if (interval_ms > s_manager_period_ms)
{
  g_app_imu.delayed_count++;
}
g_bmi088.imu_ekf.dt = (float)interval_ms * 0.001f;
```

任务计划每 1 ms 执行，但可能被更高优先级的任务抢占。如果实际过了 2 ms，却只积分 1 ms，就会低估旋转角度。这里每次把实际间隔写入滤波器。

`HAL_GetTick()` 的精度是毫秒，不是精确的传感器采样时间。它用于这份轮询学习例子；需要更高精度或同步采样时，应改用硬件时间戳/数据就绪信号，并实测执行耗时。

无符号减法可以处理正常的计数回绕。同一毫秒内的重复调用直接跳过，避免调度追赶时把相同样本积分多次。长时间断点暂停会丢失真实运动过程，改 dt 也无法恢复那些样本；学习时使用连续运行和实时监视，长暂停后建议复位。

### 第 6 步：完成采样、预测、校正

```c
g_app_imu.status = Bmi088ImuUpdate(&g_bmi088);
g_app_imu.valid = g_app_imu.status == kBmi088NoError;
if (!g_app_imu.valid)
{
  g_app_imu.error_count++;
  return;
}
```

这一行封装了完整数据链：

1. 从 SPI 读取加速度、角速度与温度原始值。
2. 根据配置量程换算成 `m/s²`、`rad/s` 和摄氏度。
3. 扣除静止标定的陀螺零偏，修正加速度尺度，再通过安装矩阵转换方向。
4. 将 `corrected_gyro` 和 `corrected_accel` 传给 `ImuEkfStep()`。
5. 读回欧拉角、欧拉角速度和 EKF 估计的残余陀螺零偏。

失败时不会发布新的显示角度，旧角度仍保留。使用它们时必须检查 `valid` 和 `last_success_ms`。目前应用统计的是驱动实际返回的错误；现有底层部分 SPI 操作没有逐笔向上传递 HAL 错误，因此 `valid` 不是对所有传感器故障的完整诊断。

### 第 7 步：把弧度转换成方便观察的度

```c
const float rad_to_deg = 57.2957795f;
g_app_imu.angle_deg.roll = g_bmi088.euler.roll * rad_to_deg;
g_app_imu.angle_deg.pitch = g_bmi088.euler.pitch * rad_to_deg;
g_app_imu.angle_deg.yaw = g_bmi088.euler.yaw * rad_to_deg;
g_app_imu.update_count = g_bmi088.update_count;
g_app_imu.last_success_ms = HAL_GetTick();
```

转换关系为 `deg = rad × 180/π`。`g_app_imu.angle_deg` 只是显示用副本，滤波器内部和 `g_bmi088.euler` 始终使用 rad。后续接 PID 时先统一目标和反馈的单位，不能把度目标与弧度反馈直接相减。

这些角度按算法归一化到约 ±180°，跨过边界会跳变。例如 yaw 从 +179° 变到 -179°，不代表突然反向转了 358°。计算角度误差时应处理周期性；本例未提供累计多圈角度。

## 5. EKF 内部到底在做什么

### 5.1 状态、输入和观测分别是什么

本工程直接估计六个状态：

```text
x = [roll, pitch, yaw, bx, by, bz]ᵀ
u = [gyro_x, gyro_y, gyro_z]ᵀ
z = [accel_x, accel_y, accel_z]ᵀ
```

前三项是角度，后三项是 EKF 估计的残余陀螺零偏。`gyro_offset` 是开机静止平均得到的偏置，`gyro_bias` 是 EKF 的状态估计，两者属于前后两层修正。不要在应用中再把这两项重复扣除。

陀螺读数是机体系绕 XYZ 的角速度，不是任意姿态下的欧拉角变化率。代码先扣除偏置得到 `p、q、r`，再用姿态运动学换算。

### 5.2 预测：根据陀螺仪推算下一时刻

对应 `EkfPredictModelAndJacobian()`：

```text
φ̇ = p + q sinφ tanθ + r cosφ tanθ
θ̇ = q cosφ - r sinφ
ψ̇ = (q sinφ + r cosφ) / cosθ

φ⁻ = φ + φ̇ dt
θ⁻ = θ + θ̇ dt
ψ⁻ = ψ + ψ̇ dt
```

这里 `φ、θ、ψ` 就是 roll、pitch、yaw，上标 `⁻` 表示尚未用本轮加速度校正的预测值。零偏状态在预测模型中保持原值，允许其不确定性通过 Q 增长。

例如水平时，绕 X 轴角速度为 `0.1 rad/s`，过了 `0.001 s`，roll 预测增加 `0.0001 rad`。陀螺能快速跟随旋转，但持续积分会积累偏置误差。

运动学中出现 sin、tan 等非线性函数，因此还要计算 `F = ∂f/∂x`，用局部线性近似传播协方差：

```text
P⁻ = F P Fᵀ + Q
```

P 表示状态估计的不确定性，Q 表示每一步过程模型的不确定性。EKF 的“扩展”就在于围绕当前估计计算这些雅可比矩阵。

### 5.3 观测模型：这个倾角下应该读到什么重力

对应 `EkfMeasurementModelAndJacobian()`：

```text
h(x⁻) = [-g sinθ,
          g sinφ cosθ,
          g cosφ cosθ]ᵀ
```

水平静止、正 Z 朝上时，模型期望读到约 `[0, 0, +9.81] m/s²`。板子倾斜后，重力分量分配到其他轴，因此加速度可以反过来约束 roll/pitch。

代码同时计算 `H = ∂h/∂x`。你可以直接在源码中看到 H 的 yaw 那一列为零：绕竖直方向旋转不会改变重力方向，所以重力无法提供绝对 yaw 观测。

### 5.4 校正：比较实际加速度与预测重力

对应 `ImuEkfStep()` 中的矩阵运算：

```text
y = z - h(x⁻)                    实测与预测的差，称为创新
S = H P⁻ Hᵀ + R                创新的不确定性
K = P⁻ Hᵀ S⁻¹                  卡尔曼增益
x = x⁻ + K y                    用创新修正状态
P = (I - K H) P⁻                更新估计的不确定性
```

K 决定这次观测能修正多少状态。不同状态和不同轴之间通过矩阵一起计算，不是给三个角度各做一次普通低通。这个实现使用上述简化协方差更新，最后把角度归一化，并输出扣除估计偏置后的角速度。

### 5.5 Q、R 怎么设置

当前没有传自定义 Q/R，使用算法默认值：

| 参数 | 默认对角线 | 意义 |
| --- | --- | --- |
| Q | `[1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6]` | 前三项为角度过程不确定性，后三项为残余零偏过程不确定性 |
| R | `[0.5, 0.5, 0.5]` | 三轴加速度观测噪声方差，单位对应 `(m/s²)²` |
| 初始 P | 6×6 单位阵 | 初始化时对状态的不确定性假设 |

相同 P 等条件下，增大 R 通常会减弱加速度对状态的修正；减小 R 会更相信加速度，也更容易把外部运动加速度当成倾斜。Q 改变预测不确定性，不能简单理解成某一个角度的“滤波强度”。这些默认值是代码初值，不是针对你这块板子测出来的最优噪声参数。

要实验，可以在 `Bmi088ImuConfigInit()` 后、`Bmi088ImuInit()` 前加入：

```c
static const float q_diag[6] = {
    1e-4f, 1e-4f, 1e-4f, 1e-6f, 1e-6f, 1e-6f};
static const float r_diag[3] = {0.5f, 0.5f, 0.5f};
config.q_diag = q_diag;
config.r_diag = r_diag;
```

先保留默认值建立对照，再只改变一个参数，比较静止抖动、倾斜响应和运动干扰。R 应为正，Q 不应为负。这里 Q 在每次更新时直接相加，没有按实际 dt 自动离散化；明显改变采样频率时需要重新评估 Q，不能认为修改 dt 就解决了所有时间尺度问题。

## 6. 常见现象与限制

**yaw 慢慢变化。** 六轴加速度/陀螺融合没有绝对航向参考。静止标定能减轻漂移，但不能保证长期无漂移，EKF 也不能凭空补出不可观测的航向。需要绝对航向时，还要融合磁力计、视觉或其他外部参考，或采用额外的运动约束。

**移动板子时 roll/pitch 晃动。** 加速度计也会响应运动，本算法将输入按重力模型参与校正，当前没有动态加速度门控或自适应 R。先缓慢倾斜、停稳观察。`motion_accel_body/earth` 是估计姿态之后扣除重力得到的派生输出，不能把它们替代 `corrected_accel` 输入这个重力观测模型。

**倾到接近 90° 后异常。** 本工程是欧拉角六状态 EKF，pitch 接近 ±90° 时存在运动学奇异性。源码对 cos 分母做了数值保护，但这没有消除欧拉角奇异性。学习阶段先在较小倾角内操作；需要大姿态翻转时再考虑四元数或误差状态 EKF。

**转动方向与预期不符。** 先核对传感器轴和板子安装。`install_matrix` 是将传感器坐标转换到目标机体系的 3×3 旋转矩阵；应同时作用于加速度和角速度，保持右手坐标系。不要只给某一个输出角度随便取负来掩盖轴向问题。

**初始化失败。** `kBmi088NoSensor` 检查 SPI2 和片选；其他状态码见 `dev_bmi088.h`，包括具体寄存器配置失败、标定运动和 EKF 错误。`initialized` 不会自动重试，避免重复标定时板子正在运动。

**更新慢或时间戳不前进。** Manager 当前优先级低于云台等任务，`1 ms` 是目标周期，不是实测保证。观察 `sample_interval_ms` 和 `delayed_count`，检查高优先级任务占用、SPI 超时及调试暂停；不要先靠修改 Q/R 解决调度问题。

**其他任务要用角度。** 在目标任务中包含 `app_imu.h` 和需要的驱动头，统一使用 rad，并校验状态与数据新鲜度。本例全局结构方便学习和调试，多字段读取不是原子快照；若跨任务用于控制，应增加适当的快照/互斥机制，避免拿到不同更新周期的字段组合。

## 7. 如何确认代码没有接错

本机数学检查使用真实 EKF 和 CMSIS 矩阵实现：已知静止倾角下从错误初值收敛；水平绕 Z 旋转时，检查不同 dt 的积分结果。

```sh
cmake -S tests/imu_ekf -B build/tests-imu-ekf -G Ninja
cmake --build build/tests-imu-ekf
ctest --test-dir build/tests-imu-ekf --output-on-failure
```

本机检查不连接 BMI088，也不验证 SPI 时序、板上安装方向、任务实时性或真实噪声。最终要下载固件，分别检查静止、缓慢单轴倾斜、回到原位和水平转动。比较实测方向、角度和时间间隔后，再考虑调整噪声或安装矩阵。

继续阅读可参考 [Bosch BMI088 官方资料](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf) 的传感器与接口说明，以及 [Welch 与 Bishop 的 Kalman Filter 入门资料](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) 中的 EKF 章节。本页具体状态、默认参数和调用顺序以仓库源码为准。
