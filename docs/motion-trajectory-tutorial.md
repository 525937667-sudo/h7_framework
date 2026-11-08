# 五次多项式位置轨迹：从公式到控制代码

本文对应本项目 `alg_motion.c` 中的 `MotionTrajectoryReset()` 和 `MotionTrajectoryUpdate()`，面向第一次接触轨迹规划的读者。

相关源码：[实现](../Skywalker/Algorithm/Src/alg_motion.c)、[结构体与接口](../Skywalker/Algorithm/Inc/alg_motion.h)。本文只讲算法和接入方式，不修改现有控制程序。

## 1. 为什么需要轨迹规划

假设云台当前在 0 rad，你希望它转到 1 rad。

最直接的办法是把位置环目标从 0 直接改成 1。对于控制器，这意味着目标位置突然跳变，可能产生很大的误差和控制输出。

轨迹规划在最终目标与控制器之间增加一道处理：把“去 1 rad”变成“一系列随时间变化的小目标”。控制器每轮跟踪其中一个目标。

```text
最终位置目标：1 rad
        ↓
轨迹规划器：用 2 秒生成平滑的运动过程
        ↓
本轮规划位置、规划速度、规划加速度
        ↓
位置环 / 速度环控制器
        ↓
电流或力矩指令 → CAN → 电机
        ↑
       实际反馈
```

要分清三个概念：

| 名称 | 含义 | 在哪里产生 |
|---|---|---|
| 最终目标 | 最后想去的位置，例如 1 rad | 遥控器、上层逻辑等 |
| 规划位置 | 这一刻应该走到的位置 | 轨迹规划器 |
| 实际位置 | 电机这一刻真正到达的位置 | 传感器反馈 |

规划器算的是“应该怎样运动”。电机是否能跟上，由控制器、负载和机构能力共同决定。轨迹计算结束不代表真实电机已经到位。

## 2. 两个函数分别做什么

```c
void MotionTrajectoryReset(MotionTrajectory *trajectory,
                           float position, float duration);

void MotionTrajectoryUpdate(MotionTrajectory *trajectory,
                            float target, float sampling_time);
```

- `Reset()`：建立初始状态，设置起始位置和轨迹时长，初始速度与加速度为零。
- `Update()`：每个控制周期调用一次。发现新目标时重新规划，再推进时间，计算本轮的输出。

例如：

```c
// 启动时执行一次
MotionTrajectoryReset(&trajectory, 0.0f, 2.0f);

// 后续每 1 ms 执行一次
MotionTrajectoryUpdate(&trajectory, 1.0f, 0.001f);
```

这里 `2.0f` 是整段轨迹时长，`0.001f` 是单次调用的推进时间，两者都以秒为单位。

## 3. 先理解位置、速度、加速度

以旋转运动为例：

- 位置 p：转到了什么角度，单位 rad。
- 速度 v：角度变化有多快，单位 rad/s。
- 加速度 a：速度变化有多快，单位 rad/s²。

数学上：

```text
v(t) = dp(t) / dt
a(t) = dv(t) / dt = d²p(t) / dt²
```

“求导”可以暂时理解成：根据位置随时间变化的规律，算出它变化的快慢。

对幂函数，规则是：

```text
d(tⁿ) / dt = n × tⁿ⁻¹
```

例如 `p(t) = 3t²`，那么 `v(t) = 6t`，`a(t) = 6`。

本算法先写出位置公式，再对它求导，得到速度和加速度，保证三个输出来自同一条轨迹。

## 4. 为什么选择五次多项式

一段轨迹需要满足六个条件：

| 时刻 | 位置 | 速度 | 加速度 |
|---|---|---|---|
| 开始时 | p₀ | v₀ | a₀ |
| 结束时 | p目标 | 0 | 0 |

结束时速度与加速度都为零，表示希望平滑地停在目标位置。开始时不强制静止，是为了允许运动中途修改目标。

六个条件通常需要六个待定系数。因此选择五次多项式：从常数项到五次项，正好六项。

为了统一处理不同的运动时长，先定义：

```text
T = 轨迹时长，必须大于 0
t = 本段轨迹已经运行的时间
u = t / T
```

`u` 称为归一化时间：开始为 0，结束为 1。

位置公式为：

```text
p(u) = c₀ + c₁u + c₂u² + c₃u³ + c₄u⁴ + c₅u⁵
```

注意：下文的 `c₁` 不是直接以 rad/s 表示的速度，因为它对应的是对 u 的变化率。

## 5. 速度和加速度为什么要除以 T

因为 `u = t/T`，每经过 1 秒，u 变化 `1/T`。

所以对真实时间 t 求导时：

```text
v(t) = (c₁ + 2c₂u + 3c₃u² + 4c₄u³ + 5c₅u⁴) / T

a(t) = (2c₂ + 6c₃u + 12c₄u² + 20c₅u³) / T²
```

这就解释了源码中的：

```c
trajectory->velocity /= duration;
trajectory->acceleration /= duration * duration;
```

如果漏掉换算，输出速度和加速度的数值、单位都会不正确。

## 6. 六个系数是怎么推出来的

先把起点速度和加速度转换成归一化量：

```text
Δp = p目标 - p₀
V = v₀ × T
A = a₀ × T²
```

在起点 `u = 0` 代入位置、速度、加速度公式：

```text
c₀ = p₀
c₁ = V
c₂ = A / 2
```

前三个系数就确定了。

在终点 `u = 1`，要求位置达到目标、速度和加速度为零：

```text
c₃ + c₄ + c₅       = Δp - V - A/2
3c₃ + 4c₄ + 5c₅   = -V - A
6c₃ + 12c₄ + 20c₅ = -A
```

这是三个未知数的三个一次方程。为方便看清消元过程，记三个右端分别为 B、C、D。

```text
第二式 - 第一式 × 3： c₄ + 2c₅ = C - 3B
第三式 - 第一式 × 6： 6c₄ + 14c₅ = D - 6B

再消去 c₄：
2c₅ = D - 6C + 12B
```

回代得到：

```text
c₃ = 10Δp - 6V - 1.5A
c₄ = -15Δp + 8V + 1.5A
c₅ = 6Δp - 3V - 0.5A
```

对应源码：

```c
float delta = target - trajectory->position;
float velocity = trajectory->velocity * duration;
float acceleration = trajectory->acceleration * duration * duration;

c[0] = trajectory->position;
c[1] = velocity;
c[2] = acceleration / 2.0f;
c[3] = 10.0f * delta - 6.0f * velocity - 1.5f * acceleration;
c[4] = -15.0f * delta + 8.0f * velocity + 1.5f * acceleration;
c[5] = 6.0f * delta - 3.0f * velocity - 0.5f * acceleration;
```

这些数字不是经验参数，也不需要调试，它们是由六个边界条件解出来的。

## 7. 最容易理解的情况：从静止到静止

若起点速度与加速度均为零，即 `v₀ = a₀ = 0`，公式简化为：

```text
p(u) = p₀ + Δp × (10u³ - 15u⁴ + 6u⁵)
v(u) = Δp/T × (30u² - 60u³ + 30u⁴)
a(u) = Δp/T² × (60u - 180u² + 120u³)
```

例如从 0 rad 到 1 rad，T = 2 s：

| 时间 t（s） | 进度 u | 规划位置（rad） | 规划速度（rad/s） | 规划加速度（rad/s²） |
|---:|---:|---:|---:|---:|
| 0 | 0 | 0 | 0 | 0 |
| 0.5 | 0.25 | 0.103516 | 0.527344 | 1.406250 |
| 1.0 | 0.50 | 0.500000 | 0.937500 | 0 |
| 1.5 | 0.75 | 0.896484 | 0.527344 | -1.406250 |
| 2.0 | 1 | 1 | 0 | 0 |

前半段加速，后半段减速；位置平滑到达终点。表中 t = 0 是初始状态，第一次调用 Update 后得到的是 t = sampling_time 的状态。

## 8. 结构体中每个变量的用途

```c
typedef struct
{
    float coefficient[6]; // 六个多项式系数
    float target;         // 上一次接受的最终目标
    float elapsed;        // 当前这段轨迹已经运行的时间，秒
    float duration;       // 当前规划使用的总时长，秒
    float position;       // 当前规划位置
    float velocity;       // 当前规划速度
    float acceleration;   // 当前规划加速度
} MotionTrajectory;
```

这个对象必须跨周期保存，不能在每轮循环里重新创建并清零。它既保存输出，也保存下一轮需要的历史状态。

## 9. Reset 的代码逻辑

```c
void MotionTrajectoryReset(MotionTrajectory *trajectory,
                           float position, float duration)
{
    memset(trajectory, 0, sizeof(*trajectory));
    trajectory->position = position;
    trajectory->target = position;
    trajectory->duration = duration;
    trajectory->coefficient[0] = position;
}
```

1. 清零整个对象，所以初始速度、加速度、已运行时间都为零。
2. 把当前规划位置和已有目标设为起始位置。
3. 保存轨迹时长。
4. 把常数项设为起始位置，其他系数为零，所以未收到新目标时位置保持不动。

Reset 适合在建立轨迹或明确需要重新对齐时调用。如果电机正在运动，它会把规划速度、加速度强制清零，不适合每轮调用。

## 10. Update 的完整执行逻辑

### 10.1 获取系数数组和时长

```c
float *c = trajectory->coefficient;
float duration = trajectory->duration;
```

`c` 指向对象内部的数组，修改 `c[0]` 就是在修改 `trajectory->coefficient[0]`，没有另建一份数组。

### 10.2 目标变了才重新计算系数

```c
if (target != trajectory->target)
{
    // 用上一轮规划的位置、速度、加速度建立新轨迹
    // 计算 c[0]～c[5]，公式见第 6 节
    trajectory->target = target;
    trajectory->elapsed = 0.0f;
}
```

目标不变时，系数不变，只推进时间。目标变化时，从当前规划状态出发，重新花完整的 T 秒到新目标。

因此，运动中途改变目标时，规划位置、速度、加速度在连接处连续。但三阶导数，即加加速度 jerk，不保证连续。

这里没有读取电机反馈。如果电机实际落后，重新规划也仍然从“规划位置”出发。

### 10.3 推进时间

```c
trajectory->elapsed =
    fminf(trajectory->elapsed + sampling_time, duration);

float u = trajectory->elapsed / duration;
```

`fminf()` 取两个数中的较小值，防止 elapsed 超过终点。若周期为 1 ms，每次就增加 0.001 s。

本函数不会读取系统时钟。如果实际 10 ms 才调用一次，却一直传 0.001 s，真实运动时间就会与规划不符。

### 10.4 计算三项输出

源码采用嵌套乘法，例如位置写成：

```c
trajectory->position =
    ((((c[5] * u + c[4]) * u + c[3]) * u + c[2]) * u + c[1]) * u + c[0];
```

展开后就是：

```text
c₀ + c₁u + c₂u² + c₃u³ + c₄u⁴ + c₅u⁵
```

这种写法叫霍纳法，避免反复计算幂，适合在单片机周期循环中使用。速度、加速度也用相同思路计算。

### 10.5 到终点后精确置位

```c
if (trajectory->elapsed >= duration)
{
    trajectory->position = trajectory->target;
    trajectory->velocity = 0.0f;
    trajectory->acceleration = 0.0f;
}
```

直接赋值可以消除终点浮点误差。后续只要目标不变，就保持这个状态。

## 11. 更容易读懂的等价写法

下面保留同样的数学逻辑，把嵌套公式写成显式幂。用于学习对照，不需要替换项目源码。调用者应保证指针有效、T > 0、dt > 0，且参数为有限数值。

```c
void MotionTrajectoryUpdateReadable(MotionTrajectory *s,
                                     float target, float dt)
{
    float T = s->duration;
    float *c = s->coefficient;

    if (target != s->target)
    {
        float dp = target - s->position;
        float V = s->velocity * T;
        float A = s->acceleration * T * T;

        c[0] = s->position;
        c[1] = V;
        c[2] = A / 2.0f;
        c[3] = 10.0f * dp - 6.0f * V - 1.5f * A;
        c[4] = -15.0f * dp + 8.0f * V + 1.5f * A;
        c[5] = 6.0f * dp - 3.0f * V - 0.5f * A;

        s->target = target;
        s->elapsed = 0.0f;
    }

    s->elapsed = fminf(s->elapsed + dt, T);
    float u = s->elapsed / T;
    float u2 = u * u;
    float u3 = u2 * u;
    float u4 = u3 * u;
    float u5 = u4 * u;

    s->position = c[0] + c[1] * u + c[2] * u2
                + c[3] * u3 + c[4] * u4 + c[5] * u5;

    s->velocity = (c[1] + 2.0f * c[2] * u
                  + 3.0f * c[3] * u2 + 4.0f * c[4] * u3
                  + 5.0f * c[5] * u4) / T;

    s->acceleration = (2.0f * c[2] + 6.0f * c[3] * u
                      + 12.0f * c[4] * u2
                      + 20.0f * c[5] * u3) / (T * T);

    if (s->elapsed >= T)
    {
        s->position = s->target;
        s->velocity = 0.0f;
        s->acceleration = 0.0f;
    }
}
```

需要包含 `alg_motion.h` 和 `math.h`。如果运行到第 1000 轮，不要重新算“从初始位置出发”的轨迹，而是保留对象继续 Update。

## 12. 最小练习：先观察规划输出

先在 PC 上计算，不接电机，也能理解这个算法。下面是独立演示程序，不要把它的 main 放进现有嵌入式工程。

```c
#include <stdio.h>
#include "alg_motion.h"

int main(void)
{
    MotionTrajectory trajectory;
    const float dt = 0.001f;

    MotionTrajectoryReset(&trajectory, 0.0f, 2.0f);
    printf("time,position,velocity,acceleration\n");
    printf("0,0,0,0\n");

    for (int i = 1; i <= 2001; ++i)
    {
        MotionTrajectoryUpdate(&trajectory, 1.0f, dt);
        if (i % 100 == 0 || i == 2001)
        {
            printf("%.3f,%.6f,%.6f,%.6f\n",
                   i * dt, trajectory.position,
                   trajectory.velocity, trajectory.acceleration);
        }
    }
    return 0;
}
```

由于 0.001 用二进制浮点数不能精确表示，反复累加后，数值到达 2 秒可能有微小误差；第 2001 轮用于确保观察到终点保持状态。表格使用的是理想数学时间。

练习时依次修改目标距离和 duration，观察：距离越远，同样时长下速度越大；时长越长，同样距离下速度和加速度越小。

## 13. 怎么接到控制任务

最基本的接法：规划位置交给位置环，位置环输出目标速度，再交给速度环。

```text
最终角度 → 五次轨迹 → 规划角度 → 位置 PID → 目标角速度
                                                  ↓
实际角速度 ───────────────────────────────────→ 速度 PID
                                                  ↓
                                               目标电流
```

本项目 DJI 驱动已有位置/速度串级计算。若 motor 已正确初始化、使能并配置为位置模式，可以使用这样的业务片段：

```c
// 文件级状态，跨控制周期保存
static MotionTrajectory s_trajectory;

// 在获得有效反馈、机构处于预期初始状态后调用一次
// initial_angle 必须和控制器反馈使用相同的坐标及单位
void AxisTrajectoryInit(float initial_angle)
{
    MotionTrajectoryReset(&s_trajectory, initial_angle, 2.0f);
}

// 每 1 ms 调用；motor 的配置、反馈更新和输出使能由应用保证
void AxisControlStep(DjiMotor *motor, float final_angle)
{
    MotionTrajectoryUpdate(&s_trajectory, final_angle, 0.001f);
    motor->state.given_angle = s_trajectory.position;

    // 现有驱动执行位置环、速度环，更新 given_current
    DjiMotorCurrentCalculate(motor);

    // 后续按同一总线和指令组统一打包，再调用 FdcanTransmit()
}
```

示例需要 `alg_motion.h`、`dev_dji_motor.h`。这两个 `Axis...` 函数是教学封装，当前项目没有预留同名函数。可以把状态和业务代码放在 `app_robot.c`，分别接入对应的任务初始化和循环。

注意当前驱动位置反馈的选择：GM6020 使用连续角度，其余 DJI 类型在现有默认位置计算中使用单圈角度。轨迹跨圈时，需要确认驱动的反馈与目标处理符合你的机构需求。

`AppConfigInit()` 执行时可能还没有收到有效电机反馈，因此不应无条件把尚未更新的零值当作真实起点。可以先初始化通信与对象，再在任务启动流程中等待有效反馈后建立轨迹。若其他任务也使用这套对象，应在就绪前禁止它们执行控制。

## 14. 规划速度、加速度要不要用

初次使用时，可以先只用规划位置，调好位置/速度环，再考虑前馈。

一种常见的自定义串级结构是：

```text
速度目标 = 位置控制器输出 + 规划速度
```

规划速度提前告诉速度环“这条轨迹本来就需要这么快”，位置环主要修正跟踪误差。规划加速度也可通过惯量等模型换算成力矩前馈。

但现有 `DjiMotorCurrentCalculate()` 不会自动把轨迹的 velocity、acceleration 加进去。特别是只给 `given_omega` 赋值再调用位置模式计算，会被驱动内部的位置环结果覆盖。需要前馈时，应明确修改或另写控制流程，并检查单位、限幅与饱和处理。

## 15. duration 怎么选

当前算法没有最大速度、最大加速度输入，必须由应用选择合理的 T。

对于“从静止到静止，且中途不改目标”的简化轨迹，可以从第 7 节公式推得：

```text
峰值速度绝对值 = 1.875 × |Δp| / T
峰值加速度绝对值 = (10 / √3) × |Δp| / T²
                  ≈ 5.773503 × |Δp| / T²
```

速度峰值出现在 u = 0.5；加速度极值出现在 `u = (3 ± √3) / 6`。

若允许最大速度为 vmax、最大加速度为 amax，可选择：

```text
T ≥ 1.875 × |Δp| / vmax
T ≥ √(5.773503 × |Δp| / amax)
```

取两者较大值，并结合机构实际留出余量。vmax 和 amax 都应为正。

例如 Δp = 1 rad，vmax = 1 rad/s，amax = 2 rad/s²：

```text
速度要求 T ≥ 1.875 s
加速度要求 T ≥ 1.699 s
```

选 2 s 时，峰值速度为 0.9375 rad/s，峰值加速度约 1.4434 rad/s²。

这些公式只适用于静止到静止。中途重新规划时，初始速度与加速度可能不为零，需要检查新轨迹的实际峰值，不能直接套用这个结论。

## 16. 初学者容易踩的坑

| 问题 | 原因与处理 |
|---|---|
| 每轮位置都像重新起步 | 每轮都调用了 Reset；应保留对象，只周期调用 Update |
| 轨迹比预期慢或快 | sampling_time 与实际调用间隔不一致，或把毫秒当秒 |
| 电机尚未到位，规划已经结束 | 轨迹结束只是目标到终点；应另外判断实际位置误差和实际速度 |
| 上电目标突然跳变 | 未用有效实际位置建立初始状态，或坐标零点不一致 |
| 频繁重新规划 | 函数用浮点数 `!=` 判断新目标，微小抖动也会重置时间；在应用层锁存命令或设置适当更新门槛 |
| 始终追着移动目标走 | 每次目标变化都会重新安排完整 T 秒，持续变化的目标不保证按原截止时间到达 |
| 目标改变后继续向原方向走了一段 | 为保持原来的速度、加速度连续，可能需要先制动；重新规划不保证单调、不保证不越界 |
| 修改 duration 后行为异常 | 系数建立时已包含旧 T，直接修改 T 不会自动触发重规划；运动中不要这样修改 |
| 数值无效或除零 | 源码没有参数校验；保证对象指针有效、数值有限、T > 0、dt > 0 |
| 位置变化很平滑，电流仍然很大 | 时长太短、机构受限、PID 未调好等；轨迹本身不限制电流，也不保证跟踪稳定 |

`rise_rate`、`fall_rate` 属于另一个斜坡函数，与本算法的 T 不同。本算法也不是严格保证 jerk 上限的 S 曲线规划器；它保证这里设置的边界条件及重规划连接处的位置、速度、加速度连续，但没有配置 jerk 上限。

## 17. 按这个顺序上手

1. 先用第 12 节的方式观察 0 → 1、2 秒轨迹的输出，核对第 7 节表格。
2. 保持目标不变，理解每轮推进时间、沿同一条多项式求值的过程。
3. 运动到一半改变一次目标，观察轨迹如何从当前规划状态重新开始计时。
4. 在控制任务中用有效位置初始化轨迹，只将规划位置接到已有位置环。
5. 记录规划位置与实际位置，确认跟踪误差和输出限幅，再逐步缩短 T。
6. 确有需要时，再增加规划速度或加速度前馈。

阅读源码时，可以始终围绕这条主线：**目标变化时算系数，每轮推进时间并求值，控制器跟踪本轮规划目标。**
