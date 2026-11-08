# GM6020 空载自动整定测试

本测试使用当前 `.ioc` 中的喵板 STM32H723VGT6：CAN1 = FDCAN1，RX 为 PD0、TX 为 PD1，经典 CAN 1 Mbps。电机是 **GM6020、拨码 ID 1、已启用电流控制模式、空载自由转动**。反馈标准帧为 `0x205`，电流指令标准帧为 `0x1FE`，电流放在前两个字节，其余六字节为零。因此本测试期间 CAN1 的同组 ID 2～4 电机也会收到零电流，不要接入其他控制程序共用该指令组。

## 代码入口

- `Skywalker/Application/Src/app_gm6020_test.c`：接收、启动/停止、1 ms 整定、CAN 电流发送、结果导出。
- `Skywalker/Application/Inc/app_gm6020_test.h`：调试命令、故障码、监控变量。
- `AppConfigInit()` 初始化测试并启动 CAN1 接收。
- `GimbalTaskLoop()` 每 1 ms 执行一次测试，其他任务不接管电机。

测试复用 `MotorTune`，流程为模型辨识及基准验证 → 回中 → 双向速度特征测量 → 9 组速度 PI 候选训练及独立验证 → 帕累托筛选。默认只整定速度 PI，不将结果自动写入 Flash，也不自动进入正常控制。

## 启动

1. 编译 Debug；若旧固件正在调试，先停止旧调试，再按 F5 下载新固件。
2. 接好电机电源和 CAN1 的 H/L、公共参考地，确认总线终端电阻配置正确。电机必须已启用电流控制模式。
3. 让固件继续运行。上电不会自动启动整定，空闲时只发零电流。
4. 当前调试配置选择 `STM32H723: STM32Cube Live Watch (OpenOCD)`。在左侧 **STM32Cube 实时监视** 标题右侧点 `+`，输入 `g_gm6020_test` 后回车，再展开查看成员。`online` 应为 `true`，`rx_count` 持续增加；静止且反馈连续有效约 100 ms 后，`ready` 为 `true`。
5. 将 `g_gm6020_test.command` 写为 **1**。命令消费后自动回到 0。若实时监视面板不支持写值，可暂停后在普通 Watch 中修改，再继续运行；开始后保持连续运行。
6. 观察 `active`、`state`、`error`、`identification_error`、`evaluated_count`、`pareto_count`，也可展开 `g_gm6020_tune` 查看完整模型和候选结果。

| command | 操作 |
| --- | --- |
| 0 | 无命令 |
| 1 | 从当前角度作为零点，启动一次整定；运行中再次写 1 不会重启 |
| 2 | 取消，清除待发指令并发送零电流 |
| 3 | 导出 `pareto_index` 指定的已验证结果；不驱动电机 |

电机会主动正反转。每个训练/验证轨迹为 16 秒，9 个候选全部训练并验证约 288 秒，另加辨识、特征测试和各轮回中，通常需数分钟。应用在 10 分钟时强制结束，实际是否完成取决于实测模型和验证结果。整定中不要单步、打断点或复位；中断控制流将使测试时序失效。需要结束时用命令 2，紧急停止直接切断电机电源。

实时监视面板与调试器必须匹配：`stgdbtarget` 使用 **STM32Cube 实时监视**，`cortex-debug` 使用 **Cortex Live Watch**。当前配置以 STM32Cube 的通用 GDB 调试器连接 OpenOCD，保留旧 ST-LINK 固件兼容性；OpenOCD 允许两个 GDB 连接，并禁止监视连接接入时暂停 CPU。变量 `g_gm6020_test` 已包含在 Debug ELF 中，无需额外声明或改名。

## 初始测试范围

配置在 `g_gm6020_tune_config`，修改源码后重新编译：

| 参数 | 默认值 |
| --- | --- |
| 周期 | 1 ms |
| 电流指令限幅 | ±0.3 A |
| 速度限制 | 8 rad/s（约 76 rpm）；算法在超过 1.2 倍时判运动越界 |
| 辨识减速 | 速度达到 4.8 rad/s 时暂停随机激励，施加随速度衰减的制动；低于 1.6 rad/s 连续 20 ms 后丢弃旧脉冲、重新生成激励；硬保护仍为 9.6 rad/s |
| 辨识积分窗口 | 50 ms，减小低速整数 RPM 反馈对窗口速度差的影响 |
| 辨识脉冲持续时间 | 约 100～275 ms，至少覆盖两个积分窗口 |
| 辨识等待上限 | 24 s；4 s 起检查模型，达到要求后可提前结束 |
| 辨识输入策略 | 独立三档电流脉冲，取消常态比例速度反馈；保留速度、位置及电流保护 |
| 位置周期项 | 阶次 10 / 20 / 36，由已保存波形对照选取；不是所有 GM6020 工况的通用保证 |
| 速度测试目标幅度 | 1.8 rad/s（约 17.2 rpm），辨识阶段可能高于此值 |
| 双向低速目标 | 工作幅度的 30%，即 0.54 rad/s（约 5.2 rpm）；原先约 1.15 rpm 接近整数 RPM 反馈的一格 |
| 连续角度范围 | 相对启动位置 ±4π rad（±2 圈），属于空载测试边界 |
| 自然频率搜索 | 24 / 48 / 72 rad/s；扩大低速负载下的控制增益范围 |
| 比例增益筛查 | `Kp * 0.001 / model.inertia > 0.3` 的候选跳过实测，状态为 `kMotorTuneDynamicLimit` |
| 负载补偿 | 基准验证、回位、候选训练及独立验证统一使用已辨识负载前馈 |
| 基准速度阻尼比 | 3.0；增强低速停转后突动时的比例抑制，积分增益公式不变 |
| 阻尼比搜索 | 1.0 / 2.0 / 3.0 |
| 加速度前馈 | 0 |
| 验证限制 | 归一化 RMS ≤0.25、超调 ≤0.30、调节时间 ≤0.5 s、饱和比例 ≤0.20 |

这些是首次空载测试设置，不保证一定找到可行参数。若辨识失败，先检查反馈和电流模式，查看 `identification_error`；不要把失败结果当作有效增益。若机构以后装上负载，应按实际负载重新整定。有限行程机构需要重新配置角度边界，不能沿用此空载范围。

首次实测在约 12 s 时失败：有效窗口 248 个、拟合度约 0.741、黏性阻尼约 -0.0306。数量和动态激励已达到原判据，但拟合度和阻尼符号未通过。当前设置增加积分窗口、脉冲时长及速度活动范围，用于改善低速量化和参数可辨识性；这属于待实机复测的调整，未降低拟合度 0.75 和阻尼非负等有效性门槛。

复测时在 `g_gm6020_test` 中直接看 `model_windows`、`model_fitness`、`model_inertia`、`model_viscous`、`identification_current`、`identification_peak_speed` 和 `identification_peak_current`。其中 `model_*` 是辨识过程中的候选模型，只有整个整定完成才能作为最终结果使用。先停止旧调试、重新 F5 下载更新后的 ELF，再写 `command = 1` 启动；不能在旧固件运行时直接使用新结构体布局读取这些新增字段。

`identification_motion_ratio` 显示最近一个 0.5 s 区间内有效运动的时间占比。当前激励会在该占比低于 50% 且未触发高速回退时增加幅度，激励幅度最高为 0.21 A，最终电流请求仍限幅 ±0.3 A。短暂速度峰值不会再单独阻止低运动占比时的幅度增长。

当前启用位置周期扰动模型，`model_fitness` 显示留出窗口的拟合度；`model_windows` 为训练窗口数，`model_validation_windows` 为留出窗口数，`model_position_span` 为辨识期间角度覆盖范围。10 参数模型至少需要 100 个训练窗口、20 个验证窗口，且最低阶次 10 对应的角度覆盖至少约 0.628 rad。满足计数只代表可以评价模型，最终仍须通过物理参数和拟合度检查。后续统一采用速度 PI 加摩擦、恒定负载及位置周期电流前馈，总电流仍受 ±0.3 A 限幅。

`baseline_state` 显示内部处于辨识还是速度验证，`validation_rms/peak` 显示基准验证误差；`speed_reference`、`pid_current`、`load_current` 可区分参考速度、PI 输出及负载补偿电流。`identification_current` 只在辨识阶段更新，单位始终为 A，不再将后续速度测试幅度误显示成电流。`error == kMotorTuneBaselineValidationFailed` 表示模型已通过、基准轨迹验证失败，与模型辨识失败分开报告。

基准速度阻尼由 `.plant.speed_damping` 单独设置，候选 `.damping` 只在基准通过后参与搜索。当前使用 3.0 是针对低速停转、突动的待实机验证设置，不保证适合其他电机或负载；原 RMS 0.55、峰值 1.3 的基准门槛及候选性能门槛保留。

## 完成与取出 PID

仅 `g_gm6020_test.state == kMotorTuneComplete`（数值 6）且 `pareto_count > 0` 表示完成。结束后自动发零电流，电机不主动保持位置。

1. 查看 `g_gm6020_tune.result.pareto[]`。元素是 `candidate[]` 的下标，每个候选保存 `speed`、训练指标和独立验证指标。
2. 将 `g_gm6020_test.pareto_index` 设为帕累托集合中的序号，范围为 `0..pareto_count-1`。
3. 写 `command = 3`。`exported == true` 后读取整份 `g_gm6020_speed_config`，同时保存 `g_gm6020_load_model` 和 `g_gm6020_model_origin`。序号 0 只是默认选择，不代表最优排名。
4. 正式控制时，采用与测试相同的电机连续角度坐标，计算 `position = continuous_position - g_gm6020_model_origin`；总请求为速度 PI 输出加 `SysidLoadCurrent(&g_gm6020_load_model, position, speed_reference, 0.08f)`，限幅后通过 `PidTrackOutput()` 回传“最终执行电流减总请求”。单独复制 PI 无法复现本测试控制链。原点与模型必须配套保存，跨重启恢复需重建相同角度坐标。此测试不会自动保存或驱动电机验证导出后的业务控制器。

默认速度测试中，导出的 `g_gm6020_position_config` 是候选设计值，**没有经过位置轨迹验证**。若需要位置环，先将 `.plant.tune_position` 改为 `true`；保留或调整 `.position_amplitude`，重新完成整定后再使用位置结果。

## 故障与监测

`g_gm6020_test.fault` 是应用接入层的故障：0 无故障，1 未就绪，2 配置无效，3 反馈丢失，4 CAN 发送失败，5 任务延迟超过 3 ms，6 超时，7 非有限输出，8 导出失败。反馈超过 10 ms 未更新、发送失败或应用超时时，立即取消整定，撤销待发指令并尝试持续发送零电流；总线断线时零电流无法保证送达，`tx_error_count` 可辅助排查。

`state/error` 是 `MotorTune` 的原始结果；`identification_error` 是其内部 `Autotune` 的失败细节。取消和故障都不会自动重试，修正原因并静止后需重新写 `command = 1`。

若 `kMotorTuneNoFeasibleCandidate`，展开 `g_gm6020_tune.result.candidate[i].training`，查看 `rms_error`、`overshoot`、`settled`、`unstable_segments` 与 `worst_overshoot_segment`。`unstable_segments` 按位记录失败段：1/2 正低速、4/5 负低速、6/7 正工作速度、8/9 负工作速度，其余零速度。训练通过后才会填入独立 `validation` 指标，其段号使用验证序列。

当前失败候选可以提前退出，因此不一定每组都运行 16 s。`completed_segments` 显示该候选最近一次训练/验证完成段数，`rejection_reasons` 显示失败原因；它们位于 `candidate[i]`，与 `training/validation` 同级。正常合格结果仍必须完成 16 段训练和 16 段独立验证。较大速度跑偏的候选会先交由基准控制器回位；触发硬运动保护或振荡保护时仍整轮停机。

`g_gm6020_test.stop_state`、`terminal_position`、`terminal_omega` 保存算法失败前的阶段和最后反馈。普通 `position/omega/current` 会继续刷新到停稳后的值，判断 `kMotorTuneMotionLimit` 应优先使用冻结值，不能仅凭停机后 `omega=0` 排除超速。

`prepare_braking=true` 表示正在减速、等待低速连续稳定 50 ms，此时不立刻向起点反转；变为 false 后，`prepare_reference` 从 0 平滑变化到回位速度。进入已有位置容差后参考归零，并连续稳定 0.25 s 才切入下一阶段。失败候选的瞬态电流不会作为回位积分继续保留；保护在这两个子阶段都有效。

若 `kMotorTuneOscillationDetected`，整定已因快速振荡终止，当前请求清零且不会自动重启。`g_gm6020_test` 底部的 `candidate_index`（从 0 开始）、`segment`、`guard_state`、`guard_rms_speed_error`、`guard_rms_current_slew`、`guard_reversals` 用于定位；更完整信息位于 `g_gm6020_tune.guard_*`。触发窗口为 50 ms，需同时有大于测试幅度 50% 的速度跟踪 RMS、大于 30 A/s 的执行电流变化率 RMS，以及至少 4 次显著变化方向反转。测试覆盖了应用撤销待发 CAN 指令、发送零电流及连接恢复后不自动重启。

当前 9 个网格点中，较高比例反馈的点可能被提前跳过，因此 `evaluated_count` 小于 9 也可能已测完全部可运行候选。跳过点仍保存在 `candidate[]`，不能导出。

当前工作/低速目标调整是为该电机反馈分辨率选择新的测试工况；即使本轮完成，也不能宣称原约 1 rpm 工况已经通过。百分比误差、超调和稳定性门槛保留，未增加电流、速度或角度保护上限。

`kMotorTuneMotionLimit` 表示速度或连续角度越界，外层可在内部辨识器更新前停止，因此 `identification_error == kAutotuneNoError` 不能证明辨识成功。候选 `model_fitness` 较高时仍须检查惯性、阻尼等全部判据；负阻尼模型不会通过。

`position` 为本次测试起点的相对连续角度，单位 rad；`omega` 为 rad/s；`current` 是反馈电流 A；`sent_current` 是已成功加入 CAN 发送队列的量化电流指令 A。两种电流不能互换，加入队列也不等于收到电机执行确认。

接收中断只复制原始反馈，任务一致性快照后解码；整定使用实测电流，并通过 `MotorTuneTrackOutput()` 回传限幅及 CAN 量化后的指令。测试没有另行添加反馈滤波，以免改变现有模型输入链。

## 辨识波形记录

模型辨识及基准速度验证阶段每隔至少 8 ms 将一条样本保存在 `g_gm6020_trace`：相对时间、接收计数、反馈年龄、连续角度、速度、实测电流，以及采样时上一条入队的电流指令。退出基准阶段后冻结；重新启动整定会清空计数并覆盖旧记录，复位也会丢失记录。

`g_gm6020_test.trace_samples` 为已记录样本数。最多 6400 条，约覆盖 51.2 s；满后停止写入，保留试验起始段。旧版本记录间隔为 4 ms。缓冲区静态占用 179200 B。加入周期模型及矩阵工作区后，固件 RAM_D1 共使用约 82.3%。离线 `replay_sysid` 从 CSV 时间戳推导采样间隔，拒绝不等间隔数据。

保持调试连接，在辨识退出后、下次启动或复位前，从工程根目录运行：

```sh
python tools/export_gm6020_trace.py
```

脚本使用与板端固件匹配的 `build/Debug/h7_framework.elf` 定位符号，通过本地 OpenOCD 的 4444 端口只读导出到 `build/Debug/gm6020-trace.csv`，不会启动电机或暂停 CPU。CSV 可以用来比较角度差分速度与电机反馈速度，并分析电流/速度的时间关系；不能仅凭一个模型结果就假定反馈延迟。

本次复测仍在约 24 s 后失败（拟合度约 0.607，惯性和阻尼为负），因此前一次增加时序窗口的调整尚未解决实机问题。当前修正了指数 RLS 丢弃匀速窗口的源码条件，详见 [源码问题与改动记录](autotune-failure-log.md)。需要重新下载本版并复测；若仍失败，先导出本次波形再继续定位。

随后一轮仍失败，6001 条记录已保存到 `build/Debug/gm6020-failed-181843.csv`，并保存对应 `.bin` 和 `.elf`。波形显示大多数时间停转，已据此修正仅用峰值调整激励的逻辑；本版仍需要重新下载并实机复测，不能将编译通过当作整定成功。

## 本机接入测试

`tests/gm6020/` 使用真实电机驱动与整定算法，仅替换 HAL 时钟及 CAN 收发，不连接硬件。用本机 GCC/CMake/Ninja 构建运行：

```sh
cmake -S tests/gm6020 -B build/tests-gm6020 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/tests-gm6020
ctest --test-dir build/tests-gm6020 --output-on-failure
```

覆盖上电零输出、未就绪拒绝启动、错误 ID/编码器/帧长过滤、命令不重复重启、停止、断线及恢复、发送失败、周期超时、运动越界、量化指令回传和导出下标检查。这些检查不能代替实机辨识与轨迹验证。

补偿控制仿真覆盖基准验证通过、弱响应增加增益、放大响应降低增益及固定重试频率。另一个带 RPM 量化、2 ms 电流环及周期负载的全流程仿真能够通过模型和基准验证，但候选仍因超调/稳定时间不达标而被拒绝；测试确认此时不导出结果。电脑端测试通过不表示实机已找到可行控制器。

补充测试复现并修正了共享 PID 在未饱和时因误差变号反复削减积分的问题，同步检查优化器敏感度。线性机械模型、2 ms 电流环与 RPM/CAN 量化的完整流程可产生经过训练和独立验证的帕累托集合并导出；周期负载负例继续保留，证明不会为显示成功而绕过判据。
