# GM6020 空载自动整定测试

本分支使用上游 `b3def4b` 的算法，接入空载 GM6020、电流模式、ID 1、喵板 CAN1。`GimbalTaskLoop()` 每 1 ms 调用测试，上电不自动启动。此前本地扩展的位置周期模型、负载补偿、自定义低速比例和振荡保护不属于本版算法；旧实测问题记录见 [历史记录](autotune-failure-log.md)，其中参数与接口不能直接用于本版。

## 启动与停止

1. 在 RoboMaster Assistant 中确认 GM6020 已开启电流控制模式，ID 为 1，CAN1 为 1 Mbit/s。
2. 编译并下载 `build/Debug/h7_framework.elf`，保持电机空载及转动空间充足。
3. 在调试器中观察 `g_gm6020_test.online` 和 `ready`。连续收到静止反馈 100 ms 后就绪。
4. 写入 `g_gm6020_test.command = 1` 开始；运行中重复写 1 不会重启。保持固件连续运行，不要单步或打断点。
5. 写入 `command = 2` 取消并发送零电流。紧急停止时切断电机电源。

## 当前配置

配置位于 `g_gm6020_tune_config`：采样周期 1 ms，电流限幅 ±0.3 A，速度限制 8 rad/s，连续角度范围为相对启动点 ±4π rad。超过 1.2 倍速度限制或到达角度边界时算法失败并停止。

辨识积分窗口为 50 ms，总时长上限为 24 s，最低滑动速度为 0.08 rad/s；使用上游的启动探测、准静态、动态和独立模型验证阶段。当前未配置电流配对延迟。具体判据见 [上游算法说明](autotune.md)。

速度测试幅度为 1.8 rad/s，每段 1 s。自然频率范围为 24～72 rad/s，阻尼比为 1～3，加速度前馈固定为 0；三个 `count` 的乘积提供默认 9 组候选预算，不代表等间距网格。低速目标由上游算法定义为测试幅度的 12%。性能约束为归一化 RMS ≤0.25、超调 ≤0.30、调节时间 ≤0.5 s、饱和比例 ≤0.20。应用总运行时间上限为 10 分钟。

这些配置尚需实机复测，编译与本机测试通过不代表电机已完成整定。

## 观察与导出

`state/error` 是 MotorTune 状态和错误，`baseline_state/identification_error` 是内部 Autotune 状态和错误。`identification_stage` 显示辨识阶段；`model_status`、`model_windows`、`model_fitness`、`model_validation_fitness`、`model_inertia` 和 `model_viscous` 显示模型结果。`identification_peak_speed/current` 记录本轮辨识峰值。`identification_current` 在辨识之后保留最后激励幅度，不表示后续速度测试幅度。

只有 `state == kMotorTuneComplete` 且 `pareto_count > 0` 才能导出。设置 `pareto_index` 为 `0..pareto_count-1`，写入 `command = 3`；`exported == true` 后读取 `g_gm6020_speed_config`。集合序号不是最优排名。位置环默认不测试；需要位置环时显式启用 `plant.tune_position`，重新完成整定后再使用位置参数。

`fault` 为应用故障：0 无故障，1 未就绪，2 配置无效，3 反馈丢失，4 CAN 发送失败，5 任务延迟超过 3 ms，6 总运行超时，7 非有限输出，8 导出失败。反馈超过 10 ms 未更新时取消测试。停止或异常退出会撤销待发 CAN 指令并尝试发送零电流；断线时无法保证零电流送达。恢复连接后不会自动重启。

`terminal_position/terminal_omega` 保留运行退出时的反馈，其他反馈字段继续更新。新旧结构体布局不同，调试器须加载与板端固件对应的 ELF。

## 波形与验证

辨识和内部基准验证期间，每 8 ms 保存一次反馈，最多 6400 条，缓冲区占 179200 B。下一次开始会清空计数。保持 OpenOCD 连接，退出辨识后从根目录执行 `python tools/export_gm6020_trace.py`，依据 Debug ELF 的符号从本地 4444 端口只读导出。`replay_sysid` 使用上游四参数模型，从 CSV 推导采样间隔并拒绝不等间隔记录；稀疏记录的回放不能替代板端独立验证。上游串口记录器及离线分析工具见 [采集说明](../tools/tuning/README.md)。

```sh
cmake -S tests/gm6020 -B build/tests-gm6020 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/tests-gm6020
ctest --test-dir build/tests-gm6020 --output-on-failure
python -m unittest discover -s tests/gm6020 -p "test_*.py"
```

本机测试使用真实驱动和上游算法，模拟 HAL 时钟和 CAN。覆盖量化反馈模型辨识、无运动辨识失败、配置校验、上电零输出、帧过滤、启动停止、断线恢复、发送失败、任务延迟、超速、量化执行量回传和导出下标检查。旧本地算法专属用例随接口替换移除，可在历史提交中查看。
