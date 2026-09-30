# 批量拍照数据测试输入切换器

运行：

```bash
python3 scripts/批量拍照数据测试/prepare_dataset_inputs.py
```

脚本开头的 `DATA_ROOT` 和 `INPUT_DIR` 是唯一需要配置的两个绝对路径。

每次按 Enter，脚本仅覆盖项目 `input` 中的三张深度图、`raw_color.png` 和 `request.txt`；不会启动服务、发送 TCP 或触发规划。准备完成后，请手动执行你现有的规划触发操作。

同一次拍照文件夹内的每个 `Workpiece[n] - T_base_obj` 都作为独立测试项。每次 Enter 都会写入一个且仅一个 `T_B_O_target`；当前拍照的 `T_base_flange` 与 `T_base_flange_joint_angles_deg` 会随该文件夹所有工件复用。

## 自动批量触发（仅在规划服务已由人工启动后）

`run_dataset_trigger_regression.py` 不启动或停止服务；它等价于反复执行“按 Enter 准备下一项 → 调用既有 TCP 触发器”。每项失败都会记录并自动继续下一项：

```bash
python3 scripts/批量拍照数据测试/run_dataset_trigger_regression.py
```

若某次测试已经执行到第 1 项，续跑第 2 项：

```bash
python3 scripts/批量拍照数据测试/run_dataset_trigger_regression.py --start-sequence 2
```

默认每轮完成后等待 5 秒；可用 `--interval-s 5` 明确指定。

逐项 JSONL 报告会写到项目 `logs/`。
