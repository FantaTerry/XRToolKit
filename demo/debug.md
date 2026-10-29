# Quest 按钮无响应：现场采集

当前故障尚未定位。这份文件只记录排查步骤，现场确认前不加入 README。

先停止 `run_teleop.zsh`，保持 PC-service 运行、Quest 的 Controller Tracking 和 Send 开启。
这次只采手柄输入，不启动机械臂或夹爪。现有诊断脚本直接读取 `demo/src`，无需编译。

在现场主机复制执行这一组命令：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
mkdir -p log
./diagnose_xrt.zsh --duration 30 --interval 0.02 2>&1 | tee log/quest_input.txt
```

开始输出后，按下面顺序操作，每步约 5 秒：

1. 左右手全部松开。
2. 只按住左 Grip。
3. 松开左 Grip，只按住右 Grip。
4. 松开右 Grip，只按住左 Trigger。
5. 松开左 Trigger，只按住右 Trigger。
6. 全部松开，等采集自动结束。

数据和错误都保存在：

```text
$HOME/Documents/dj_main/XRToolKit/demo/log/quest_input.txt
```

先保留这一个文件，不需要为了这次采集启动机器人。
持续按住时若 `raw` 仍为 0，检查原始 SDK 输入链路；若 `raw` 正常变化，再定位桥接到控制节点的路径。
`invalid` 表示读取失败或数值无效。这里只记录实际结果，不用离线测试通过来判断现场故障已修复。
