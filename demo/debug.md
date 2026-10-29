# Quest 按钮无响应：现场采集

## 已定位：启动被废弃 SDK 接口阻断

两份现场 CSV 均只有 `owner_stopped`，`state=3`、`reason=status_interval_100ms`、
`fault_code=-9999`；没有进入伺服循环。SDK 头文件将 -9999 定义为 `ERR_OBSOLETE`。
`set_status_data_update_time_interval` 和紧邻的 `set_block_wait_timeout` 都只适用于
v2.1.12 之前的 SDK。本次删除这两条启动配置调用，保留状态/保护/伺服检查和故障锁存。
本地反馈查询仍按 `feedback_period_s` 执行，不声称修改了 SDK 缓存周期。

原始输入采集的四路 Grip/Trigger 都达到过 1.0；本次故障发生在启动阶段。
已有模拟测试已复现旧路径失败并验证修复；现场是否正常启动仍需核验。

更新现场代码后，停止原来失败的遥操作进程。在 demo 目录编译并使用原现场参数启动，保存终端输出：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./build.zsh && ./run_teleop.zsh \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 \
  gripper_force:=20 \
  gripper_speed:=40 \
  gripper_configure_tio:=false 2>&1 | tee log/teleop_startup.txt
```

先保持 Grip 松开，检查左右都出现 `servo_move_enable ret=0`，且没有 `latched fault`。
若仍启动失败，保留 `demo/log/teleop_startup.txt`。现场通过后再整理正式操作说明。

## 原始输入采集（已完成）

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
