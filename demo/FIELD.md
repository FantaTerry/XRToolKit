# FIELD · 现场操作单（临时文件，问题解决后删除）

分支：`fix/servo-governors`（origin），最新 `ab8ee80` 之后。
机器人上的工作分支是 `canghao`，**这条分支不用切过去**，下面的命令都能直接用。

---

## Step 0 · 先确认你跑的是哪一版（30 秒，必须先做）

上一轮两次测试的指令节奏都是 **10 ms 为主 + 15% 的 0 ms 叠窗**，这是**旧二进制**的特征（新版的定时器路径会出现 8 ms 的整数倍）。所以先查：

```zsh
ros2 param get /rc_ctrl_left servo_min_period_ms      # 新版才有
ros2 param get /rc_ctrl_left servo_timer_driven       # 上一版才有
ls -l install/jaka_driver/lib/jaka_driver/rc_ctrl_node  # 看编译时间
```

| 输出 | 结论 |
|---|---|
| `Parameter not declared` | **跑的是旧二进制**，编译没生效 |
| `8.5` / `false` | 新版已加载 |

旧的话，**必须清干净再编**（colcon 有时判定"无变化"直接跳过）：

```zsh
cd ~/Documents/dj_main/XRToolKit/demo
./clean.zsh && ./build.zsh
ls -l install/jaka_driver/lib/jaka_driver/rc_ctrl_node   # 时间戳应该是刚才
```

顺便把这三行的输出发我：

```zsh
git status -sb
git log --oneline -3
ls -l install/jaka_driver/lib/jaka_driver/rc_ctrl_node
```

---

## Step 1 · 跑一次，抓启动日志

新版本启动会多打这两行，**这是这次测试最关键的信息**：

```
[right] feedback calibration: actual_tcp=..us(ok=0/1) actual_joints=..us(ok=0/1) robot_status=..us(ok=0/1) simple=..us
[right] feedback source=robot_status every N cycle(s) (.. us/cycle), gate=on, alarm log=on every M cycle(s)
```

- `feedback source=none` → 三种反馈全不可用，闸门和奇异点降速都关着，**告诉我**
- `source=robot_status` → 反馈终于通了（上一版是 none）

```zsh
cd ~/Documents/dj_main/XRToolKit/demo
./run_teleop.zsh robot_left_ip:=192.168.71.37 robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 gripper_force:=20 gripper_speed:=40 \
  gripper_configure_tio:=false 2>&1 | tee /tmp/teleop_$(date +%H%M).log
```

测试动作（5 分钟）：

1. 两臂各握一次、松一次
2. 把左臂带到会报警的那个姿态附近，慢慢靠近再退出来，重复 3 次
3. **在会报警的位置停住 3 秒**（保持 Grip 按住不动）
4. 左手腕左右转、上下翻各 3 次
5. 做完 Ctrl-C

记录表（看墙上的钟，误差半分钟没关系）：

| 时间 | 哪只手 | 什么感觉 | 当时在做什么 |
|---|---|---|---|
|  |  |  |  |
|  |  |  |  |

---

## Step 2 · 出报告

```zsh
cd ~/Documents/dj_main/XRToolKit
python3 demo/tools/teleop_report.py demo/log/teleop
```

---

## Step 3 · 发我这些

1. Step 0 的三行输出（确认版本）
2. 启动的 `feedback calibration` 和 `feedback source` 两行
3. 报告输出（10 行）
4. 记录表
5. `grep -E 'ALARM|CONTROLLER' /tmp/teleop_*.log`

