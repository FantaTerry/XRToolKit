# FIELD · 现场操作单（临时文件，问题解决后删除）

分支：`fix/servo-governors`（origin），最新 `6a6ac49`。
机器人上的工作分支是 `canghao`，**这条分支不用切过去**，下面的命令都能直接用。

---

## Step 0 · 不重编译、不改分支：先用你刚才那次测试的日志

黑匣子在你上一轮跑的那一版里就已经在记了。先看有没有：

```zsh
cd ~/Documents/dj_main/XRToolKit
git fetch origin
ls -l demo/log/teleop/*.csv 2>/dev/null || ls -l log/teleop/*.csv 2>/dev/null || echo "no csv"
```

- **有 csv** → 直接出报告（不需要编译、不需要切分支）：

  ```zsh
  git show origin/fix/servo-governors:demo/tools/teleop_report.py > /tmp/report.py
  python3 /tmp/report.py demo/log/teleop
  ```

  把输出（大约 10 行）发我。**这一步就能定位大部分问题。**

- **没有 csv** → 说明你跑的那一版没有黑匣子，走 Step 1。

---

## Step 1 · 只有需要时才编译（把改动合到 `canghao`）

```zsh
cd ~/Documents/dj_main/XRToolKit
git status -sb                 # 必须在 canghao 分支、工作区干净
git log --oneline -5           # 如果和我的分支差很多，先别合并，把这两行发我
git merge origin/fix/servo-governors
```

- 出现 `CONFLICT` → 立刻 `git merge --abort`，把冲突文件名发我，不要硬解。
- 合并成功 → 编译：

```zsh
cd demo && ./build.zsh
```

**编译报错就把第一条错误发我**（这段 C++ 没在本机编译过）。

---

## Step 2 · 一次测试，日志务必留下来

终端 A：

```zsh
cd ~/Documents/dj_main/XRToolKit/demo
./run_teleop.zsh robot_left_ip:=192.168.71.37 robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 gripper_force:=20 gripper_speed:=40 \
  gripper_configure_tio:=false 2>&1 | tee /tmp/teleop_$(date +%H%M).log
```

终端 B（确认新版生效，五行都应该有）：

```zsh
grep -E 'payload:|joints\(rad\)|feedback calibration|governors:|black box' /tmp/teleop_*.log
```

**测试动作（就一次，5 分钟）：**

1. 两臂各握一次、松一次，看看有没有报警
2. 把左臂带到会报警的那个姿态附近，慢慢靠近，再退出来 → 重复 3 次
3. 左手腕左右转、上下翻各 3 次（你说摆动的地方）
4. 换右臂做同样的事
5. 做完按 Ctrl-C

**记录表**（看墙上的钟就行，误差半分钟没关系）：

| 时间 | 哪只手 | 什么感觉 | 当时在做什么 |
|---|---|---|---|
| 例 14:32 | 左 | 腕部左右抖 | 手腕左右转 |
|  |  |  |  |
|  |  |  |  |
|  |  |  |  |

---

## Step 3 · 出报告

```zsh
cd ~/Documents/dj_main/XRToolKit
python3 demo/tools/teleop_report.py demo/log/teleop
```

如果合并了分支，脚本已经在了；没合并就用 Step 0 里的 `git show ... > /tmp/report.py`。

---

## Step 4 · 发我这些

1. **报告输出**（`teleop_report.py` 的 10 行）
2. **记录表**（上面那张，几行就行）
3. 如果编译过：`grep` 出来的五行启动日志
4. 如果报过警：`grep -E 'ALARM|CONTROLLER' /tmp/teleop_*.log`

---

## 报告怎么读（你也可以自己先看）

```
ALARM   14:32:07  errcode=1212416          ← 报警时刻，和你的记录表对齐
SEND    median=8.00ms ... missed=N overlap=N
        ├ missed>0：指令之间有大空档 → 控制器等不到指令
        └ overlap>0：两条指令挤进同一个 8ms 窗口 → 控制器丢步
LAG     x 个窗口超过 5mm
        └ 指令点跑在实际点前面 → 跟随误差在累积
STALL   x 个窗口：在下指令但 TCP 几乎不动
        └ 就是你说的"快卡住"：指令在发，机器不跟
JRATE   j1..j6 各自的最大角速度及时刻
        └ 哪个关节速率异常高，就是被 IK 放大到失控的那个
```

**看 `STALL` 和 `JRATE` 这两行**——如果"快卡住"的那几秒对应某个关节速率尖峰，基本就锁定是 IK 放大；如果 `STALL` 里 `cmd` 很大而 `moved` 是 0，且关节角度没怎么动，那就是控制器拒绝执行，方向完全不同。

---

## 开关清单（临时用，需要时再动）

```zsh
# 看当前值
ros2 param get /rc_ctrl_right servo_timer_driven
ros2 param get /rc_ctrl_right servo_singularity_slowdown

# 关掉可疑环节做对照（每次只关一个）
ros2 param set /rc_ctrl_right servo_max_track_error_mm 0.0    # 跟随闸门
ros2 param set /rc_ctrl_right servo_trans_accel_mm_s2 0.0     # 加速度限制
ros2 param set /rc_ctrl_right servo_leash_radius_mm 0.0       # leash
ros2 param set /rc_ctrl_right servo_timer_driven false         # 退回消息驱动
```
