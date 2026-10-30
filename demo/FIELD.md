# FIELD · 唯一调试路线（临时文件，解决后删除）

一条路走到黑，中途不要跳步、不要切别的分支。
每个 **STOP** 点没通过就停下来发我日志，不要继续测。

---

## 正在跑旧参数的话，先做这四条（不用重编译，立即生效）

上一轮"死区 + 猛跳"是我加的跟随闸门造成的，先关掉它、打开奇异点降速：

```zsh
ros2 param set /rc_ctrl_left  servo_max_track_error_mm 0.0
ros2 param set /rc_ctrl_right servo_max_track_error_mm 0.0
ros2 param set /rc_ctrl_left  servo_singularity_slowdown true
ros2 param set /rc_ctrl_right servo_singularity_slowdown true
```

**设完必须用 `ros2 param get` 确认一次**——上一轮发现右侧是跑到一半才生效的
（黑匣子里 `held` 从 90% 掉到 0），说明参数是在节点起来之后才被设上的。

---

## 姿态锁定（L1）：固定末端姿态，只跟平移

报警发生在 **J4/J6（腕部）**，而 J4/J6 主要由**旋转指令**驱动。
把末端姿态锁死，旋转指令就没了，腕部只跟着几何走。

两种做法，都不用改控制律：

```zsh
# ① 完全锁死：只跟位置，姿态保持握持瞬间的样子（launch 参数）
./run_teleop.zsh ... use_rpy_ctrl:=false

# ② 允许小幅调整：绕握持姿态 ±N 度以内（热参数，可现场调）
ros2 param set /rc_ctrl_left  servo_max_rotation_deg 15.0
ros2 param set /rc_ctrl_right servo_max_rotation_deg 15.0
ros2 param set /rc_ctrl_left  servo_max_rotation_deg 0.0    # 等于全锁
ros2 param set /rc_ctrl_left  servo_max_rotation_deg 180.0  # 解除
```

`servo_max_rotation_deg` 默认 180（不限制），只影响旋转，不影响平移和奇异点降速。

---

## Step 1 · 切到一个确定的代码版本（不动你的 `canghao` 分支）

```zsh
cd ~/Documents/dj_main/XRToolKit
git stash -u || true
git fetch origin
git checkout -B verify-servo origin/fix/servo-governors
git log --oneline -1
```

**期望**：`git log` 里能看到 `fix(rc_ctrl):` 和 `docs:` 这两类提交，分支名是 `verify-servo`。
`canghao` 分支原样保留，随时 `git checkout canghao` 回去。

> 这里就是你之前出错的地方：我的改动一直在 `origin/fix/servo-governors` 上，
> 而你在 `canghao` 上编译，所以 `./build.zsh` 编的一直是**旧代码**。

---

## Step 2 · 清干净重编，然后确认二进制

```zsh
cd ~/Documents/dj_main/XRToolKit/demo
./clean.zsh
./build.zsh
source ./setup.zsh
```

验证（三条都要看）：

```zsh
git log --oneline -1
ros2 pkg prefix jaka_driver
ls -l "$(ros2 pkg prefix jaka_driver)/lib/jaka_driver/rc_ctrl_node"
```

**STOP ①**：二进制的修改时间必须是刚才；`ros2 pkg prefix` 必须指向
`~/Documents/dj_main/XRToolKit/install/jaka_driver`。
如果指向别处（比如旧的 overlay），说明启动了别的工作区，告诉我路径。

---

## Step 3 · 启动（顺序不变：PC-service → Quest → 本机）

照你原来的顺序。唯一区别是**把日志留下来**：

```zsh
cd ~/Documents/dj_main/XRToolKit/demo
./run_teleop.zsh robot_left_ip:=192.168.71.37 robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 gripper_force:=20 gripper_speed:=40 \
  gripper_configure_tio:=false 2>&1 | tee /tmp/teleop_$(date +%H%M).log
```

---

## Step 4 · 核对五行启动日志

另开终端：

```zsh
grep -E 'feedback calibration|feedback source|governors:|black box|joints\(rad\)' /tmp/teleop_*.log
```

应该看到类似：

```
[right] feedback calibration: actual_tcp=..us(ok=0/1) actual_joints=..us(ok=0/1) robot_status=..us(ok=0/1) simple=..us
[right] feedback source=actual_* every N cycle(s) (.. us/cycle), gate=on, alarm log=on every M cycle(s)
[right] governors: ... track_gate=0.0mm ... singularity=on(15deg,min 0.35) timer_driven=off min_period=8.5ms blackbox=on
[right] black box: log/teleop/right_....csv
[right] joints(rad): ... -> J5 is .. deg from the wrist singularity
```

**STOP ②**：
- 这五行**一条都没有** → 跑的还是旧二进制，回 Step 2。
- `feedback source=none` → 反馈全不通，闸门和奇异点降速都不会工作，**先别测**，把 calibration 那行发我。

---

## Step 5 · 测一次（5 分钟）

1. 两臂各握一次、松一次
2. 左臂慢慢靠近会报警的姿态，再退出来，重复 3 次
3. **在会报警的位置按住 Grip 停 3 秒**
4. 左手腕左右转、上下翻各 3 次
5. Ctrl-C 结束

记录（看墙上的钟，误差半分钟没事）：

| 时间 | 哪只手 | 什么感觉 | 在做什么 |
|---|---|---|---|
|  |  |  |  |

**报警时顺手拍一下示教器弹窗的中文原文**（这是现在最缺的一块）。

---

## Step 6 · 出报告

```zsh
cd ~/Documents/dj_main/XRToolKit
python3 demo/tools/teleop_report.py demo/log/teleop
```

---

## 发我这些

1. Step 2 的三条验证输出
2. Step 4 的五行日志（缺哪行也要说）
3. Step 6 的报告
4. 记录表 + 示教器报警原文
5. `grep -E 'ALARM|CONTROLLER' /tmp/teleop_*.log`

---

## 出错时的回退

```zsh
cd ~/Documents/dj_main/XRToolKit
git checkout canghao        # 回到你原来的分支
cd demo && ./clean.zsh && ./build.zsh
```
