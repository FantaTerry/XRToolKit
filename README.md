# Quest 3 + XRoboToolKit 控制 JAKA 双臂与 AMR 底盘

本轮控制链路修复及离线验证记录见文末“伺服连续性修复”。上线前必须按该节核对新增参数和行为变化。

本工程是 `RemoteCtrl-master` 的 XRoboToolKit 迁移版。Quest 3 不再通过旧 APK 的
adb/logcat 文本传输位姿，而是运行 `XRoboToolkit-Quest-1.0.1.apk`，由
`XRoboToolkit-PC-Service` 接收头显数据；`xrt_teleop` 再把数据转换成本工程原有的
ROS 2 控制话题。JAKA 双臂现在支持位置 XYZ + 姿态 RPY 的 6 维遥操作，PGI 夹爪、
AMR 底盘接口及设备地址保持原样。

Quest 3 的 MAC 地址为 `50:99:03:0a:38:78`。XRoboToolkit 使用 PC-service 自动
发现设备，日常遥操作不需要配置 MAC；该地址用于设备识别、库存记录和网络排查。

## 标准运行顺序：先 JAKA 双臂，后 D405

现场必须按下面的顺序启动。D405 和机械臂控制使用两个独立进程，但这里固定采用“先确认
机械臂登录、上电、使能和手柄跟踪全部正常，再启动视频”的顺序，便于一次只排查一条链路。

一句话顺序：`PC-service -> JAKA 双臂就绪 -> D405 发送端 -> Quest Remote Vision`。
日常运行且配置未变化时跳过第 0 步和第 2 步，严格按第 1、3、4、5、6、7 步执行。

### 修改代码后的固定编译命令

每次修改任何 Python、C++、YAML、launch、脚本或依赖后，必须先停止正在运行的
`run_teleop.zsh`，然后执行：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"

./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
ros2 pkg prefix jaka_driver
```

编译完成后重新执行标准流程第 3 步。只有修改了 `CMakeLists.txt`、`package.xml`、
JAKA SDK、依赖或出现旧编译缓存冲突时，才使用完整清理重建：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./clean.zsh
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
```

`d405_vision/` 下的 Python 发送端是由 `run_d405_vision.zsh` 直接运行的脚本，单独修改
D405 脚本时不需要 colcon 编译，停止终端 C 后重新执行 `./run_d405_vision.zsh` 即可。
如果同一次更新还包含 `src/` 下的 ROS 节点、Python bridge 或配置，仍必须执行上面的
`./build.zsh`。

### 0. 首次安装或代码更新后执行

只有第一次部署、代码或依赖发生变化时需要执行：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_xrt_sdk.zsh
./install_vision_deps.zsh
./clean.zsh
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
ros2 pkg prefix jaka_driver
```

最后一条命令必须输出 `jaka_driver` 的安装路径。D405 的
`video_source_d405.yml` 修改后，还必须重新执行一次
`./setup_d405_profile.zsh`，否则 Quest 会继续读取旧配置。

### 1. 终端 A：先启动 PC-service

保持此终端运行：

```zsh
cd /opt/apps/roboticsservice
./runService.sh
```

在继续前确认 PC-service 已正常启动，不要关闭该终端。

### 2. Quest：首次配置视频源

这一步只在首次使用 D405，或者 `video_source_d405.yml` 修改后执行。必须在启动双臂
控制前完成，因为该脚本会重启 APK。

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./launch_quest.zsh
./setup_d405_profile.zsh
```

第一次建议用 USB 连接 Quest；脚本会建立无线 adb 并保存 IP。重新打开 APK 后确认
Controller Tracking 已开启。此时先不要点 Remote Vision 的 Listen。

### 3. 终端 B：启动 JAKA 双臂和夹爪

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"

./run_teleop.zsh \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 \
  gripper_force:=20 \
  gripper_speed:=40 \
  gripper_configure_tio:=false
```

`run_teleop.zsh` 会再次启动 APK、加载工程 ROS 环境，然后启动 `xrt_teleop_bridge`、
左侧 `rc_ctrl`、右侧 `rc_ctrl` 和两个 PGI 夹爪。

### 4. 等待 JAKA 双臂完全就绪

观察终端 B，确认左右两侧都出现以下关键日志：

```text
[INFO] ... xrt_teleop_bridge: XRoboToolkit bridge started
[INFO] ... xrt_teleop_bridge: left controller tracking acquired
[INFO] ... xrt_teleop_bridge: right controller tracking acquired
[INFO] ... [left] login_in ret=0
[INFO] ... [right] login_in ret=0
[INFO] ... [left] servo_move_enable ret=0
[INFO] ... [right] servo_move_enable ret=0
```

只有左右两臂都出现 `servo_move_enable ret=0` 且没有锁存故障后才继续启动 D405。如果任一臂出现
`ERR_INVALID_HANDLER`、`ERR_NOT_ENABLED` 或无法进入伺服模式，先处理机械臂问题，不要
继续测试视频。

保持终端 B 运行。启动后先松开两侧 Grip，再重新按住。机械臂空载时按住右手柄 Grip，轻微移动后松开，确认右臂能跟随并
停止；再按同样方法测试左臂。不要在运动范围未确认时测试。

### 5. 终端 C：确认没有旧视频进程，再启动 D405

先确认终端 C 当前目录正确：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
```

首次运行或摄像头节点变化时先检查：

```zsh
./run_d405_vision.zsh --list-devices
./run_d405_vision.zsh --check
```

两条命令都成功后，保持终端 B 的双臂节点运行，在终端 C 启动视频：

```zsh
./run_d405_vision.zsh
```

等待终端 C 打印下面两类信息：

```text
Selected /dev/videoN (...), capture=YUY2
In Quest Remote Vision, enter JAKA host IP 192.168.1.x (...)
Waiting for Quest Remote Vision on 0.0.0.0:13579
```

把 `In Quest Remote Vision, enter JAKA host IP` 中的 IPv4 地址记下来。它必须是运行
D405 发送端的 JAKA 主机地址，不能是 JAKA 控制器 `192.168.71.36/37`，不能是底盘
`192.168.71.50`，也不能是 Quest 自己的地址。

### 6. Quest：最后打开 Remote Vision

在确认第 4 步的双臂伺服和第 5 步的 D405 发送端都运行后：

1. 戴上 Quest，保持 XRoboToolkit 的 Controller Tracking 和 Send 开启。
2. 打开 Remote Vision。
3. 视频源选择 `REALSENSE_D405`。
4. 点击 Listen，输入终端 C 打印的 `In Quest Remote Vision, enter JAKA host IP`。
5. 点击 Confirm。
6. 确认终端 C 出现 `Remote Vision control connected`、`OPEN_CAMERA`、
   `Streaming D405` 和持续增加的 `Video sent: frames=...`。

此时双臂遥操作和 D405 画面应同时工作。左手柄 Grip 控制左臂，右手柄 Grip 控制右臂，
松开 Grip 后清除未完成目标并保持；左手 Trigger 控制左夹爪，右手 Trigger 控制右夹爪。
本轮修复将握持期间的 Trigger 合并为最新值，松开 Grip 且跟踪新鲜时才执行夹爪同步通信；
执行前显式退出伺服，完成后恢复零增量保持。不要依赖握持运动期间夹爪立即动作。

`xrt_teleop_bridge` 每 5 秒打印一次原始输入，例如：

```text
Quest input: left tracked=True grip=0.000 trigger=0.000 | right tracked=True grip=0.000 trigger=0.000
```

按住右手 Grip 后应变成接近 `1.000`，并在对应 `rc_ctrl` 终端依次看到：

```text
[right] state 0 -> 1 reason=fresh_grip_reference fault=0 ...
```

松开后应看到：

```text
[right] state 1 -> 0 reason=grip_released fault=0 ...
```

左臂同理。Grip 状态现在由 `xrt_teleop_bridge` 以 125 Hz 持续发布，不再只在数值变化
时发布，因此即使 `rc_ctrl` 比 bridge 晚十几秒启动，也能收到当前状态。

### 7. 关闭顺序

按与启动相反的顺序关闭：

1. Quest 中停止 Remote Vision 的 Listen。
2. 终端 C 按 `Ctrl-C`，停止 D405。
3. 终端 B 按 `Ctrl-C`，停止双臂和夹爪控制。
4. 最后停止终端 A 的 PC-service。

### 只重启其中一项时

- 只重启视频：保留终端 A 和终端 B，停止并重新启动终端 C，然后在 Quest 重新 Listen。
- 只重启双臂：停止终端 B 后重新执行第 3 步；不需要重启 PC-service，D405 可保持运行。
- Quest 重启后没有无线 adb：用 USB 连接一次，再运行 `./launch_quest.zsh`。

如果自动 adb 启动失败，可显式指定 Quest IP：

```zsh
QUEST_IP=<Quest IP> ./run_teleop.zsh
```

## 目录

```text
demo/
├── src/
│   ├── xrt_teleop/          # XRoboToolkit SDK -> ROS 2 位姿/按键桥
│   ├── jaka_driver/         # JAKA 双臂伺服与 PGI 夹爪驱动
│   ├── jaka_msgs/           # JAKA 消息和服务
│   ├── sr_amr_control/      # AMR 底盘 ROS 2 控制
│   ├── sr_amr_interfaces/   # AMR 消息/动作接口
│   └── robot_bringup/       # 双臂 + 底盘整体启动
├── vendor/                  # AMR Python SDK wheel
├── .venv/                   # 工程私有 Python 环境（不提交 Git）
├── build.zsh                # 编译 ROS 2 工作区
├── clean.zsh                # 删除编译和运行生成物
├── setup.zsh                # 加载 ROS 与工作区
├── install_xrt_sdk.zsh      # 安装 xrobotoolkit_sdk
├── launch_quest.zsh         # 自动连接 adb 并启动 Quest APK
├── setup_d405_profile.zsh   # 向 APK 注入 REALSENSE_D405 视频源
├── install_vision_deps.zsh  # 安装 GStreamer/V4L2 视频依赖
├── run_d405_vision.zsh      # 启动 D405 H.264 发送端
├── d405_vision/             # D405 设备发现、视频源、发送端和配置
├── run_teleop.zsh           # 一键启动 Quest + 双臂遥操作
├── diagnose_xrt.zsh         # 原始手柄位姿、Grip 和 Trigger 诊断
├── xrt_receive_test.cpp     # PC-service 原始数据诊断程序
├── Makefile
└── .gitignore
```

`demo` 是改动后的工程；`RemoteCtrl-master` 原工程没有被修改。

## 环境与编译产物隔离

- Python 运行环境固定在 `demo/.venv`。
- `xrobotoolkit-sdk`、`sros-sdk-py` 和 pip 下载缓存都在 `demo` 内。
- ROS 2 本身仍使用 `/opt/ros/humble`；venv 通过 `--system-site-packages` 只读访问
  ROS 2 的系统 Python 包。
- 节点启动时设置 `PYTHONNOUSERSITE=1`，不会读取或写入用户的全局
  `~/.local/lib/python*/site-packages`。
- colcon 输出固定在 `demo/build`、`demo/install`、`demo/log`。
- colcon 和 ROS 缓存固定在 `demo/.colcon`、`demo/.ros`、`demo/log/ros`。
- `XRToolKit/build`、`XRToolKit/install`、`XRToolKit/log` 不再使用。
- `build.zsh` 检测到仓库根目录存在这些编译目录时会拒绝编译，要求先执行清理。
- xrt bridge 和 AMR Python 节点由 `demo/.venv/bin/python` 显式启动，不依赖系统
  Python 的第三方包。

清理所有编译和运行生成物，但保留工程 venv：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./clean.zsh
```

如需连同 venv 一起彻底删除：

```zsh
./clean.zsh --venv
```

## 数据链路

```text
Quest 3 XRoboToolkit APK
        │ Wi-Fi
XRoboToolkit PC-service (JAKA 主机)
        │ xrobotoolkit_sdk
    xrt_teleop
        ├─ /rc_ctrl/left_target   (geometry_msgs/PoseStamped)
        ├─ /rc_ctrl/right_target  (geometry_msgs/PoseStamped)
        └─ /rc_ctrl/button        (sensor_msgs/Joy)
                ├─ jaka_driver/rc_ctrl_node x2 -> JAKA 左/右臂 + PGI 夹爪
                └─ sr_amr_control             -> AMR 底盘
```

桥接节点以 125 Hz 读取 SDK，沿用旧工程的位姿低通滤波、坐标映射、手柄按键布局和
ROS 话题。控制消费端无需感知输入已经由旧 APK 换为 XRoboToolKit。

远程视觉使用另一条独立链路，不经过 PC-service 的位姿 TCP 通道：

```text
Intel RealSense D405
        │ USB / V4L2
D405 GStreamer 编码器（JAKA 主机）
        │ H.264 / TCP 13579 + 12345
XRoboToolkit Remote Vision（Quest 3）
```

以下命令均按 zsh 编写，使用 ROS 和工作区生成的 `setup.zsh`。

## 固定设备信息

| 设备 | 地址/参数 |
| --- | --- |
| 左 JAKA | `192.168.71.37` |
| 右 JAKA | `192.168.71.36` |
| AMR 底盘 | `192.168.71.50` |
| Quest 3 MAC | `50:99:03:0a:38:78` |
| Quest APK 包名 | `com.xrobotoolkit.client.quest` |
| D405 彩色流 | USB V4L2，默认 `1280x720@30` |
| Remote Vision 控制端口 | TCP `13579` |

以上 JAKA 和底盘地址沿用原工程，未作修改。

## 安装

以下步骤以 JAKA 主机为 Ubuntu 22.04 + ROS 2 Humble + ARM64 为例。

### 1. 安装对应架构的 PC-service

JAKA 主机是 `aarch64/arm64`，必须安装 PC-service 的 ARM64 包，不能使用
`amd64/x64` 包。

```zsh
uname -m                              # 应为 aarch64
dpkg --print-architecture             # 应为 arm64
```

从
<https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases>
下载 `XRoboToolkit-PC-Service_1.0.0.0_arm64.deb` 后安装：

```zsh
sudo apt-get update
sudo apt-get install -y build-essential
sudo dpkg -i XRoboToolkit-PC-Service_1.0.0.0_arm64.deb
sudo apt-get -f install
```

安装 Python SDK（`xrobotoolkit_sdk`）：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_xrt_sdk.zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -c "import xrobotoolkit_sdk; print('XRT SDK OK')"
```

安装脚本优先从 PyPI 安装官方预编译的 `xrobotoolkit-sdk`；如果预编译包不可用，
再使用或克隆
`demo/.deps/XRoboToolkit-PC-Service-Pybind` 中的源码执行
`demo/.venv/bin/python -m pip install .`。官方当前版本不再提供
`setup_ubuntu.sh`。

安装脚本会自动创建 `demo/.venv`，并同时安装 AMR SDK；不会运行
`pip install --user`。

### 2. 安装 AMR SDK 与 ROS 依赖

AMR wheel 已由 `install_xrt_sdk.zsh` 安装到 `demo/.venv`，不需要再安装到系统
Python。可这样验证：

```zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -c "import xrobotoolkit_sdk, sros_sdk_py; print('project env OK')"
```

JAKA SDK 所在的 `demo/src/jaka_driver/lib/libjakaAPI.so` 是 ARM64 版本，并且与复制
过来的 JAKA 头文件匹配。若更换机械臂控制器 SDK，必须同时替换匹配版本的库，并保持
架构为 `aarch64`：

```zsh
file "$HOME/Documents/dj_main/XRToolKit/demo/src/jaka_driver/lib/libjakaAPI.so"
```

### 3. 编译

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./clean.zsh
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
```

JAKA SDK 是 ARM64 二进制，不能在普通 x86_64 电脑上链接出可在 JAKA 主机运行的节点。
`install/setup.zsh` 是 `colcon build` 成功后生成的，不能在编译前 source。

## 机械臂控制选项

默认 `use_rpy_ctrl:=true`，同时控制位置 XYZ 和姿态 RPY。第一次验证旋转时应降低增益：

```zsh
./run_teleop.zsh \
  rotation_scale:=0.3 \
  rotation_gain:=0.3 \
  rotation_max_step_rad:=0.001
```

只测试 XYZ 位置控制时：

```zsh
./run_teleop.zsh use_rpy_ctrl:=false
```

不希望启动时初始化夹爪时：

```zsh
./run_teleop.zsh \
  gripper_rs485_channel:=1 \
  gripper_configure_tio:=false \
  gripper_initialize:=false
```

需要 AMR 底盘时，用 `RC_robot_bringup.launch.py` 替代第 3 步的
`dual_arm_teleop.launch.py`：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"

ros2 launch robot_bringup RC_robot_bringup.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  connect_ip:=192.168.71.50 \
  gripper_rs485_channel:=1 \
  gripper_configure_tio:=false
```

不要同时启动 `dual_arm_teleop.launch.py` 和 `RC_robot_bringup.launch.py`，否则会创建
重复的 JAKA 控制节点。

## 手柄映射

| Quest 输入 | ROS Joy 位置 | 功能 |
| --- | --- | --- |
| 左手 Grip | `axes[0]` | 按住控制左 JAKA；松开停止 |
| 右手 Grip | `axes[1]` | 按住控制右 JAKA；松开停止 |
| 左 Trigger | `axes[2]` | 左 PGI 夹爪，0 张开，1 闭合 |
| 右 Trigger | `axes[3]` | 右 PGI 夹爪，0 张开，1 闭合 |
| 左摇杆 | `axes[4..5]` | AMR 前后移动 / 转向 |
| 右摇杆 | `axes[6..7]` | 保留并发布 |
| X / Y | `buttons[0..1]` | 原 AMR 节点的急停 / 解除急停输入 |
| A / B | `buttons[2..3]` | 发布为 Joy 按键 |
| 菜单键 | `buttons[4..5]` | 发布为 `LThU/RThU` |
| 摇杆按下 | `buttons[6..7]` | 发布为 `LJ/RJ` |
| Grip | `buttons[8..9]` | 数字量状态 |
| Trigger | `buttons[10..11]` | 数字量状态 |

左右臂 Grip 使用 `0.6` 按下、`0.4` 松开的回差。夹爪默认使用模拟量 Trigger；如改用数字前端按钮，
可启动时加 `gripper_use_button:=true`。

## 6D 旋转控制

旋转桥接链路如下：

```text
Quest 手柄四元数
  -> headset_to_robot 坐标变换
  -> 与 Grip 按下时的姿态求相对旋转
  -> 取最短旋转并限制每周期角度
  -> JAKA servo_p(INCR) 的 rx/ry/rz
```

手柄每个新源帧姿态在 `xrt_teleop` 中应用共用坐标映射；其与两臂现场用户坐标系的对应关系仍须标定核查。默认只映射一半的手柄相对旋转，
并在每个控制周期以较小的角速度跟随。JAKA 驱动会保留尚未完成的旋转，避免快速转动
时直接丢失目标。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rpy_ctrl` | `true` | `true` 为 6D；`false` 只控制 XYZ |
| `rotation_scale` | `0.5` | 手柄相对旋转映射到机械臂的比例，范围 0..1 |
| `rotation_gain` | `0.5` | 以 8 ms 为基准的剩余旋转比例，其他周期按时间归一化，范围 0..1 |
| `rotation_deadband_rad` | `0.005` | 姿态死区，约 0.29° |
| `rotation_max_step_rad` | `0.0015` | 每次指令最大旋转，另受角速度和角加速度限制；8 ms 时约 10.7°/s |
| `rotation_sign` | `1` | 全部旋转轴的统一方向；方向整体相反时设为 `-1` |
| `grip_press_threshold` | `0.6` | Grip 达到该值才进入控制 |
| `grip_release_threshold` | `0.4` | Grip 低于该值才释放控制 |

实现保留参考系中左乘旋转、`Rz Ry Rx` 转 RPY 的明确假设；附带 SDK 未写明 INCR 姿态组合方向，需厂商或官方仿真确认。若手柄绕某个轴转动时机械臂三个旋转轴都朝相反
方向运动，可先用 `rotation_sign:=-1` 验证；若只有某两个轴交换或部分方向异常，应
修改 `xrt_bridge.yaml` 的 `headset_to_robot` / `tf_rotation`，不要把 `rotation_sign`
当作单轴反号参数。

本实现参考了 `vr-teleop-kit` 的 clutch-relative 映射、旋转比例缩放和松开后保持
最后目标的思想。当前 JAKA 版本仍使用控制器自身的 `servo_p` 完成笛卡尔到关节的 IK；
`vr-teleop-kit` 的 MuJoCo 解耦 IK 与 DK1 几何直接绑定，不能仅替换 IP 后用于 S5。

现场首测步骤：

1. 机械臂空载，爪内无工件，周围留出完整旋转空间。
2. 先只测试左臂，使用 `rotation_scale:=0.3`、`rotation_gain:=0.3` 和
   `rotation_max_step_rad:=0.001`。
3. 按住 Grip，分别做小角度 roll、pitch、yaw，确认方向和轴对应关系。
4. 松开 Grip 后确认位置和姿态都停止。
5. 确认无误后再测试右臂，再逐步提高旋转比例和速度。

## 松开 Grip 后的保持

JAKA `servo_p` 使用同一个 SDK 所有者循环发送。握持静止、平移在死区内和松手后的正常保持
都按 `step_num × 8 ms` 生成新的零 INCR，不重放上一条非零指令。Grip 松开立即丢弃尚未
提交的平移和姿态残差。夹爪维护时先退出伺服；故障时停止发送并退出，均有明确状态记录。
松手/失联时软件轨迹速度状态清零，实际停止过程依赖控制器滤波及退出伺服行为，须现场验证。

Grip 使用 `0.6` 按下、`0.4` 松开的回差，避免模拟量在 `0.5` 附近波动时反复触发
离合器。若现场 Grip 数值范围不同，可同时调整：

```zsh
grip_press_threshold:=0.6
grip_release_threshold:=0.4
```

## 坐标与标定

默认使用旧 Quest reader 和 XRoboToolkit 官方 teleop 示例相同的映射：

```text
(x, y, z) -> (-z, -x, y)
```

配置位于 `demo/src/xrt_teleop/config/xrt_bridge.yaml`：

- `headset_to_robot`：3x3 右手正交旋转矩阵，按行排列。
- `tf_translation`：Quest 原点在机器人基坐标系中的平移，单位米。
- `tf_rotation`：Quest 到机器人基坐标系的旋转，顺序为 `[w, x, y, z]`。
- `pose_scale`：平移缩放，默认 `1.0`。
- `smooth_alpha`：0 表示不平滑，越大越跟手。
- 姿态方向由同一坐标矩阵变换；旋转增量随后由 JAKA 驱动发送。

先保持默认值测试。只有在方向或零点不符合现场安装关系时才修改标定参数。

## D405 低时延远程视觉

现有 release APK 的 Remote Vision 可在运行时读取：

```text
/sdcard/Android/data/<APK 包名>/files/video_source.yml
```

因此不需要先重编 APK。工程提供 `REALSENSE_D405` 视频源配置，并沿用 APK 已验证的
H.264 TCP 和 `ZEDMINI` 解码路径。

D405 是单目 RGB 相机，而当前 APK 的平面 Remote Vision 使用左右眼分屏贴图。发送端
会把同一幅 D405 图像复制成左右完全相同的 `2560x720` 侧并排画面，双眼中看到的图像
一致，不会把单幅图像错误地拆给左右眼。它不提供真实立体深度。

### D405 首次配置说明

不要在机械臂尚未就绪时重复执行一套 D405 启动命令。D405 的安装、配置和运行分别对应
标准流程的第 0、2、5、6 步：

- 第 0 步运行 `install_vision_deps.zsh`，安装 GStreamer、`x264enc`、V4L2 工具和
  Python GI 绑定。
- 第 2 步运行 `setup_d405_profile.zsh`，把 `REALSENSE_D405` 配置写入 Quest APK。
- 第 5 步运行 `--list-devices` 和 `--check`，检查相机节点及 GStreamer 协商。
- 第 5 步最后运行 `run_d405_vision.zsh`。
- 第 6 步才在 Quest 中打开 Remote Vision。

`video_source_d405.yml` 修改后必须重新执行 `setup_d405_profile.zsh`，否则 Quest 仍会
使用 APK 数据目录中的旧配置。

`REALSENSE_D405` 的 `camera` 类型保持为官方发送端和 APK 已验证的 `ZED`；
`REALSENSE_D405` 只是 Remote Vision 中显示和选择的名字。发送端日志会显示实际请求
类型，不需要把 `camera` 改成 `REALSENSE_D405`。

`video_source_d405.yml` 的 shader 参数已与 Quest 专用
`XRoboToolkit-Unity-Client-Quest` 的双目 `ZEDMINI` 配置对齐：
`visibleRatio=0.46747`、`contentRatio=1.58458`。不要继续使用通用 Unity Client 中的
`0.439464/1.704850` 参数。

H.264 发送参数也已按 Quest 端 Android MediaCodec 兼容路径调整：

- `--encoder auto` 依次选择 `nvv4l2h264enc`、`v4l2h264enc`、`openh264enc`、`x264enc`。
- x264 使用 Baseline profile、`aud=true`，减少 Quest 硬件解码器兼容问题。
- 每 15 帧强制一个 IDR，且 `h264parse config-interval=-1` 在每个关键帧前携带 SPS/PPS。
- 视频 TCP 仍使用 4 字节大端长度前缀和 Annex-B H.264，与 Quest 插件
  `RobotVisionUnityPluginQuest` 完全一致。

### D405 设备节点

JAKA 主机连接多个摄像头时，不能把 `/dev/video0` 当成固定彩色节点。本次现场机器的
`/dev/video0` 实际只提供 `Z16` 深度流，强制作为 `YUY2` 使用会在 GStreamer 中出现
`reason not-negotiated (-4)`，Quest 只显示 Remote Vision 的绿色占位框。

发送端默认使用 `--device auto`，通过 RealSense D405 的 USB 产品 ID `0b5b` 找到 D405，
再排除 `Z16` 等深度格式，只选择 `YUYV/YUY2`、`UYVY` 或 `MJPG` 彩色节点。其他型号的
RealSense 和普通 USB 摄像头不会进入候选列表。

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh --list-devices
```

输出应包含类似内容：

```text
/dev/video0 | D405 | USB 8086:0b5b | ... | no color | formats=Z16
/dev/video2 | D405 | USB 8086:0b5b | ... | YUY2 | formats=Z16, YUYV, MJPG
Selected D405 color source: /dev/video2 (YUY2)
```

实际编号以上述输出为准。只有需要覆盖自动识别时才显式指定：

```zsh
./run_d405_vision.zsh --device /dev/video2
```

下面的 `--check` 会打开所选彩色节点并实际完成一次 GStreamer 协商，不再只是打印
管线文本：

```zsh
./run_d405_vision.zsh --check
```

D405 建议先使用 `1280x720@30`。默认输出为 Remote Vision 所需的
`2560x720@30`、4 Mbps。

`--capture-format AUTO` 会优先使用低延迟的 `YUY2`，没有 `YUY2` 时使用其他已报告
的彩色格式。通常不需要手工指定；需要覆盖时才使用：

```zsh
./run_d405_vision.zsh --capture-format MJPG
```

### D405 运行检查

标准流程第 5 步启动后，终端 C 应显示：

```text
Selected /dev/video2 (...), capture=YUY2
In Quest Remote Vision, enter JAKA host IP 192.168.1.20 (...)
Host IPv4 addresses: eth0=192.168.71.10, wlan0=192.168.1.20
Waiting for Quest Remote Vision on 0.0.0.0:13579
```

优先填写上面 `In Quest Remote Vision, enter JAKA host IP` 给出的地址。多网卡机器不能
直接凭网卡名称猜，JAKA 控制网段的 `192.168.71.x` 通常无法被 Quest 访问。

Quest 点击 Confirm 后，发送端必须依次出现：

```text
Remote Vision control connected ...
Control command: OPEN_CAMERA ...
OPEN_CAMERA request: camera=ZED target=192.168.x.x:12345 ...
Streaming D405 to 192.168.x.x:12345
Video sent: frames=1 bytes=... last_frame=... bytes
First H.264 access unit NAL types: [9, 7, 8, 5] (AUD=9, SPS=7, PPS=8, IDR=5)
```

NAL 列表中必须包含 `7, 8, 5`；启用 AUD 时通常是 `[9, 7, 8, 5]`。若没有 7/8，
MediaCodec 通常无法开始解码。

### D405 失败定位

按终端 C 最后出现的日志判断：

- 没有 `Remote Vision control connected`：Quest 中填错主机 IP，或者 TCP `13579` 被防火墙
  阻止。
- 有连接但没有 `OPEN_CAMERA`：APK 与视频源配置不匹配，重新运行
  `./setup_d405_profile.zsh` 后重启 APK。
- 有 `OPEN_CAMERA` 但没有 `Streaming D405`：发送端选择了错误的 V4L2 节点或 GStreamer
  协商失败，运行 `./run_d405_vision.zsh --list-devices` 和 `--check`。
- 有持续更新的 `Video sent: frames=...` 但 Quest 仍是绿色：网络和相机发送已经正常，
  检查 NAL 类型是否包含 `7, 8, 5`，然后按下面的顺序继续定位。

如果只出现一次 `Video sent: frames=1`，随后连续打印：

```text
AttributeError: 'Sample' object has no attribute 'unref'
```

说明运行的是修复前的发送端。PyGObject 会自动管理 `Gst.Sample` 的生命周期，旧代码却
在每帧结束时手工调用 `sample.unref()`，导致第一帧之后的所有回调都异常退出。该调用
已经删除；更新代码后停止并重新启动 D405 即可，不需要重新执行
`setup_d405_profile.zsh`。修复后应持续看到：

```text
Video sent: frames=30 bytes=... last_frame=... bytes
Video sent: frames=150 bytes=... last_frame=... bytes
```

如果先发送了几十帧，约 3 秒后又出现：

```text
WARNING Control session ended: timed out
D405 stream stopped (frames=89, bytes=...)
Remote Vision control disconnected
```

说明运行的是修复控制通道空闲超时之前的版本。Remote Vision 在 `OPEN_CAMERA` 后通常
不会持续发送控制命令，旧代码把正常的读超时误判成断线并停止视频。现在控制连接会：

- 每秒继续发送一次 `PING`；
- 允许正常的 `socket.timeout`，保持视频流不停；
- 只有对端真正关闭 TCP 或发生网络错误时才停止。

本次只修改了 `d405_vision/` 下的直接运行脚本，不需要 colcon 编译。停止 D405 终端后
重新执行：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh
```

修复后不会再因空闲 3 秒触发 `Control session ended: timed out`，`frames` 应持续增加。

### 更新 D405 配置和编码后重新验证

本次同时修改了 `video_source_d405.yml` 和发送端编码参数，因此必须按以下顺序重新验证。
D405 Python 是直接运行脚本，不需要 colcon 编译。

1. 停止当前 D405 终端。
2. 重新写入 Quest 配置：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./setup_d405_profile.zsh
```

3. 检查实际选中的编码器和管线：

```zsh
./run_d405_vision.zsh --check
```

Jetson/Orin 优先应显示 `encoder: nvv4l2h264enc`；没有 NVIDIA 编码器时会回退到
`v4l2h264enc`、`openh264enc` 或 `x264enc`。

4. 启动发送端：

```zsh
./run_d405_vision.zsh
```

5. Quest 先选择自定义的 `REALSENSE_D405`。若仍是绿色，停止 Listen，改选 APK 内置的
`ZEDMINI`。两者发送的都是同一幅 D405 画面，`ZEDMINI` 用于排除自定义 YAML 参数问题。

如果日志已有持续更新的帧和 `7/8/5` NAL、但两种视频源都是绿色，则主机侧发送已经
确认正常。Quest 官方工程记录了一个相关已知问题：从 Oculus XR Plugin 切换到 OpenXR
后，网络和 MediaCodec 解码仍工作，但 Unity 纹理不更新。若 logcat 出现
`MediaCodec created and started successfully` 和 `OES texture updated`，却没有画面，
需要重新检查 APK 的 XR Plugin 构建配置，不能再通过修改 JAKA 或 D405 发送端解决。

本项目 D405 兼容实现依据以下官方工程：

- Quest 客户端：<https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest>
- Quest 视频插件：<https://github.com/XR-Robotics/RobotVisionUnityPluginQuest>
- PC/Orin 发送端：<https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender>
- Quest 默认视频源：<https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest/blob/main/Assets/StreamingAssets/video_source.yml>

不打开 D405，只发送标准测试画面：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh --test-source
```

测试画面能显示而 D405 不能显示时，问题在相机采集/编码参数；两者都是绿色时，检查
Quest IP 输入、Remote Vision 配置或更换 `ZEDMINI` 项。不要继续反复修改 JAKA 遥操作
代码，因为视频链路与机械臂控制相互独立。

需要确认 APK 解码状态时，在 JAKA 主机另开终端：

```zsh
adb logcat -c
adb logcat | grep -E "MediaDecoder|MediaCodec|RemoteCamera|StartListen|OPEN_CAMERA"
```

正常应看到 `MediaDecoder server started on port 12345` 和
`MediaCodec ... started successfully`；若只看到 `MediaCodec not ready` 或
`Failed to create MediaCodec decoder`，应保存这段日志后再调整编码参数。

### D405 画质和编码选项

如果 CPU 负载较高或 Wi-Fi 延迟大，可降到 `848x480@30` 采集并按需降低码率：

```zsh
./run_d405_vision.zsh \
  --source-width 848 \
  --source-height 480 \
  --fps 30 \
  --bitrate 2000000
```

Jetson/Orin 主机如已安装 NVIDIA GStreamer 编码器，可尝试：

```zsh
./run_d405_vision.zsh --encoder nvv4l2h264enc
```

视频窗口固定在头显前方，因此移动或转头时始终显示 D405 的全局视角。D405 安装方向
决定画面是否倒置；若需要旋转 180°，后续应在发送端增加旋转参数，而不是旋转头显
整体坐标。

### 网络和延迟

- Remote Vision 控制连接使用 TCP `13579`。
- 编码后的 H.264 视频由主机主动连接头显提供的 TCP 端口，通常为 `12345`。
- 保持 JAKA 主机有线连接，Quest 使用 5 GHz Wi-Fi，关闭 Wi-Fi 省电。
- 发送端使用 `zerolatency`、无 B 帧、1 帧以内的关键帧间隔和丢弃旧帧队列。
- 该视频链路与手柄到 PC-service 的位姿链路相互独立，不会阻塞遥操作。

## 检查话题

先排查 Quest 原始输入。运行前停止 `run_teleop.zsh`，避免两个进程同时初始化 SDK：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./diagnose_xrt.zsh
```

未跟踪时输出 `untracked`；抓住手柄时应看到姿态数值持续变化，握住 Grip 时对应
`grip` 接近 `1.000`，按 Trigger 时 `trigger` 接近 `1.000`。按 `Ctrl-C` 结束，再
重新启动 `run_teleop.zsh`。

遥操作启动后，用另一个终端检查 ROS 话题：

```zsh
ros2 topic echo /rc_ctrl/button --once
ros2 topic hz /rc_ctrl/right_target
ros2 topic hz /rc_ctrl/left_target
ros2 topic hz /rc_ctrl/button
ros2 topic echo /rc_ctrl/left_target --once
ros2 topic echo /rc_ctrl/right_target --once
```

头显未跟踪到手柄时，`xrt_teleop` 不发布对应位姿，并发布 Grip=0 使机械臂停止；
Trigger 保持最后一次有效值，避免跟踪中断时夹爪突然动作。Joy 现在持续发布，
`ros2 topic hz /rc_ctrl/button` 应显示接近 125 Hz。按住 Grip 后，bridge 与 `rc_ctrl`
日志应分别出现原始 Grip 数值和 `reason=fresh_grip_reference`。位姿频率取决于实际新源帧，
不再由发布定时器把缓存帧伪装成新数据；`source_hz`、`pose_hz` 和实际 servo CSV 间隔须分别核查。

## 安全注意事项

- 首次运行前清空双臂运动范围，确认急停可用，人员不要进入机械臂工作空间。
- 先分别测试左臂和右臂，使用低速度、低夹爪力度，再双人复核后同时操作。
- Grip 是离合器：按住时才跟踪，松开后清除残差并持续发送零增量保持；不执行故障恢复。
- 默认启用 6D 姿态控制；首次调试旋转时必须空载并限制
  `rotation_scale` / `rotation_gain` / `rotation_max_step_rad`。
- 默认不校准或主动开爪；显式 `gripper_initialize:=true` 可能移动夹爪，须按现场流程执行。
- 修改 `rotation_sign`、坐标映射或夹爪方向后，先脱离负载测试。
- AMR 底盘的 X/Y 急停逻辑沿用原 `sr_amr_control` 包，上线前必须按现场安全流程验证。

## 常见问题

**`/opt/ros/humble/setup.bash:.:11` 指向当前工作区的 `setup.sh`**

这说明当前终端继承了错误的 `COLCON_CURRENT_PREFIX`，并不一定是 ROS 安装损坏。
在 zsh 中先清除该变量，再从干净环境加载：

```zsh
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh
```

随后执行 `demo/build.zsh`，再 source `demo/setup.zsh`。如果仍然报错，请新开
一个 zsh 终端后执行上述命令，避免旧终端的 `AMENT_PREFIX_PATH` 和 `PYTHONPATH` 继续
污染环境。

**`source: no such file or directory: /path/to/XRToolKit/demo/install/setup.bash`**

`/path/to/XRToolKit` 是占位符，不能原样执行；且本工程现在使用 `.zsh`。先编译，再
使用实际路径：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
```

**`Package 'jaka_driver' not found`**

这通常表示只加载了 `/opt/ros/humble`，没有加载 `demo/setup.zsh`，或者工作区
从未成功编译。确认下面三条命令分别有输出：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
ros2 pkg prefix jaka_driver
```

**`colcon: error: unrecognized arguments: --log-base`**

`--log-base` 是 colcon 的全局参数，必须写在 `build` 前面。当前 `build.zsh` 已按以下
顺序调用：

```zsh
colcon \
  --log-base "$PWD/log/build" \
  build \
  --build-base "$PWD/build" \
  --install-base "$PWD/install" \
  --packages-up-to xrt_teleop jaka_driver sr_amr_control robot_bringup
```

如果终端仍报这个错误，说明执行的 `build.zsh` 还是旧版本。检查
`grep -n -- "--log-base" build.zsh`，必要时重新同步工程文件后执行
`./clean.zsh && ./build.zsh`。

**`No module named xrobotoolkit_sdk`**

先执行下面的安装脚本，再用工程 venv 中的 Python 验证：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_xrt_sdk.zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -c "import xrobotoolkit_sdk; print('XRT SDK OK')"
```

验证通过后重新执行 `run_teleop.zsh`。不要使用 `/usr/bin/python3` 或
`pip install --user` 验证，本工程运行节点只使用 `demo/.venv`。

**安装时出现 `bash: setup_ubuntu.sh: No such file or directory`**

这是旧安装流程残留的错误。最新版脚本不再调用该文件，而是执行：

```zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -m pip install --upgrade xrobotoolkit-sdk
```

然后通过 `demo/install_xrt_sdk.zsh` 验证 `demo/.venv` 能导入包。

**左臂日志出现 `ERR_INVALID_HANDLER`**

这是左 JAKA 控制器未上电或网络不可达时登录失败。先给左右控制柜上电，确认
`192.168.71.37` 可 ping 通，再重新启动整套 launch。bridge 节点退出时，launch 会同时
关闭双臂节点，因此随后出现的 `rcl context is invalid` 是关停造成的连带日志。

**PC-service 启动失败或在 x86 上运行 ARM64 SDK**

检查 `uname -m`、PC-service 的 `.deb` 架构，以及
`file demo/src/jaka_driver/lib/libjakaAPI.so`。JAKA 主机上两者都必须是 ARM64。

**有 Joy 数据但没有位姿**

确认 Quest 中已开启 Controller Tracking 和 Send，手柄已被头显识别，且 PC-service
与头显处于可互访网络。

**机械臂不动**

先确认 `run_teleop.zsh` 正在运行；D405 发送端本身不会控制机械臂。修改过本轮 Grip
发布代码后，必须先执行“修改代码后的固定编译命令”。然后依次检查：

1. 跑 `./diagnose_xrt.zsh` 时，对应手柄不能是 `untracked`，握住 Grip 时 `grip`
   必须从 0 变为接近 1。
2. 正常运行 `run_teleop.zsh` 时，bridge 日志 `Quest input` 中的 `grip` 应随 Grip
   变化；若一直为 0，问题在 APK、PC-service 或手柄输入。
3. 启动、失联或超期后先松开 Grip，再检查 `reason=fresh_grip_reference`；同时核查新源帧与 tracking 话题。
4. 若日志包含 `latched fault`，保持故障，按下文独立流程由现场人员恢复；普通 Grip 不清错或使能。
5. Joy 发布约 125 Hz，target 取决于新源帧；核查 bridge 的 `source_hz` 和 `pose_hz`。
6. 已处于 `tracking` 后仍不动，检查 CSV 中的原始 servo 返回码、碰撞、`ERR_NOT_ENABLED`、
   `enable_y_limit` 和 JAKA 安全停止状态。

日志中的 `sequence size exceeds remaining buffer` 来自 JAKA SDK 内部的协议调试输出，
在没有 `servo_p failed` 时不是本工程的 JSON/文本解析错误，也不是机械臂不动的直接原因。

若原始 `grip` 变化但话题没有变化，检查 APK 的 Send 和 PC-service；若 `/rc_ctrl/button`
变化且有 `fresh_grip_reference`，但姿态话题没有数据，检查 Controller Tracking；若二者都正常但仍
不动，检查 JAKA 登录、上电、使能、伺服模式和安全停止状态。

**夹爪不动**

确认 `gripper_rs485_channel` 与实际 TIO 接线一致（默认 `1` = RS485L/TIO 通道 2），
确认 PGI 站号和波特率，并检查 JAKA App 中的 TIO 配置。

## 伺服连续性修复（2026-10-28）

### 实际链路、状态与提交语义

实际入口是 `demo/run_teleop.zsh` → `dual_arm_teleop.launch.py` →
`xrt_teleop_bridge` 和两个独立 `rc_ctrl_node` 进程。bridge 经 `xrt_adapter` 完成坐标映射、
四元数和平滑，发布 PoseStamped、Joy、每臂 Int64 `input_state` 恢复屏障及 Bool `tracking` 诊断。
原 IP、Pose/Joy 话题、Grip/Trigger 轴与数字按钮索引保留；输入状态协议见文末第三轮说明。
`XRoboToolkit-Teleop-Sample-Python/` 仅作参考，运行时不切换到它。

每臂只有一个 SDK 所有者线程，执行登录、启动配置、反馈查询、servo、夹爪和退出；
ROS 回调只更新最新输入邮箱，短暂松手/跟踪丢失另有粘性中断标记，避免在阻塞调用期间丢失。
失联时 bridge 合成的 Grip=0 不算人工松手：释放确认及夹爪执行要求 tracking 有效、Pose/Joy
新鲜且具有同一个源帧的 ROS stamp。恢复时仍握持不会自动重新接合；自定义发布者也须遵循同帧 stamp 约定。
运动与保持共用唯一 `servo_p(INCR, step_num)` 调用位置。周期使用单调时钟，从发送决策时刻计算
下一截止时刻；SDK 若已阻塞到截止时刻，不再附加一个完整周期。没有集中补发循环。
本地 SDK 没有明确保证 servo_p 是否阻塞，记录实际调用耗时及开始间隔，不把返回成功等同于到位。

| 状态 | 含义与转换 |
| --- | --- |
| `Hold` / 0 | 新零增量保持；新鲜 Grip 按下、位姿及反馈有效时建立新参考并进入 Tracking |
| `Tracking` / 1 | 输入映射为目标，规划受限增量；成功提交后才推进命令参考 |
| `AwaitRelease` / 2 | 启动、输入失联、跟踪丢失、反馈过期、发送间隔超期或维护后等待新的松手帧；持续握持不能重新接合 |
| `Fault` / 3 | SDK 失败、保护停机、反馈调用或 servo 调用超时等锁存；保存首次错误码、单调时刻、上下文；停止目标和发送 |

最新手柄输入、映射目标、候选轨迹、成功提交状态和机器人反馈分别保存。
平移以“接合时机器人位置 + 手柄相对位移”生成绝对目标，限幅后只按成功提交的增量推进，
消除旧 `ctrl_prev=完整手柄位置` 导致的未发送位移丢失。独立平移软死区与低通抑制噪声，旋转输入
不会让亚毫米平移噪声穿透。已接受的目标残差可在保持同一输入时逐步完成；释放/失联立即清除残差。
软件速度、加速度约束应用于 Tracking 中的轨迹；释放、失联、硬边界和故障停止优先于该约束，
实际减速需核查控制器滤波和 servo 退出。首次接合初始化命令参考，后续握持沿用累计提交参考；
反馈有效性单独检查，不把缓存查询或命令提交当作同步到位。

Y 边界在与反馈/INCR 平移相同的假定参考系内同时限制目标和候选发送位置，左臂不低于
−110 mm，右臂不高于 +110 mm；接合时若已经超界，只禁止继续向外，不生成突然回拉。
这是两臂各自坐标中的单轴约束，不是经证明的整机防碰撞空间。
不写入用户坐标系、TCP、安装姿态、负载和碰撞等级，保留现场标定与碰撞配置。

普通 Grip 不再调用 collision_recover、clear_error、power_on 或 enable_robot。
启动仍可按原流程上电/使能，但先检查控制器错误和保护状态；已就绪时跳过重复上电。
恢复流程：现场确认停止原因并消除风险 → 在控制器独立恢复/清错/检查上电使能与标定 →
停止并重启对应节点 → 检查启动返回码与新鲜数据 → 新松手帧后重新握持。
节点不提供隐式自动恢复。退出不再自动打开可能夹有负载的夹爪。

### 参数与配置入口

新增 `control_config` launch 参数默认读取 `jaka_driver/config/teleop_control.yaml`，两臂共享
限制配置。用 `control_config:=/绝对路径/本次配置.yaml` 指定经现场审核的配置副本。
原姿态、Grip 滞回和 enable_y_limit launch 参数继续生效，并优先于 YAML 中同名项。
为避免误解，默认 YAML 不重复声明这些原 launch 参数。左右臂可通过分别启动 RC 节点配置不同限制。

| 参数 | 默认值 / 单位 | 依据与范围 |
| --- | --- | --- |
| `servo_step_num` | 1；8 ms | 附带头文件约定 step_num×8 ms；1..10，须同时满足 watchdog ≥ 周期 |
| `max_servo_interval_s` | 0.032 s | 4 个默认周期；调度间隔超期清残差等待松手；单次 servo/反馈调用超期锁存故障 |
| `translation_speed_mm_s` | 125 mm/s，每轴 | 采用旧注释的预期速度；旧 5 mm/8 ms 实为 625 mm/s，已移除该常量 |
| `translation_accel_mm_s2` | 500 mm/s²，每轴 | 0→125 mm/s 约 0.25 s 的初始调试值，非 S5 认证安全上限 |
| `translation_deadband_mm` | 0.5 mm | 固定握持原点的连续软死区，每轴最多 0.5 mm 映射偏置 |
| `translation_filter_tau_s` | 0.04 s | 目标低通时间常数，0 禁用；5 mm/s 时约 0.2 mm 额外滞后 |
| `rotation_workspace_rad` | 2.9670597283903604 rad | 相对握持原点 170°，须小于 π；越界清残差并等待释放 |
| `rotation_speed_rad_s` | 0.1875 rad/s | 对应原 0.0015 rad/8 ms，约 10.7°/s |
| `rotation_accel_rad_s2` | 0.75 rad/s² | 约 0.25 s 加速到默认角速度的初始调试值 |
| `rotation_max_step_rad` | 0.0015 rad/次 | 保留原步长上限，另取角速度×周期的较小值 |
| `rotation_deadband_rad` | 0.005 rad | 保留约 0.29° 死区；与平移噪声独立 |
| `rotation_scale/gain/sign` | 0.5 / 0.5 / +1 | gain 以 8 ms 为基准按时间换算；sign 作用于目标旋转，不反复反号误差 |
| `grip_press/release_threshold` | 0.6 / 0.4 | 保留滞回；实际比较为 >，释放条件为 ≤ release |
| `input_timeout_s` / `max_source_age_s` | 0.2 / 0.2 s | 将源剩余有效期转换为单调截止点，执行/接合/夹爪均检查；跨机需同步，未来容差 50 ms |
| `require_tracking` | true | 要求 Int64 输入状态心跳未超时；false 仅关闭该心跳年龄检查，仍须有效源恢复屏障，不能跳过代次隔离 |
| `feedback_period_s` / `feedback_timeout_s` | 0.1 / 0.3 s | SDK 状态缓存设置 100 ms；反馈年龄为查询接收年龄，无硬件采样时刻保证 |
| `y_limit_mm` / `enable_y_limit` | 110 mm / true | 保留现场原单轴边界，参考系仍待确认 |
| `gripper_call_timeout_s` | 0.25 s | 退出 servo 后同步 IO 的返回耗时门限；超时锁存，不恢复 servo；不是可取消的 SDK 超时 |
| `diagnostic_dir` | `log/teleop` | run_teleop 先切到 demo；直接 ros2 启动也应从 demo 执行或指定 demo 内绝对路径 |
| bridge `source_timeout_s` | 0.2 s | 根据 SDK 时间戳是否推进判断，不以位姿变化判断 |
| bridge `require_source_timestamp` | true | 缺少 get_time_stamp_ns 时停止位姿发布；显式 false 仅接收新鲜度回退，并打印能力不足 |

`servo_filter` 是单选：`carte_nlf`（默认）、`none`、`speed_foresight`。默认 NLF 参数保留现场
原值：平移 1500 mm/s、2500 mm/s²、2800 mm/s³；旋转 1120 °/s、1180 °/s²、1720 °/s³，
分别配置于 `nlf_translation_{speed_mm_s,accel_mm_s2,jerk_mm_s3}` 与
`nlf_rotation_{speed_deg_s,accel_deg_s2,jerk_deg_s3}`。这些原 NLF 值高于本轮软件限值，
本轮未声称已优化滤波或可据此防止关节超限。foresight 备选为 buffer=10、kp=0.5。
启动记录所选模式、所有参数和原始返回码，失败日志对应实际配置接口。没有同时启用多个滤波模式。

### 夹爪调度与诊断

PGI 线性/数字映射、地址和寄存器功能保留；握持时只合并最新 Trigger，松开且位姿、跟踪、
Joy 新鲜后执行。退出 servo 成功 → 同步 TIO 写入 → 未失败/未超时才重新进入零保持。
失联产生的中性 Grip 不触发夹爪待执行目标。维护期间的新输入在操作返回后重新读取，
不恢复旧手柄参考；重新接合仍需要新的释放/握持过程。未假定 SDK 支持同进程第二连接或并发 TIO。
SDK 没有 TIO 取消接口，若某个调用永不返回，应用不能抢占该调用或保证关机时限，需控制器侧保护。

每臂 CSV 文件放在 `demo/log/teleop/`，文件名含启动标识。
4096 条缓冲、每秒在 ROS 线程写盘，溢出丢最旧记录并累计 dropped_records，SDK 线程不写盘。
记录输入源的桥接 ROS stamp、接收单调时刻、Grip/跟踪、手柄位姿、映射位移、目标、
限幅前残差、实际发送增量、规划/提交位置、目标/提交姿态、六关节反馈、反馈接收时刻、
servo 起止/间隔/原始返回码、夹爪总调用耗时、状态和首个故障码/时刻/上下文。
启动日志另记 SDK 版本、用户坐标系及定义、TCP 及定义、安装姿态、负载、碰撞等级与读取返回码。
反馈使用附带 SDK 的 RobotStatus 缓存；该接口已在头文件标为 deprecated，OptionalInfoConfig
可能影响字段，采样时间及更新序号不可用。必须与控制器记录对齐后才分析真实跟随误差。
bridge 每 5 秒分别记录 poll_hz/source_hz/pose_hz，servo 频率从 CSV 的实际开始间隔计算，
反馈频率从 feedback 行计算，三者不可互相替代。

### 纯离线验证与环境范围

从仓库根目录执行（Windows Python 3.10、UCRT64 GCC 15.2.0；无需加载机器人库）：

```powershell
python demo/test/run_offline_tests.py --cxx C:/msys64/ucrt64/bin/g++.exe
```

Linux 主机装好 pytest/C++17 编译器后也可执行：

```sh
python3 demo/test/run_offline_tests.py --cxx g++
```

runner 将 GCC 动态库搜索路径限制在其子进程中，修复本机错误加载其他 winpthread 导致的
cc1plus/clock_gettime64 弹窗；不修改系统 PATH。产物、测试缓存和假节点 CSV 均在 demo/build/offline。
编译使用 `-std=c++17 -Wall -Wextra -Wpedantic -Werror -UNDEBUG`，测试断言不会被 Release 配置移除。
包含：orientation_math、PGI 协议、纯控制逻辑（14 组）及包含实际 rc_ctrl_node.cpp、真实 PGI
实现、离线 ROS 邮箱和 fake SDK 定义的集成测试。生产 CMake 不引用 fake/shim。
后者验证 SDK 单一线程、零保持、残差释放、夹爪退出/恢复顺序、长 IO、发送失败、保护错误
0x128000、反复 Grip 不恢复、SDK 延迟与部分启动失败退出。它不链接 libjakaAPI.so，不发网络数据。
另跑 xrt_teleop 31 项（另 9 个子测试）和 sr_amr_control 24 项，检查 launch/bridge Python 语法。
运行记录以 `demo/build/offline/results.txt` 为准。

纯逻辑断言覆盖静止/死区每周期零保持、不重放非零指令、[0,20,20,15,10,5,0] mm 返回原点、
速度/加速度与提交事务、平移/姿态大残差松手重建、输入/反馈过期、SDK 延迟失败、q/-q、
非零初始姿态、正反旋转方向、左右 Y 边界/超界接合/提交失败、16 ms 周期和无效参数。
独立数学修复包括 sign=-1 反复反号误差导致方向发散，以及精确 π 时 q=(0,0,0,1) 与 -q
因有符号零选择相反插值轴；现在用确定性轴符号消除后者。

#### 姿态工作范围与重新接合

旧映射将 +179°/+181° 缩放为 +89.5°/−89.5°。本轮实现受执行约束的 170° 相对姿态工作区，
不实现任意多圈旋转。端点越界或观察到跨 π 路径（如丢帧 +169°→+191°）立即零保持、清残差，
进入 AwaitRelease；返回范围内仍握持也不恢复，必须真实释放再握持建立新手柄原点。
同批完整帧中的中间越界事件会锁存，不能被后续范围内样本覆盖。保留 q/-q、非零初始姿态和正反方向。
没有观测到的完整转圈不能仅由端点四元数识别，这不是任意路径安全证明。

本机缺 ROS 2/ament/colcon，附带库为 Linux AArch64 .so，不能在 Windows 完成真实链接。
ROS shim 检查生产源与真实 SDK 声明的 C++ 兼容性，不代表真实 ROS 构建/运行验证。
现有 launch 测试需要 XRT_VENV，runner 仅为声明检查创建不执行的 bin/python 文件标记。
集成测试把发送 watchdog 放宽到 120 ms、夹爪门限降至 30 ms，以隔离 Windows 调度噪声；
默认 32 ms 的具体超时语义由可控模拟时间测试覆盖，不声称 Windows 达到实机实时性。
目标 Linux/ARM 主机需另行运行 `./build.zsh`、`colcon test --packages-select jaka_driver xrt_teleop sr_amr_control`
和 `colcon test-result --verbose`，之后才做设备验证。

### SDK、标定与现场待确认项

参考 [JAKA C++ 文档](https://www.jaka.com/docs/legacy172/guide/1.7.2/SDK/Cpp.html)、
[Servo 滤波说明](https://www.jaka.com/docs/legacy172/guide/1.7.2/SDK/Filter%20setting.html)、
[报警查询](https://www.jaka.com/zh/commonQuestion) 及
[XRoboToolkit Python binding](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind)。
在线 legacy172 页中的扩展参数比附带头文件多，不可据网页假定本地二进制支持队列长度、
异步控制或新的滤波姿态参数。本轮以附带声明编译；实机 SDK/控制器版本需启动日志和厂商比对。
报警页本次工具读取因页面过大失败，未据此重新解释报警。

附带头文件仅规定周期和 INCR 模式，未完整定义 servo_p 的 RPY 叠加方式、旋转组合方向、
增量平移参考系与当前用户坐标的关系。实现明确使用 Rz Ry Rx、参考系左乘的小旋转假设，
与既有四元数链保持一致；用非零 TCP 初始姿态、分别绕三个轴的微小正反旋转，在官方仿真或
厂商示例中区分左乘/右乘与 RPY 分量相加，再确认本实现。未经确认不能把离线数学自洽当作 SDK 契约。

左臂 USRFRM15、右臂“世界坐标系 + TCP15”需分别读取比对；bridge 的 world frame_id 是元数据，
共用映射不会自动应用或验证两臂用户坐标标定。本轮不改标定，Y 边界同参考系假设也需现场核验。
上游 get_time_stamp_ns 在本地参考代码与官方 binding 文档中存在，但未获得所装二进制实现，
必须确认它随实际设备新帧推进且断连后停止。getter 之间没有已确认的原子快照保证；没有确认
每个手柄独立的跟踪/连接标志，因此全局源仍推进而单侧缓存冻结的情况尚不能完全检测。

J5≈−180°及提供的右臂角度只作为诊断线索。本轮未以任意阈值判断 S5 奇异，也未实施完整 IK
或 servo_j 迁移。必须使用准确 S5 模型、关节零位和六关节时序反馈，区分近奇异、目标不连续、
伺服延迟与控制器跟随误差，才能归因腕部上下晃动及 0x128000 的实际触发顺序。

现场验证按以下顺序分项记录，由现场人员另行执行：

1. 脱离设备先完成目标主机真实构建/离线测试，核对 SDK 与控制器版本、CSV 输出和现有标定备份。
2. 保持原碰撞保护，分别读取左右用户坐标/TCP、侧装姿态与负载；核查共用映射和 Y 方向/边界。
3. 用 input_probe/bridge 日志验证静止新帧、停 Send、断连、单侧遮挡与恢复；确认时间戳语义。
4. 单臂空载低速，先位置控制，核查静止零流、慢动、往返原点、较大残差松手及重新握持无追赶。
5. 测试 Trigger 排队、松手夹爪动作和重新接合；核对 SDK 阻塞耗时、退出/恢复和输入新鲜度。
6. 在官方仿真或厂商确认姿态语义后，非零初始姿态下逐轴微小正反旋转，再测试 q/-q 与姿态边界。
7. 现场按正式流程验证输入丢失、延迟和保护停机；故障持续握持不能恢复，独立恢复后重启验证。
8. 单臂通过后再验证双臂；由厂商/现场人员规划接近可疑姿态的专项试验，同步保存控制器原始报警、
   六关节反馈与主机 CSV。用记录补充晃动改善结论，不能以本次离线通过代替实机结果。

## 第二轮审查修复：时间参考、输入配对与恢复持载（2026-10-28）

保留单一 SDK 所有者、唯一 servo_p 调用点、故障锁存、成功提交后推进和断开设备的测试结构。
先运行旧实现回归，失败证据见 demo/build/offline/regressions_before_{core,bridge,gripper,input}.txt。
坐标系、TCP、负载、安装姿态与碰撞配置保持；未连接设备。

### 七项根因与执行规则

1. **缓存反馈重接合**：旧 engage 每次覆盖 submitted。现在只在首次接合以静止启动反馈初始化
   命令 epoch，后续握持沿用累计成功提交状态建立新手柄原点，反馈仅作有效性和观察。
   101.76 mm 重握后保持 101.76；测试使用 104 ms 反馈更新、80 ms 延迟执行、两次反馈间多次重握，
   接受累计和实际执行 Y 都不超过 110。查询缓存不是同步屏障。
   假设 INCR 基于上一条已接受命令累计；若 servo 退出/重入会丢弃指令并重建实际位置参考，
   必须独立处理该 epoch，不能用重新查询缓存替代同步。首次绝对参考也依赖静止启动前提。
2. **Grip/Trigger 有效性**：保留数值并独立保存读取成功标志；异常、NaN/Inf、越界不作为有效 0。
   axes[0..7]、buttons[0..11] 不变，追加 buttons[12/13]=左右 GripValid、[14/15]=左右 TriggerValid。
   缺失或任一对应标志无效即等待、禁止释放确认和夹爪；读取恢复但仍握持不解除等待。
   源/位姿失效也清有效标志，自定义发布者必须升级该协议。
3. **双期限**：每回调计算单调 deadline=接收时刻+min(接收 TTL,源剩余有效期)，配对帧取两者较早值。
   150 ms 旧消息在 200 ms 上限下只剩 50 ms，规划、重接合、夹爪（退出伺服后再次确认）共用。
   重复帧不续期，精确门限以可控时间测试，源 ROS 时钟仍需同步。
4. **有界完整帧配对**：teleop_input.hpp 按纳秒 stamp 配对，部分/完整帧各最多 32 项。
   源序处理事件后仅作一次发送决定，Pose(k+1)/Joy(k) 不会覆盖已经完整的释放帧。
   原始接收时刻用于期限和诊断；core 用独立递增序号接收完整帧，允许较新源帧更早到达。
   过时重放忽略，新读取错误/源失效清队列；源 SDK 重启经失效事件隔开旧部分帧。
   缓冲溢出计数并等待；配对只证明同一桥接 stamp，不证明上游 getters 原子硬件采样。
5. **慢动**：每轴固定握持原点的 sign(x)*max(abs(x)-deadband,0) 连续软死区，再加 tau=40 ms 目标低通，
   不再累计 0.5 mm 跳一次目标。保留速度/加速度约束，释放清除滤波和轨迹残差。
6. **跨 π**：采用上文 170° 工作区，越界在规划前锁存等待，不追逐翻转目标，真实释放后才能重建参考。
7. **恢复夹爪**：默认 gripper_initialize=false、gripper_startup_open=false、
   gripper_configure_motion_parameters=false；不隐式写校准/力度/速度/开爪 RTU 寄存器。
   启动仍可能按现有配置设置 TIO 电压/通信，不能称为完全无硬件写入；其持载影响需现场核查。
   第一个有效 Trigger 只建立基线；之后明确变化才在有效释放后执行。
   第一次主动位置命令先懒加载配置力度/速度，再写位置，以后只写位置。
   显式 initialize=true 会写 0x0100，硬件校准可能移动；startup_open=true 才主动写开爪位置 0x0103。
   没有实际夹爪状态回读，has_position_command/closed 仅描述已提交目标。

### 相同输入下的量化回归

| 场景 | 修改前 | 修改后 |
| --- | --- | --- |
| Y=101.76、反馈前重握 | submitted 回退 100 mm | submitted 保持 101.76，延迟执行的累计 Y≤110 |
| 5 mm/s、125 Hz，1 s 后稳态 | 最小 0、最大 16、均值 4.98403 mm/s | 最小/最大/均值 5 mm/s |
| 同场景速度波动 RMS | 5.48449 mm/s | 0 mm/s（数值精度内） |
| 原点 ±0.2 mm 噪声 5000 周期 | 保留原静止断言 | 累计漂移 0 |
| 远离原点 ±0.2 mm 噪声 5000 周期 | 非本轮前后对比项 | 位置扰动峰值 0.032 mm，窗口均值差约 0.0104 mm；回中心误差 1.42e-14 mm |
| 持载位置 350、两次重启 | 两次写 0x0103=1000 | 默认无上述 RTU 写入，模拟位置仍为 350 |

软死区引入每轴最多 0.5 mm 映射偏置，40 ms 滤波在 5 mm/s 时另有约 0.2 mm 滞后。
这些为明确的噪声/跟随折衷，不能据此宣称零误差或实机完全不抖。

### 离线验证与参数

执行 `python demo/test/run_offline_tests.py --cxx C:/msys64/ucrt64/bin/g++.exe`，
保留原回归，新增可控时间配对/期限、真实 RTU 内容/CRC 启动策略、实际节点 source/pairing/validity 专项。
核心共 14 组；XRT 26 项+9 子测试、AMR 24 项；完整子命令及退出码见 demo/build/offline/results.txt。
新增 translation_filter_tau_s=0.04s、rotation_workspace_rad=2.9670597283903604rad，三项夹爪启动策略均默认 false。
测试不加载厂商库、不通信设备；Windows 仍不能替代目标 ROS2/ARM64 构建及实机验证。

夹爪多次写入补充：每次同步 RTU 写入前复查同一源/接收截止点、有效配对释放和 Trigger 是否仍为该请求。
首个力度设置若阻塞到过期，不再发送后续速度/位置；已在 SDK 内的调用无法取消。
输入授权取消时等待重新释放；接口总耗时超过 gripper_call_timeout_s 仍锁存超时故障。
新增 gripper_deadline 实际节点回归先复现旧路径过期后写位置，再验证取消位置写入。
发送非零 servo 前也再次检查截止点，过期改为本周期新零增量，提交状态不推进。

## 第三轮小范围修复：在途帧代次与未配对 Joy 撤销（2026-10-28）

本轮仅修改输入关联/请求授权、对应 bridge 状态协议和离线回归。第二轮控制核心、软死区/低通、
170° 姿态范围、双期限数值、夹爪启动/校准策略均保持；所有 SDK 调用仍由同一所有者执行。

### 1. 恢复屏障从源端产生

单靠 invalidate 把 floor 推到已见 stamp，无法隔离失联前生成却整组未到达的 release/press/move。
新增 `/rc_ctrl/left_input_state`、`/rc_ctrl/right_input_state`，类型 `std_msgs/Int64`：

- 正数 `+B`：该臂恢复后首个有效源帧的 bridge ROS 纳秒 stamp，作为固定恢复屏障。
  健康期间每次心跳重复同一 B，不跟随每个后续帧推进。
- 负数 `-L`：源端首次观察到该臂源/位姿失效时的 ROS 纳秒 stamp，失效期间重复同一 L。
- 首次健康状态也建立屏障；首次无源时发送负状态。原 Bool tracking 保留用于诊断，
  不再授权或解除等待。旧发布者必须同时升级 Int64 状态协议，只有 Bool 的发布者不能接合。
  每臂节点可用 `input_state_topic` 配置输入状态话题，bridge 对应 `left/right_input_state_topic`。

接收端以恢复状态 B 设置包含边界 `stamp >= B`，清除旧部分帧/完整帧及旧释放事件。
即使旧 release/press/move 的 stamp 大于此前 seen，只要生成于恢复屏障前也会丢弃；
恢复后仍握持保持 AwaitRelease，只有恢复后的有效配对释放及再握持可接合。
每个正心跳都携带 B，因此负转换消息被 depth=1 合并时，新正屏障仍能隔离旧代次。
这是一项源协议屏障，不是接收时刻、最大已见帧或“重新查询缓存即同步”。

标识及顺序假设：每臂只有一个状态发布源；该状态话题保持发布顺序，Pose/Joy 按各自 stamp 配对，
不依赖不同话题的到达先后。B/L 与 Pose/Joy 都使用同一 bridge ROS 时钟，源端帧/失效/恢复事件的
stamp 必须跨代次严格递增。原始 SDK 时间戳仍只是新帧进展标记，不直接当作 ROS 时钟。
源状态时间倒退、零值或同时间冲突时保守清输入并等待，不伪造更大代次；须出现严格更新的
有效恢复屏障后才能继续。clock 回退/暂停不能用普通 Grip 修复，需按现场流程恢复时钟/状态源。
源与接收的时钟同步及年龄限制继续由既有双期限检查承担；同 stamp 仍不证明 SDK getters 原子采样。
`require_tracking=false` 只放宽状态心跳超时，不跳过源代次屏障或源/接收双期限。

### 2. 新有效 Joy 可以先撤销，不能先授权

旧 guard 只看完整配对帧，因此力度写阻塞时，新 Joy(Grip=0,Trigger=0) 若缺 Pose，
仍可能写入旧 Trigger=0.8 的位置 0x0103=200。现在按有效 Joy 的源 stamp 和原始 Trigger 值保存
独立候选请求及其撤销锁存。较新有效 Joy 相对该请求原值的变化达到 gripper_trigger_deadband，
该请求即持续撤销；改回原值也不会复活旧请求，必须以较新的完整帧重新授权。
新无效或已被取代的 Joy 不作为新释放/新请求；过时 Joy 重放不会更新撤销标记。

所有者在夹爪调度前、退出伺服后和每次寄存器写入前，都检查同一请求是否尚未撤销。
因此 Joy 可以先阻断旧力度/速度/位置的后续写入，而新的动作继续要求新鲜、有效、完整配对的真实释放。
这不能取消已经进入 SDK 的同步调用，也不能撤回已经提交给控制器的寄存器写入。
CSV 追加精确 source_stamp_ns、原始 input_state_ns 和 state_accepted，便于关联恢复代次与请求。

### 修改前/后证据及离线运行

先在本轮基线编译实际节点链接 fake SDK 并运行新增场景，两项均退出 1：

- 在途完整帧：运动场景出现非零 servo；夹爪场景旧位置写入 1 次。
- 力度写阻塞 35 ms 后，撤回 Joy 已到、Pose 延迟，仍写入旧 0x0103=200。

修复后同场景退出 0：旧在途帧的非零 servo 与旧位置写入均为 0；恢复握持保持等待，
新的配对释放/再握持可运动；撤回后旧位置写入为 0，新的完整 Trigger=0.7 可写入目标 300，
其力度/速度写入期间重放旧 Joy 不会撤销该新请求。
日志：`demo/build/offline/regressions_before_input_events.txt`、`regressions_after_input_events.txt`；
bridge 屏障回归的修复前/后记录另见 `regressions_before/after_tracking_barrier.txt`。

完整命令保持：

```powershell
python demo/test/run_offline_tests.py --cxx C:/msys64/ucrt64/bin/g++.exe
```

另可执行生成的 `demo/build/offline/rc_ctrl_node_offline_test.exe inflight_epoch` 和 `trigger_cancel`。
保留原 14 组控制核心、软死区量化、姿态范围、双期限、夹爪启动策略、乱序/重复帧/上限/异常读取回归；
新增可控时间恢复屏障、丢失负转换心跳、状态时钟倒退、未配对撤销/改回及旧 Joy 重放测试。
本轮 XRT 为 31 项+9 子测试，AMR 原 24 项；所有生成文件在 demo 内。

仍需现场核验：INCR 累计基准及 servo 退出/重入契约、SDK 缓存采样时刻及 getters 原子性、
源帧真实推进/断连行为、坐标标定与旋转语义、同步 TIO 调用的不可抢占时延。
本轮没有修改现场坐标系、TCP、负载、碰撞保护；离线通过不代表真实 ROS2/ARM64 链接或运动验收。

### Trigger 请求值绑定的撤销锁存补充

全局变化基准曾漏掉 `0.81 → 0.799（请求）→ 0.82 → 0.799`：后两值距全局 0.81 都不足 0.02，
但 0.82 距实际请求 0.799 为 0.021。现按每个请求的源 stamp 和不可变 Trigger 原值比较，
达到门限即锁存该请求已撤销。记录在有效 Joy 到达时建立，Pose 尚未配对也能撤销；
动作授权仍需完整配对、原有源/接收期限及恢复屏障。源序更旧的 Joy 不能撤销较新请求。
请求记录沿用配对器的容量上限，淘汰记录只使该旧请求失效；退休 stamp 屏障禁止重放重建已淘汰记录。
现有节点接线无需变更，运动核心、参数、状态协议、SDK 线程和夹爪启动实现均未改动。

在现有实际节点 `trigger_cancel` 回归中增加上述数值：握持 0.81 后 0.799，释放完整请求 0.799；
力度写阻塞 35 ms 时先接收未配对 Joy=0.82，4 ms 后接收 Joy=0.799。
旧实现仍写一次 `0x0103=201`；修复后，延迟 Pose 到达前所有位置写入为 0，
该较新 0.799 的 Pose 到达形成完整帧后可写新位置 201；期间重放旧 0.82 不会撤销新请求。
修复前后记录位于 `demo/build/offline/regressions_before_request_latch.txt` 和
`regressions_after_request_latch.txt`，完整验证继续执行原 `demo/test/run_offline_tests.py`。
