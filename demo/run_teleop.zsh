#!/usr/bin/env zsh

setopt ERR_EXIT

typeset _run_file="${(%):-%N}"
typeset _run_demo_dir="${_run_file:A:h}"

source "${_run_demo_dir}/setup.zsh"

"${_run_demo_dir}/launch_quest.zsh" || \
  print -u2 "自动启动 Quest APK 失败，请检查 adb；ROS 遥操作仍会继续启动。"

exec ros2 launch jaka_driver dual_arm_teleop.launch.py "$@"
