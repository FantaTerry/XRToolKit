#!/usr/bin/env python3
"""Compile/run disconnected control tests; never load the vendor robot library.

Run from any directory: python demo/test/run_offline_tests.py
Builds, bytecode and pytest caches remain under demo/. The ROS shim verifies
C++ and bundled SDK declarations only; a real ROS 2 colcon build is still needed
on the target. Neither this runner nor its executables contact a device.
"""

import argparse
import ctypes
import os
from pathlib import Path
import shutil
import subprocess
import sys


DEMO = Path(__file__).resolve().parents[1]
DRIVER = DEMO / "src" / "jaka_driver"
BUILD = DEMO / "build" / "offline"


def run(command, *, env=None):
    heading = "+ " + subprocess.list2cmdline([str(part) for part in command])
    print(heading, flush=True)
    result = subprocess.run(
        [str(part) for part in command], cwd=DEMO, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding="utf-8", errors="replace")
    print(result.stdout, end="", flush=True)
    with (BUILD / "results.txt").open("a", encoding="utf-8") as transcript:
        transcript.write(heading + "\n" + result.stdout + "EXIT " + str(result.returncode) + "\n")
    result.check_returncode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "g++"))
    args = parser.parse_args()
    compiler = shutil.which(args.cxx)
    if compiler is None:
        parser.error("C++ compiler unavailable; set --cxx to a C++17 compiler")
    BUILD.mkdir(parents=True, exist_ok=True)
    (BUILD / "results.txt").write_text(
        "Disconnected tests; real ROS 2 build and ARM64 vendor-library link are not executed.\n",
        encoding="utf-8")
    env = os.environ.copy()
    if os.name == "nt":
        # GCC's cc1plus lives below lib/gcc, so its DLLs are found via PATH.
        # Other installed applications can supply an incompatible winpthread.
        # Scope the toolchain path to children; never edit the user's settings.
        compiler_bin = str(Path(compiler).resolve().parent)
        env["PATH"] = os.pathsep.join(
            [compiler_bin, str(Path(os.environ.get("SystemRoot", "C:/Windows")) / "System32"),
             os.environ.get("SystemRoot", "C:/Windows")])
        # Failed compiler child loading must return an error, not a modal dialog.
        ctypes.windll.kernel32.SetErrorMode(0x0001 | 0x0002 | 0x8000)
    # Keep Python outputs in demo even if invoked from the repository root.
    env["PYTHONPYCACHEPREFIX"] = str(BUILD / "pycache")
    env["PYTHONPATH"] = os.pathsep.join(
        [str(DEMO / "src" / "xrt_teleop"),
         str(DEMO / "src" / "sr_amr_control"), env.get("PYTHONPATH", "")])
    options = [compiler, "-std=c++17", "-Wall", "-Wextra", "-Wpedantic",
               "-Werror", "-UNDEBUG", "-D_USE_MATH_DEFINES", "-I", DRIVER / "include"]
    extension = ".exe" if os.name == "nt" else ""
    for name in ("orientation_math_test", "pgi140_protocol_test", "teleop_control_test", "teleop_input_test", "pgi140_gripper_startup_test"):
        sources = [DRIVER / "test" / (name + ".cpp")]
        if name == "pgi140_protocol_test":
            sources.append(DRIVER / "src" / "pgi140_protocol.cpp")
        if name == "pgi140_gripper_startup_test":
            sources.extend([DRIVER / "src" / "pgi140_gripper.cpp", DRIVER / "src" / "pgi140_protocol.cpp"])
        output = BUILD / (name + extension)
        run(options + sources + ["-o", output], env=env)
        run([output], env=env)
    # Compile the actual production source against the real bundled SDK header.
    # Only ROS declarations are substituted; there is no vendor library linking.
    run(options + ["-DXRTOOLKIT_OFFLINE_ROS_SHIM", "-I", DEMO / "test" / "ros_stubs",
                   "-fsyntax-only", DRIVER / "src" / "rc_ctrl_node.cpp",
                   DRIVER / "src" / "pgi140_gripper.cpp"], env=env)
    node_test = BUILD / ("rc_ctrl_node_offline_test" + extension)
    run(options + ["-DXRTOOLKIT_OFFLINE_ROS_SHIM", "-I", DEMO / "test" / "ros_stubs",
                   DEMO / "test" / "rc_ctrl_node_offline_test.cpp",
                   DRIVER / "src" / "pgi140_gripper.cpp",
                   DRIVER / "src" / "pgi140_protocol.cpp", "-o", node_test], env=env)
    run([node_test], env=env)
    run([node_test, "source"], env=env)
    run([node_test, "pairing"], env=env)
    run([node_test, "validity"], env=env)
    run([node_test, "gripper_deadline"], env=env)
    run([node_test, "inflight_epoch"], env=env)
    run([node_test, "trigger_cancel"], env=env)
    # The existing launch unit test checks the configured interpreter path but
    # does not execute it. Supply an explicitly inert fixture on non-ROS hosts.
    # This does not make a claim about a working project virtual environment.
    fixture = BUILD / "launch_fixture" / "bin" / "python"
    fixture.parent.mkdir(parents=True, exist_ok=True)
    fixture.write_text("offline launch declaration fixture only\n", encoding="utf-8")
    env["XRT_VENV"] = str(fixture.parent.parent)
    for package in ("xrt_teleop", "sr_amr_control"):
        run([sys.executable, "-m", "pytest", "-q", "-o",
             "cache_dir=" + str(BUILD / ("pytest_" + package)),
             DEMO / "src" / package / "test"], env=env)
    run([sys.executable, "-m", "compileall", "-q",
         DEMO / "src" / "jaka_driver" / "launch",
         DEMO / "src" / "xrt_teleop" / "xrt_teleop"], env=env)
    print("PASS: offline C++ assertions, SDK declaration syntax check, Python tests and syntax.")
    print("Not executed: real ROS 2 build, ARM64 vendor library link or robot motion.")


if __name__ == "__main__":
    main()
