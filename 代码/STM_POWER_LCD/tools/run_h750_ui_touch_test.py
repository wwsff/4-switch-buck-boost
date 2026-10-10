"""Exercise actual H750 UI/touch/CAN code with a host framebuffer and fake I2C."""
from pathlib import Path
import os
import subprocess

src = Path(__file__).resolve().parents[1] / "STM_POWER_LCD"
build = src / "User/HostTest/build"
build.mkdir(parents=True, exist_ok=True)
gcc = Path("E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe")
env = os.environ.copy()
env["PATH"] = str(gcc.parent) + os.pathsep + env.get("PATH", "")
includes = ["User/HostTest/shim", "User/App/Inc", "User/Ui/Inc", "User/Shared", "User/Bsp/Inc"]
sources = ["User/Ui/Src/ui_main.c", "User/Ui/Src/ui_model.c", "User/Ui/Src/gfx_text.c",
           "User/Bsp/Src/touch.c", "User/App/Src/can_proto.c", "User/HostTest/h750_ui_touch_test.c"]
exe = build / "h750_ui_touch_test.exe"
cmd = [str(gcc), "-std=c99", "-O1", "-Wall", "-Wextra", "-Werror"]
for folder in includes:
    cmd += ["-I", str(src / folder)]
cmd += [str(src / path) for path in sources] + ["-o", str(exe)]
subprocess.run(cmd, cwd=build, env=env, check=True)
subprocess.run([str(exe)], cwd=build, env=env, check=True)
