from pathlib import Path
import subprocess
import sys
import os
root=Path(__file__).resolve().parents[1]
build=root/'tests/build';build.mkdir(exist_ok=True)
gcc=Path('E:/CLion 2025.1.2/bin/mingw/bin/gcc.exe')
if not gcc.exists():raise SystemExit('Provide a Windows GCC compiler in tools/test_native.py')
env=os.environ.copy();env['PATH']=str(gcc.parent)+os.pathsep+env.get('PATH','')
# The bench override/diagnostic switches are compile-time; the host suite always
# builds the PRODUCTION protection semantics so those tests stay meaningful.
subprocess.run([str(gcc),'-std=c99','-O2','-Wall','-Wextra','-shared','-static-libgcc','-Wl,--export-all-symbols',
                '-DPD_OVERRIDE_NO_LATCH=0','-DPD_DIAG_MASK_OC=0','-I','App','App/pd_modulation.c', 'App/pd_diag.c','App/pd_core.c','App/pd_pid.c',
                'App/pd_zero.c','App/pd_power.c','App/pd_oc.c','App/pd_cmp.c','tests/host_port.c','-o','tests/build/firmware_core.dll'],
               cwd=root,env=env,check=True)
subprocess.run([sys.executable,str(root/'tests/test_firmware.py')],check=True)
subprocess.run([sys.executable,str(root/'tests/test_modulation.py')],check=True)
