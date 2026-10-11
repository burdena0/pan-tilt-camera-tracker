import os
from pathlib import Path
import shutil
import subprocess
ROOT=Path(__file__).resolve().parents[1]
directory=ROOT/'vision/output';directory.mkdir(parents=True,exist_ok=True)
gcc=shutil.which('gcc')
if gcc is None and os.name=='nt':gcc='C:/msys64/ucrt64/bin/gcc.exe'
common=[gcc,'-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'main'),str(ROOT/'main/vision_protocol.c')]
test=directory/('protocol_test.exe' if os.name=='nt' else 'protocol_test')
subprocess.run(common+[str(ROOT/'tools/test_vision_protocol.c'),'-o',str(test)],check=True)
subprocess.run([str(test)],check=True)
library=directory/('vision_guard.dll' if os.name=='nt' else 'vision_guard.so')
subprocess.run(common+['-shared','-fPIC','-o',str(library)],check=True)
print(library)
