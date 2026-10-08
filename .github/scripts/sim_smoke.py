"""Headless sim smoke test: the robot must run 5 s in sim without exiting."""

import subprocess
import sys
import time

proc = subprocess.Popen(
    [sys.executable, "-m", "robotpy", "sim", "--nogui"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
)
time.sleep(5)
if proc.poll() is not None:
    print(proc.stdout.read())
    print(f"Sim exited early with code {proc.returncode}")
    sys.exit(1)
proc.terminate()
proc.wait(timeout=5)
print("Sim ran for 5s without error - OK")
