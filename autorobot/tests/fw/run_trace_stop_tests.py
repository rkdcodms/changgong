"""Compile/run the hardware-free trace_stop firmware harness. Never opens a device."""
from pathlib import Path
import argparse, hashlib, json, subprocess
p = argparse.ArgumentParser()
p.add_argument('--cxx', default='g++')
p.add_argument('--work-dir', required=True, type=Path)
args = p.parse_args()
tests = Path(__file__).resolve().parent
sketch = tests.parents[1] / 'arduino/trace_stop/trace_stop.ino'
build = args.work_dir.resolve(); build.mkdir(parents=True, exist_ok=True)
(build / 'sources.h').write_text('#define TRACE_STOP_SKETCH ' + json.dumps(sketch.as_posix()) + '\n', encoding='utf-8')
exe = build / 'test_trace_stop.exe'
subprocess.run([args.cxx, '-std=c++17', '-I', str(tests), '-I', str(build), str(tests / 'test_trace_stop.cpp'), '-o', str(exe)], check=True)
r = subprocess.run([str(exe)], capture_output=True, text=True)
print(r.stdout, end=''); print(r.stderr, end='')
print('firmware sha256', hashlib.sha256(sketch.read_bytes()).hexdigest())
raise SystemExit(r.returncode)
