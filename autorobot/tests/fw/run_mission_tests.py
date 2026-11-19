"""Compile/run the hardware-free MS1 mission firmware harness. Never opens a device."""
from pathlib import Path
import argparse, subprocess, json, hashlib
p = argparse.ArgumentParser()
p.add_argument('--cxx', default='g++')
p.add_argument('--work-dir', required=True, type=Path)
args = p.parse_args()
tests = Path(__file__).resolve().parent
root = tests.parents[1]
vision = root/'arduino/vision_stop/vision_stop.ino'
mission = root/'arduino/mission_stop/mission_stop.ino'
build = args.work_dir.resolve(); build.mkdir(parents=True, exist_ok=True)
(build/'sources.h').write_text('#define VISION_SKETCH ' + json.dumps(vision.as_posix()) +
                               '\n#define MISSION_SKETCH ' + json.dumps(mission.as_posix()) + '\n', encoding='utf-8')
exe = build/'test_mission_stop.exe'
subprocess.run([args.cxx, '-std=c++17', '-Wall', '-Wno-unused-variable', '-Wno-unused-function',
                '-I', str(tests), '-I', str(build), str(tests/'test_mission_stop.cpp'), '-o', str(exe)], check=True)
r = subprocess.run([str(exe)], capture_output=True, text=True)
print(r.stdout, end=''); print(r.stderr, end='')
record = {'status': 'PASS' if r.returncode == 0 else 'FAIL',
          'mission_sha256': hashlib.sha256(mission.read_bytes()).hexdigest(),
          'vision_sha256': hashlib.sha256(vision.read_bytes()).hexdigest(),
          'output': r.stdout, 'stderr': r.stderr, 'hardware_accessed': False}
(build/'mission_validation.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
raise SystemExit(r.returncode)
