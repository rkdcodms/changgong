"""Compile/run a hardware-free C++ harness. This never opens a device."""
from pathlib import Path
import argparse, subprocess, json, hashlib
p=argparse.ArgumentParser()
p.add_argument('--cxx',required=True)
p.add_argument('--work-dir',required=True,type=Path)
args=p.parse_args()
tests=Path(__file__).resolve().parent
original=tests.parent/'fixtures/line_tracer_baseline.ino'
vision=tests.parents[1]/'arduino/vision_stop/vision_stop.ino'
build=args.work_dir.resolve();build.mkdir(parents=True,exist_ok=True)
(build/'sources.h').write_text('#define ORIGINAL_SKETCH '+json.dumps(original.as_posix())+'\n#define VISION_SKETCH '+json.dumps(vision.as_posix())+'\n',encoding='utf-8')
prefix=[args.cxx]+(['c++'] if Path(args.cxx).stem.lower()=='zig' else [])
exe=build/'test_vision_stop.exe'
cmd=prefix+['-std=c++17','-I',str(tests),'-I',str(build),str(tests/'test_vision_stop.cpp'),'-o',str(exe)]
subprocess.run(cmd,check=True)
r=subprocess.run([str(exe)],capture_output=True,text=True)
print(r.stdout,end='');print(r.stderr,end='')
record={'status':'PASS' if r.returncode==0 else 'FAIL','source_sha256':hashlib.sha256(vision.read_bytes()).hexdigest(),'output':r.stdout,'stderr':r.stderr,'hardware_accessed':False}
(build/'validation.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
raise SystemExit(r.returncode)
