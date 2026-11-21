#!/usr/bin/env python3
"""Repeat local geometry/protocol/model tests; never connects to the robot."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from control import packet,runtime_info

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-root',type=Path,default=HERE.parents[1]/'third_party/Happy-Baby-R1-develop')
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();cfg=json.loads((HERE/'config.json').read_text())
    info=runtime_info(a.runtime_root,cfg);out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    root=a.runtime_root.resolve();ort=root/'controller/thirdparty/onnxruntime'
    (out/'packets.bin').write_bytes(packet(1,[0.]*12,False,False,False)+packet(2,[0.]*10+[.1,0.],True,False,True))
    commands=[
        [sys.executable,str(HERE/'test_control.py')],
        [sys.executable,str(HERE/'test_stream.py')],
        [sys.executable,str(HERE/'test_vision_pipeline.py')],
        ['g++','-std=c++17','-O2','-pthread','-I',str(root/'controller/src'),str(HERE/'test_receiver.cpp'),'-o',str(out/'test_receiver')],
        [str(out/'test_receiver'),str(out/'packets.bin')],
        ['g++','-std=c++17','-O2','-I',str(ort/'include'),str(HERE/'test_policy.cpp'),str(ort/'lib/libonnxruntime.so.1.15.1'),'-Wl,-rpath,'+str(ort/'lib'),'-o',str(out/'test_policy')],
        [str(out/'test_policy'),info['policy']],
    ]
    logs=[]
    for index,cmd in enumerate(commands):
        result=subprocess.run(cmd,capture_output=True,text=True,env={**os.environ,'R1_TARGET_TEST_RUNTIME_ROOT':str(root)})
        (out/f'{index:02d}.log').write_text(result.stdout+result.stderr)
        logs.append({'command':shlex.join(cmd),'returncode':result.returncode,'log':f'{index:02d}.log'})
        if result.returncode:
            (out/'results.json').write_text(json.dumps(logs,indent=2)+'\n')
            raise RuntimeError(f'check {index} failed; see {out}/{index:02d}.log')
    (out/'results.json').write_text(json.dumps(logs,indent=2)+'\n')
    (out/'runtime_reference.json').write_text(json.dumps(info,indent=2)+'\n')
    (out/'command.txt').write_text(shlex.join([sys.executable,*sys.argv])+'\n')
    (out/'source_sha256.json').write_text(json.dumps({str(s):hashlib.sha256(s.read_bytes()).hexdigest() for s in HERE.iterdir() if s.is_file()},indent=2)+'\n')
    snap=out/'source_snapshot';snap.mkdir()
    for source in HERE.iterdir():
        if source.is_file():(snap/source.name).write_bytes(source.read_bytes())
    print(json.dumps({'checks_passed':len(logs),'output_dir':str(out),'scope':'local math, owner packet contract and finite ONNX inference; no closed-loop balance or hardware acceptance'},indent=2))


if __name__=='__main__':main()
