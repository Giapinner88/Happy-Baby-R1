#!/usr/bin/env python3
"""Run P4 offline tests and demonstration cases, saving reproducible evidence."""
import argparse
import hashlib
import shlex
import platform
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'command.txt').write_text(shlex.join([sys.executable,*sys.argv])+'\n')
    sources=[HERE/name for name in ['verify.py','test_search.py','search.py','backends.py','feedback.py','run.py','config.json']]
    snap=out/'source_snapshot';snap.mkdir()
    for source in sources:(snap/source.name).write_bytes(source.read_bytes())
    (out/'source_sha256.json').write_text(json.dumps({str(s):hashlib.sha256(s.read_bytes()).hexdigest() for s in sources},indent=2)+'\n')
    (out/'environment.json').write_text(json.dumps({'python':sys.version,'platform':platform.platform()},indent=2)+'\n')
    hardware=[sys.executable,'-m','unittest','discover','-s',str(HERE/'hardware'),'-p','test_*.py','-v']
    h=subprocess.run(hardware,capture_output=True,text=True)
    (out/'hardware_tests.log').write_text(h.stdout+h.stderr)
    hardware_sources=list((HERE/'hardware').glob('*.py'))+list((HERE/'hardware').glob('*.hpp'))+list((HERE/'hardware').glob('*.cpp'))+list((HERE/'hardware').glob('*.sh'))
    for source in hardware_sources:(snap/source.name).write_bytes(source.read_bytes())
    manifest=json.loads((out/'source_sha256.json').read_text())
    manifest.update({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in hardware_sources})
    (out/'source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    test=[sys.executable,'-m','unittest','discover','-s',str(HERE),'-p','test_*.py','-v']
    r=subprocess.run(test,capture_output=True,text=True);(out/'tests.log').write_text(r.stdout+r.stderr)
    results=[{'case':'hardware_adapter_unit_tests','command':hardware,'returncode':h.returncode,'passed':h.returncode==0},{'case':'unit_tests','command':test,'returncode':r.returncode,'passed':r.returncode==0}]
    for name,expected in [('front','found'),('left','found'),('body','found'),('absent','not_found_in_observed_views'),('ambiguous','needs_clarification')]:
        cmd=[sys.executable,str(HERE/'run.py'),'--replay',str(HERE/('replay_'+name+'.json')),
             '--query','Tìm và chỉ cái ghế','--output-dir',str(out/name)]
        r=subprocess.run(cmd,capture_output=True,text=True);(out/(name+'.log')).write_text(r.stdout+r.stderr)
        path=out/name/'result.json';result=json.loads(path.read_text()) if path.exists() else {}
        results.append({'case':name,'command':cmd,'returncode':r.returncode,'status':result.get('status'),
                        'passed':r.returncode==0 and result.get('status')==expected})
    (out/'verification.json').write_text(json.dumps({'checks':results,'passed':all(x['passed'] for x in results),
        'synthetic_replay':True,'api_calls':0,'motor_commands_sent':False,'hardware_validated':False},ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output_dir':str(out),'passed':all(x['passed'] for x in results)},indent=2))
    return 0 if all(x['passed'] for x in results) else 1


if __name__=='__main__':raise SystemExit(main())
