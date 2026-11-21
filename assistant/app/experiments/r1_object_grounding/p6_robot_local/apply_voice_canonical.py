"""Apply only the prepared voice files to canonical source, with backup/guards."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ALLOWED={'voice/hb_voice/vision_tools.py','voice/hb_voice/approach_tools.py','object_search/experiments/r1_object_grounding/p5_voice_search/voice_overlay/sitecustomize.py'}


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--canonical',type=Path,required=True);p.add_argument('--backup-dir',type=Path,required=True);a=p.parse_args()
    m=json.loads((a.bundle/'manifest.json').read_text())
    if set(m['output_sha256'])!=ALLOWED:raise ValueError('unexpected source changes')
    for name,sha in m['base_sha256'].items():
        if hashlib.sha256((a.canonical/name).read_bytes()).hexdigest()!=sha:raise ValueError('canonical changed: '+name)
    for name,sha in m['output_sha256'].items():
        if hashlib.sha256((a.bundle/name).read_bytes()).hexdigest()!=sha:raise ValueError('bundle changed')
        if name not in m['base_sha256'] and (a.canonical/name).exists():raise ValueError('new file already exists')
    a.backup_dir.mkdir(parents=True,exist_ok=False)
    for name in m['base_sha256']:
        target=a.backup_dir/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a.canonical/name,target)
    shutil.copy2(a.bundle/'manifest.json',a.backup_dir/'manifest.json')
    for name in m['output_sha256']:
        target=a.canonical/name;temporary=target.with_suffix(target.suffix+'.local_approach_tmp')
        shutil.copy2(a.bundle/name,temporary);temporary.replace(target)
    print('Canonical voice updated with guarded backup; no robot service changed')


if __name__=='__main__':main()
