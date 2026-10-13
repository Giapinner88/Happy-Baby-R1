"""Deploy authentication regression checks without changing real sudo policy."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
AUTH = ROOT / 'integration/scripts/deploy_auth.sh'


def executable(path, content):
    path.write_text(content)
    path.chmod(0o755)


def test_first_sudo_timestamp_migration_completes_in_one_invocation(tmp_path):
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    state = tmp_path / 'state.json'
    state.write_text(json.dumps(dict(policy='tty', ticket=None, prompts=0, installs=0)))
    executable(bindir / 'ssh', f'''#!{sys.executable}
import os, subprocess, sys
if '-O' in sys.argv: sys.exit(0)
command=sys.argv[-1]
sys.exit(subprocess.call(['/bin/bash','-c',command],env=os.environ))
''')
    executable(bindir / 'sudo', f'''#!{sys.executable}
import json,os,subprocess,sys
from pathlib import Path
p=Path(os.environ['AUTH_STATE']); state=json.loads(p.read_text()); args=sys.argv[1:]
if os.environ.get('AUTH_ROOT')=='1': raise SystemExit('unexpected nested sudo')
if args==['-n','-v']:
 sys.exit(0 if state['ticket']==state['policy'] else 1)
state['prompts']+=1;state['ticket']=state['policy'];p.write_text(json.dumps(state))
if args==['-v']:sys.exit(0)
assert args==['bash','-s'], args
sys.exit(subprocess.call(['/bin/bash','-s'],env={{**os.environ,'AUTH_ROOT':'1','SUDO_USER':'unitree'}}))
''')
    executable(bindir / 'visudo', f'''#!{sys.executable}
import os,sys
assert os.environ.get('AUTH_ROOT')=='1'
if '-cf' in sys.argv:
 text=open(sys.argv[-1]).read()
 assert 'Defaults:unitree timestamp_type=global' in text
 assert 'NOPASSWD' not in text
''')
    executable(bindir / 'install', f'''#!{sys.executable}
import json,os,sys
from pathlib import Path
assert os.environ.get('AUTH_ROOT')=='1'
if '-d' not in sys.argv:
 p=Path(os.environ['AUTH_STATE']);s=json.loads(p.read_text());s['policy']='global';s['installs']+=1;p.write_text(json.dumps(s))
''')
    script = f'''
set -euo pipefail
source '{AUTH}'
hb_deploy_auth_open_ssh() {{ return 0; }}
hb_deploy_auth_start_keepalive() {{ return 0; }}
hb_deploy_auth_enable_sudo unitree@test /tmp/mock 5
[[ "$HB_DEPLOY_SUDO_READY" == 1 ]]
'''
    result = subprocess.run(['bash', '-c', script], env={**os.environ,
        'PATH': str(bindir) + ':' + os.environ['PATH'], 'AUTH_STATE': str(state)},
        text=True, capture_output=True, timeout=8)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'cache sudo mới' in result.stdout
    current = json.loads(state.read_text())
    assert current == dict(policy='global', ticket='global', prompts=2, installs=1)
    # Once configured, a normal deploy needs only the ordinary sudo authentication.
    current['ticket'] = None
    state.write_text(json.dumps(current))
    again = subprocess.run(['bash', '-c', script], env={**os.environ,
        'PATH': str(bindir) + ':' + os.environ['PATH'], 'AUTH_STATE': str(state)},
        text=True, capture_output=True, timeout=8)
    assert again.returncode == 0, again.stdout + again.stderr
    assert json.loads(state.read_text())['prompts'] == 3


def test_master_opens_a_real_session_and_cleanup_stops_sleep(tmp_path):
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    record = tmp_path / 'ssh.jsonl'
    executable(bindir / 'ssh', f'''#!{sys.executable}
import json,os,sys
with open(os.environ['AUTH_CALLS'],'a') as stream: stream.write(json.dumps(sys.argv[1:])+'\\n')
if '-O' in sys.argv and 'check' in sys.argv: sys.exit(1)
''')
    result = subprocess.run(['bash', '-c', f'''
set -euo pipefail
source '{AUTH}'
hb_deploy_auth_open_ssh unitree@test /tmp/mock 5
hb_deploy_auth_start_keepalive unitree@test /tmp/mock 5
sleep 0.1
hb_deploy_auth_cleanup /tmp/mock unitree@test
'''], env={**os.environ, 'PATH': str(bindir) + ':' + os.environ['PATH'],
           'AUTH_CALLS': str(record)}, capture_output=True, text=True, timeout=4)
    assert result.returncode == 0, result.stdout + result.stderr
    calls = [json.loads(line) for line in record.read_text().splitlines()]
    opened = next(args for args in calls if 'ControlMaster=yes' in args)
    assert '-f' in opened and opened[-1] == 'true'
    assert '-MNf' not in opened
