"""Exercise the actual boot guards without services, certificates or motors."""
import configparser
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def unit(name):
    config = configparser.ConfigParser(interpolation=None)
    config.read(ROOT / 'systemd' / name)
    return config


def test_boot_dependencies_reach_runtime_restart_policy():
    runtime = unit('hb_teleop_runtime.service.in')
    assert 'hb_high_level.service' in runtime['Unit']['Wants'].split()
    assert 'hb_high_level.service' not in runtime['Unit']['Requires'].split()
    assert runtime['Service']['Restart'] == 'on-failure'
    assert runtime['Unit']['StartLimitIntervalSec'] == '0'
    cert = unit('hb_teleop_cert.service.in')
    assert cert['Service']['Restart'] == 'on-failure'
    assert cert['Unit']['StartLimitIntervalSec'] == '0'
    assert int(cert['Service']['TimeoutStartSec']) > 60


@pytest.mark.parametrize('inactive', ['hb_high_level.service', 'hb_teleop_cert.service', ''])
def test_preflight_refuses_missing_dependencies_before_runtime_load(inactive):
    source = (ROOT / 'scripts/robot_runtime_preflight.sh').read_text()
    branch = source[source.index('for dependency in '):source.index('\n[[ "$(uname -m)"')]
    setup = '''set -euo pipefail
fail=0
ok() { :; }
bad() { echo "$*"; fail=1; }
systemctl() { [[ "$3" != "$INACTIVE" ]]; }
'''
    result = subprocess.run(['bash', '-c', setup + branch], text=True, capture_output=True,
                            env={**os.environ, 'INACTIVE': inactive})
    assert result.returncode == (1 if inactive else 0), result.stderr
    if inactive:
        assert inactive in result.stdout


@pytest.mark.parametrize('missing,expected', [(False, 0), (True, 1)])
def test_certificate_wait_handles_absent_wifi_then_ipv4(tmp_path, missing, expected):
    source = (ROOT / 'scripts/refresh_robot_cert.sh').read_text()
    branch = source[source.index('wait_for_host_ipv4()'):source.index('\ninstall -d')]
    mock = tmp_path / 'ip'
    mock.write_text('''#!/bin/bash
count=0
[[ ! -e "$COUNTER" ]] || read -r count < "$COUNTER"
count=$((count+1)); echo "$count" > "$COUNTER"
if [[ "$MISSING" == 1 || "$count" -lt 3 ]]; then
    echo 'Device "wlan0" does not exist.' >&2; exit 1
fi
echo '3: wlan0 inet 10.42.0.114/24 scope global wlan0'
''')
    mock.chmod(0o755)
    result = subprocess.run(['bash', '-c', 'set -euo pipefail; HOST_INTERFACE=wlan0; sleep() { :; };\n' +
                             branch + '\nprintf "%s" "$IP"'], text=True, capture_output=True,
                            env={**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}',
                                 'COUNTER': str(tmp_path / 'attempts'), 'MISSING': str(int(missing)),
                                 'HB_TELEOP_RUNTIME_NETWORK_WAIT_S': '0' if missing else '5'})
    assert result.returncode == expected, result.stderr
    if missing:
        assert 'service will retry' in result.stderr
    else:
        assert result.stdout == '10.42.0.114'
        assert (tmp_path / 'attempts').read_text().strip() == '3'
