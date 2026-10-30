#!/usr/bin/env python3
"""Read-only P6 bundle/runtime checks; no camera or motor command."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys


ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_bundle(root, allow_staging=False):
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('schema') != 'r1_p6_onboard_bundle.v1':
        raise ValueError('unexpected P6 bundle schema')
    for name, expected in manifest['files_sha256'].items():
        path = root/name
        if not path.is_file() or sha(path) != expected:
            raise ValueError('P6 bundle file missing/changed: ' + name)
    if not allow_staging and manifest['install_root'] != str(root):
        raise ValueError('bundle installed at unexpected path')
    config = json.loads((root/'experiments/r1_object_grounding/p2_bbox_baseline/config.json').read_text())
    calibration = root/config['calibration']
    if not calibration.is_file() or sha(calibration) != manifest['calibration_sha256']:
        raise ValueError('camera calibration missing or changed')
    import cv2
    import numpy as np
    from PIL import Image  # noqa: F401
    with np.load(calibration, allow_pickle=False) as data:
        wh = tuple(map(int, data['image_size_wh']))
        model = str(data['camera_model'])
        if len(wh) != 2 or min(wh) <= 0 or model not in ('fisheye', 'pinhole'):
            raise ValueError('invalid camera calibration metadata')
    return {'bundle_files': len(manifest['files_sha256']), 'image_size_wh': wh,
            'camera_model': model, 'opencv': cv2.__version__,
            'numpy': np.__version__}


def verify_yaw_integration(runtime_root):
    application_header = runtime_root/'controller/src/app/Application.hpp'
    policy_source = runtime_root/'controller/src/app/ApplicationPolicy.cpp'
    yaw_header = runtime_root/'controller/src/input/SearchYawReceiver.hpp'
    if not all(path.is_file() for path in (yaw_header, application_header, policy_source)):
        raise ValueError('canonical HB controller lacks the search yaw receiver')
    if ('#include "../input/SearchYawReceiver.hpp"' not in application_header.read_text()
            or 'search_yaw_.Apply' not in policy_source.read_text()):
        raise ValueError('canonical HB controller has not integrated search yaw')


def verify_runtime(root, runtime_root, agent_port):
    if not os.environ.get('OPENAI_API_KEY', '').strip():
        raise ValueError('OPENAI_API_KEY missing from service environment')
    from experiments.r1_object_grounding.p6_onboard.local_bridge import call_agent
    from experiments.r1_target_control.control import runtime_info
    free_gb = shutil.disk_usage(root).free / 1_000_000_000
    if free_gb < 1.0:
        raise RuntimeError('less than 1 GB free for P6 image and run evidence')
    config = json.loads((root/'experiments/r1_target_control/config.json').read_text())
    runtime = runtime_info(runtime_root, config)
    verify_yaw_integration(runtime_root)
    expected_owner = (runtime_root/'controller/build/run_r1').resolve()
    owners = []
    senders = []
    conflicts = ('run_r1_vendor_targets.py', 'run_r1_vendor_ik_stream.py',
                 'run_r1_policy_rollout.py', 'run_r1_upstream_ik_stream.py')
    for item in Path('/proc').iterdir():
        if not item.name.isdigit():
            continue
        try:
            command = item.joinpath('cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            argv0 = command.split(' ', 1)[0]
            if Path(argv0).name == 'run_r1':
                owners.append((item.name, (item/'exe').resolve()))
            if any(name in command for name in conflicts):
                senders.append(item.name)
        except (OSError, ValueError):
            continue
    if len(owners) != 1 or owners[0][1] != expected_owner:
        raise RuntimeError('P6 requires the sole owner from the configured HB runtime')
    if senders:
        raise RuntimeError('competing Quest/policy sender PIDs: ' + ','.join(senders))
    state = call_agent({'method': 'state'}, port=agent_port, timeout_s=5)
    if not isinstance(state, dict) or 'body_heading_deg' not in state:
        raise ValueError('agent did not return fresh state')
    return {'owner_pid': owners[0][0], 'agent_state': 'fresh',
            'disk_free_gb': round(free_gb, 2),
            'policy_sha256': runtime['policy_sha256']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-dir', type=Path, default=ROOT)
    parser.add_argument('--runtime-root', type=Path, default=Path('/home/unitree/HB'))
    parser.add_argument('--agent-port', type=int, default=5572)
    parser.add_argument('--bundle-only', action='store_true')
    args = parser.parse_args()
    root = args.bundle_dir.resolve()
    sys.path.insert(0, str(root))
    report = {'bundle': verify_bundle(root, allow_staging=args.bundle_only)}
    if not args.bundle_only:
        report['runtime'] = verify_runtime(root, args.runtime_root, args.agent_port)
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
