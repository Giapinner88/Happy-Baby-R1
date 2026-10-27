"""Actual preflight: native phase/PD, old artifacts and rejection of bad metadata."""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import numpy as np
import onnx

root = Path(sys.argv[1]).resolve()
binary = Path(sys.argv[2]).resolve()
names = (
    'armfree/nomirror_d20',
    'armfree/nomirror_d25',
    'armfree/speed20_oldaction_d20',
    'armfree/speed20_nominal_d20',
)
for name in names:
    source = root / 'policies/locomotion' / name
    model = onnx.load(source / 'policy.onnx')
    meta = {x.key: x.value for x in model.metadata_props}
    assert len(meta) == len(model.metadata_props)
    arrays = json.loads((source / 'packaging.json').read_text())['exact_resolved_arrays']
    damping = np.asarray(arrays['joint_damping'])
    scale = np.asarray(arrays['action_scale'])
    for i in range(12):
        assert damping[i] == (2.5 if 'd25' in name and i%6 in (3,4,5) else 2.)
        expected = (.15 if 'nominal' in name else .1575) if i%6 == 3 else (.3125 if 'nominal' in name else .328125) if i%6 in (4,5) else .15
        assert abs(scale[i] - expected) < 1e-7
    assert meta['deployable'] == 'false' and meta['gait_t_settle_s'] == '0.5'
    assert meta['gait_phase_contract'] == 'episode_clock_command_gated_v1'
    with tempfile.TemporaryDirectory(prefix='hb-native-phase-') as tmp:
        target = Path(tmp)
        shutil.copytree(root / 'config', target / 'config')
        config = target / 'config/locomotion.yaml'
        original_config = config.read_text()
        for folder in ('motions', 'assets'):
            if (root / folder).exists():
                (target / folder).symlink_to(root / folder, target_is_directory=True)
        (target / 'policies').mkdir()
        for p in (root / 'policies').iterdir():
            if p.name != 'locomotion':
                (target / 'policies' / p.name).symlink_to(p, target_is_directory=p.is_dir())
        loco = target / 'policies/locomotion';loco.mkdir()
        for p in (root / 'policies/locomotion').iterdir():
            if p.name != 'armfree':
                (loco / p.name).symlink_to(p, target_is_directory=p.is_dir())
        group = loco / 'armfree';group.mkdir()
        for p in (root / 'policies/locomotion/armfree').iterdir():
            if p.name != source.name:
                (group / p.name).symlink_to(p, target_is_directory=p.is_dir())
        bundle = loco / name;bundle.mkdir()
        cases = [
            ('valid', {}, True),
            ('unknown_phase', {'gait_phase_contract': 'unregistered_phase'}, False),
            ('wrong_clock', {'training_phase_clock': 'stand_reset_clock'}, False),
            ('wrong_zero_mask', {'training_phase_zero_mask': 'fsm_stand'}, False),
            ('wrong_first_tick', {'gait_phase_tick_alignment': 'increment_before_observation'}, False),
            ('wrong_leg_order', {'gait_stability_joint_names': ','.join(reversed(meta['gait_stability_joint_names'].split(',')))}, False),
            ('wrong_predicate', {'gait_stability_predicate': 'filtered_imu_ang_vel_norm_and_joint_vel_rms_v2'}, False),
            ('wrong_fsm', {'gait_fsm_contract': 'unregistered_fsm'}, False),
            ('wrong_joint_order', {'joint_names': ','.join(reversed(meta['joint_names'].split(',')))}, False),
            ('wrong_dwell', {'gait_t_settle_s': '1.0'}, False),
        ]
        for label, updates, accepted in cases:
            current = onnx.ModelProto();current.CopyFrom(model)
            for prop in current.metadata_props:
                if prop.key in updates:
                    prop.value = updates[prop.key]
            onnx.save(current, bundle / 'policy.onnx')
            sha = hashlib.sha256((bundle / 'policy.onnx').read_bytes()).hexdigest()
            (bundle / 'policy.yaml').write_text(f'contract: flat_plus_gait_h4_v1\nsha256: {sha}\nteleop_h4: true\n')
            text = re.sub(r'^walk_policy:.*$', 'walk_policy: ' + name, original_config, flags=re.MULTILINE)
            text = re.sub(r'^teleop_policy:.*$', 'teleop_policy: ' + name, text, flags=re.MULTILINE)
            config.write_text(text)
            result = subprocess.run([str(binary), '--preflight'], env={**os.environ, 'HB_PROJECT_DIR': str(target)},
                                    text=True, capture_output=True)
            assert (result.returncode == 0) == accepted, (name, label, result.stdout[-3000:], result.stderr)
            if not accepted:
                assert 'metadata mismatch' in result.stdout + result.stderr, (name, label, result.stdout[-2000:])
        # This checkout enables H4 by controller contract. Keep its newer
        # mixed-policy behavior, including default-pose compatibility checks.
        onnx.save(model, bundle / 'policy.onnx')
        sha = hashlib.sha256((bundle / 'policy.onnx').read_bytes()).hexdigest()
        (bundle / 'policy.yaml').write_text(f'contract: flat_plus_gait_h4_v1\nsha256: {sha}\nteleop_h4: true\n')
        config.write_text(re.sub(r'^walk_policy:.*$', 'walk_policy: ' + name, original_config, flags=re.MULTILINE))
        result = subprocess.run([str(binary), '--preflight'], env={**os.environ, 'HB_PROJECT_DIR': str(target)},
                                text=True, capture_output=True)
        assert result.returncode == 0, (name, 'current_teleop_route', result.stdout[-3000:], result.stderr)
        assert 'policies/locomotion/u2a_seed42/policy.onnx' in result.stdout
        # A stale bundle hash must still reject before model loading.
        (bundle / 'policy.yaml').write_text('contract: flat_plus_gait_h4_v1\nsha256: ' + '0'*64 + '\n')
        result = subprocess.run([str(binary), '--preflight'], env={**os.environ, 'HB_PROJECT_DIR': str(target)},
                                text=True, capture_output=True)
        assert result.returncode != 0 and 'SHA-256' in result.stdout + result.stderr
    print(name, 'PASS: exact PD/scales, grouped preflight, bad metadata/hash rejected, mixed H4 route retained')
