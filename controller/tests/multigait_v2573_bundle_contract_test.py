"""Preflight acceptance of the multigait v2573/v2574 bundle, and its identity gate.

1. The shipped bundle, selected as walk_policy, passes `run_r1 --preflight`
   and prints the hardware-trial warning (the export says deployable=False).
2. Every identity or semantic key that MultiGaitV2573Controller pins is
   mutated one at a time; each mutation must stop preflight with
   "ONNX metadata mismatch". The contract name alone must never be enough.
3. A gait_period_s in policy.yaml is refused: this policy owns its clock.
"""
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

import onnx

root, binary = (Path(p).resolve() for p in sys.argv[1:3])
bundle_name = "multigait/v2574_armtight_c_iter24900"
model = onnx.load(root / f"policies/locomotion/{bundle_name}/policy.onnx")
metadata = {p.key: p.value for p in model.metadata_props}
assert metadata["deployable"] == "False", metadata["deployable"]

yaml_text = (root / f"policies/locomotion/{bundle_name}/policy.yaml").read_text()
assert re.search(r"^contract: r1_armfree0917_multigait_h4_v2573$", yaml_text, flags=re.M), yaml_text
assert not re.search(r"^gait_period_s:", yaml_text, flags=re.M), yaml_text

mismatch_cases = {
    "task_id": {"task_id": "Unitree-R1-ArmFree0917-MultiGait-Run25-v2575-Unknown-2048"},
    "source": {"source_sha256": "0" * 64},
    "config": {"config_sha256": "0" * 64},
    "contract": {"contract": "r1_armfree0917_multigait_h4_v257_control_recovery_run20_arms_inward_v1"},
    "smoke": {"smoke": "True"},
    "actor_dim": {"actor_dim": "335"},
    "timing_index": {"timing_action_index": "23"},
    "modes": {"mode_names": "STAND,WALK,W2S"},
    "phase_clock": {"phase_clock": "episode_time_continuous_legacy_0909"},
    "timing_rule": {"timing_rule": "walk_log_0.65_1.0"},
    "recovery": {"recovery_revision": "latched_stop_permission_imu_predict012_exit04_relax15_timeout6_v254"},
    "stop_gate": {"runtime_stop_gate": "FK_Jdq_gyro_support_persistence_land010_motion014_006s_or010_020s_not_absolute_no_slip_sensor"},
    "quiet": {"quiet_conditioning": "STAND_or_W2S_hold_no_recovery_small_scheduled_canonical_mode0_sub0_progress1"},
    "quiet_alias": {"actor_quiet_revision": "none"},
    "policy_dt": {"policy_dt": "0.01"},
    "joint_order": {"joint_names": ",".join(reversed(metadata["joint_names"].split(",")))},
}
cases = {"unmutated": {}, **mismatch_cases, "yaml_period": {}}

with tempfile.TemporaryDirectory(prefix="hb-multigait-v2573-") as tmp:
    project = Path(tmp)
    shutil.copytree(root / "config", project / "config")
    (project / "motions").symlink_to(root / "motions")
    (project / "policies").mkdir()
    for p in (root / "policies").iterdir():
        if p.name != "locomotion":
            (project / "policies" / p.name).symlink_to(p)
    loco = project / "policies/locomotion"
    loco.mkdir()
    for p in (root / "policies/locomotion").iterdir():
        (loco / p.name).symlink_to(p)
    bundle = loco / "multigait_v2573_test"
    bundle.mkdir()
    config = project / "config/locomotion.yaml"
    config.write_text(re.sub(r"^walk_policy:.*$", "walk_policy: multigait_v2573_test",
                             config.read_text(), flags=re.M))
    for label, changes in cases.items():
        current = onnx.ModelProto()
        current.CopyFrom(model)
        for prop in current.metadata_props:
            prop.value = changes.get(prop.key, prop.value)
        onnx.save(current, bundle / "policy.onnx")
        sha = hashlib.sha256((bundle / "policy.onnx").read_bytes()).hexdigest()
        extra = "gait_period_s: 0.6\n" if label == "yaml_period" else ""
        (bundle / "policy.yaml").write_text(
            f"contract: r1_armfree0917_multigait_h4_v2573\nsha256: {sha}\n{extra}")
        result = subprocess.run([str(binary), "--preflight"],
                                env={**os.environ, "HB_PROJECT_DIR": str(project)},
                                text=True, capture_output=True)
        output = result.stdout + result.stderr
        if label == "unmutated":
            assert result.returncode == 0, (label, output[-2500:])
            for phrase in ("[MultiGaitV2573] Unitree-R1-ArmFree0917-MultiGait-Run25-v2574-ArmTight-C-23k-2048",
                           "deployable=False", "gait_period=policy"):
                assert phrase in output, (label, phrase, output[-2500:])
        elif label == "yaml_period":
            assert result.returncode != 0, (label, output[-1500:])
            assert "has no fixed gait period" in output, (label, output[-1500:])
        else:
            assert result.returncode != 0, (label, output[-1500:])
            assert "ONNX metadata mismatch" in output, (label, output[-1500:])
print(f"MULTIGAIT_V2573_BUNDLE_CONTRACT_OK: {len(cases)} cases")
