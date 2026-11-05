"""Preflight acceptance of the multigait V2 bundle: it must be REFUSED, precisely.

Two things are pinned here.

1. The refusal itself. `r1_armfree0917_multigait_h4_v2` is registered but not
   implemented, so selecting it must stop startup with a message that names
   every decision a human still owes. A future change that quietly makes this
   contract "work" has to update this test, which is the point: the 25th action
   and the 8-mode FSM are not something to switch on by accident.

2. The identity gate in front of the refusal. Before refusing,
   MultiGaitH4V2Controller checks the metadata keys that pin which artifact
   this is. Each mutation below must produce "ONNX metadata mismatch" instead
   of the refusal: without that, the contract name alone would be enough to
   route any 355/25 file here.
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
bundle_name = "multigait_v2_run25_iter8400"
model = onnx.load(root / f"policies/locomotion/{bundle_name}/policy.onnx")
metadata = {p.key: p.value for p in model.metadata_props}

# Bundle khong khai gait_period_s: chu ky cua contract nay do policy phat ra
# moi buoc (action index 24). Preflight van phai di toi loi tu choi, khong
# duoc chet som o "bundle thieu gait_period_s".
yaml_text = (root / f"policies/locomotion/{bundle_name}/policy.yaml").read_text()
assert not re.search(r"^gait_period_s:", yaml_text, flags=re.M), yaml_text

mismatch_cases = {
    "contract": {"contract": "r1_armfree0917_multigait_h4_v3"},
    "actor_dim": {"actor_dim": "335"},
    "action_dim": {"action_dim": "24"},
    "prefix": {"normalization_prefix": "355"},
    "timing_index": {"timing_action_index": "23"},
    "modes": {"mode_names": "STAND,WALK,W2S"},
    "phase_clock": {"phase_clock": "episode_time_continuous_legacy_0909"},
    "phase_mask": {"phase_zero_mask": "command_3norm_lt_0.1"},
    "support": {"scheduled_support_semantics": "measured_contact"},
    "policy_dt": {"policy_dt": "0.01"},
    "joint_order": {"joint_names": ",".join(reversed(metadata["joint_names"].split(",")))},
}
# Khong mutate: artifact nguyen ban phai bi tu choi bang loi thiet ke, khong
# phai bang mismatch.
cases = {"unmutated": {}, **mismatch_cases}

required_phrases = (
    "MultiGaitH4V2 refuses to run",
    "deployable=False",
    "iteration=8400",
    "action index 24",
    "STAND,WALK,RUN,S2W,W2S,W2R,R2W,TURN",
    "30 khoa metadata",
)

with tempfile.TemporaryDirectory(prefix="hb-multigait-v2-") as tmp:
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
    bundle = loco / "multigait_v2_test"
    bundle.mkdir()
    config = project / "config/locomotion.yaml"
    config.write_text(re.sub(r"^walk_policy:.*$", "walk_policy: multigait_v2_test",
                             config.read_text(), flags=re.M))
    for label, changes in cases.items():
        current = onnx.ModelProto()
        current.CopyFrom(model)
        for prop in current.metadata_props:
            prop.value = changes.get(prop.key, prop.value)
        onnx.save(current, bundle / "policy.onnx")
        sha = hashlib.sha256((bundle / "policy.onnx").read_bytes()).hexdigest()
        (bundle / "policy.yaml").write_text(
            f"contract: r1_armfree0917_multigait_h4_v2\nsha256: {sha}\n")
        result = subprocess.run([str(binary), "--preflight"],
                                env={**os.environ, "HB_PROJECT_DIR": str(project)},
                                text=True, capture_output=True)
        output = result.stdout + result.stderr
        # Khong bao gio duoc khoi dong duoc, du mutate hay khong.
        assert result.returncode != 0, (label, output[-1500:])
        if label == "unmutated":
            for phrase in required_phrases:
                assert phrase in output, (label, phrase, output[-2000:])
            assert "metadata mismatch" not in output, (label, output[-2000:])
        else:
            assert "ONNX metadata mismatch" in output, (label, output[-1500:])
            assert "refuses to run" not in output, (label, output[-1500:])
print(f"MULTIGAIT_V2_BUNDLE_CONTRACT_OK: {len(cases)} cases")
