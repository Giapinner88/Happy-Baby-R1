"""Preflight T1 metadata acceptance and rejection against the actual ONNX."""
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
model = onnx.load(root / "policies/locomotion/t1_settle05_iter19999/policy.onnx")
metadata = {p.key: p.value for p in model.metadata_props}
reversed_joints = ",".join(reversed(metadata["joint_names"].split(",")))
# gait_task_variant la provenance, khong con la cong chan: mot ban train lai voi
# ten bien the moi phai nap duoc, nhung van phai qua day du hop dong quan sat.
# Cac case bi bo: "envs" (training_num_envs) va "budget"
# (training_target_iterations). Hai khoa do mo ta cach train, khong anh huong
# inference; cho cua chung la packaging.json chu khong phai cong runtime.
DELETE = object()  # sentinel: bo han khoa khoi metadata thay vi doi gia tri
cases = {
    "valid": ({}, True),
    "new_variant": ({"gait_task_variant": "retrain_unknown_20261106"}, True),
    "new_variant_joint_order": (
        {"gait_task_variant": "retrain_unknown_20261106", "joint_names": reversed_joints},
        False),
    "period": ({"gait_period_s": "0.6"}, False),
    "phase": ({"gait_phase_contract": "unknown"}, False),
    "clock": ({"training_phase_clock": "stand_reset"}, False),
    "mask": ({"training_phase_zero_mask": "fsm_stand"}, False),
    "first_tick": ({"gait_phase_tick_alignment": "increment_first"}, False),
    "joint_count": ({"gait_stability_joint_count": "12"}, False),
    "joint_order": ({"joint_names": reversed_joints}, False),
    "rms_order": ({"gait_stability_joint_names": reversed_joints}, False),
    "predicate": ({"gait_stability_predicate": "filtered_imu_and_legs12_joint_rms_v1"}, False),
    "fsm": ({"gait_fsm_contract": "flat_plus_gait_fsm_v1"}, False),
    "settle": ({"gait_t_settle_s": "1.0"}, False),
    # Xoa khoa la duong vong khac han doi gia tri: truoc day bo qua ca khoi
    # phase roi chay bang episode clock trong im lang. Bundle khai
    # gait_phase_source nen bay gio lech la chet.
    "phase_deleted": ({"gait_phase_contract": DELETE}, False),
    "phase_block_deleted": ({"gait_phase_contract": DELETE,
                             "training_phase_clock": DELETE,
                             "training_phase_zero_mask": DELETE,
                             "gait_phase_tick_alignment": DELETE,
                             "gait_t_settle_s": DELETE}, False),
    "clock_deleted": ({"training_phase_clock": DELETE}, False),
    "joint_names_deleted": ({"joint_names": DELETE}, False),
    # KHE HO CON LAI, co y de lai va ghi ro: xoa han gait_stability_joint_names
    # hoac gait_stability_joint_count thi khong bi chan, vi bundle
    # flat_plus_gait_0917_armfree that su khong export hai khoa nay, nen "vang
    # mat" la hop le. Khac voi khoi phase o tren, hai khoa nay chi anh huong
    # NGUONG CHUYEN TRANG THAI cua FSM, khong doi obs dua vao mang, nen hau qua
    # la thoi diem vao STAND lech chu khong phai policy chay sai dau vao.
    "rms_order_deleted": ({"gait_stability_joint_names": DELETE}, True),
    "joint_count_deleted": ({"gait_stability_joint_count": DELETE}, True),
}
with tempfile.TemporaryDirectory(prefix="hb-t1-contract-") as tmp:
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
    bundle = loco / "t1_test"
    bundle.mkdir()
    config = project / "config/locomotion.yaml"
    config.write_text(re.sub(r"^walk_policy:.*$", "walk_policy: t1_test", config.read_text(), flags=re.M))
    for label, (changes, accepted) in cases.items():
        current = onnx.ModelProto()
        current.CopyFrom(model)
        kept = [p for p in current.metadata_props if changes.get(p.key) is not DELETE]
        for prop in kept:
            if prop.key in changes:
                prop.value = changes[prop.key]
        del current.metadata_props[:]
        current.metadata_props.extend(kept)
        onnx.save(current, bundle / "policy.onnx")
        sha = hashlib.sha256((bundle / "policy.onnx").read_bytes()).hexdigest()
        # Bundle khai dung chu ky/dwell cua T1. Case "period"/"settle" lam lech
        # metadata ONNX so voi khai bao nay -> phai bi tu choi.
        (bundle / "policy.yaml").write_text(
            f"contract: flat_plus_gait_h4_v1\nsha256: {sha}\n"
            f"gait_period_s: {metadata['gait_period_s']}\n"
            f"gait_settle_dwell_s: {metadata['gait_t_settle_s']}\n"
            # Bundle khai nguon term phase; case nao xoa/doi khoa phase trong
            # ONNX se lech voi khai bao nay va phai bi tu choi.
            "gait_phase_source: command_gated\n")
        result = subprocess.run([str(binary), "--preflight"],
                                env={**os.environ, "HB_PROJECT_DIR": str(project)},
                                text=True, capture_output=True)
        assert (result.returncode == 0) == accepted, (label, result.stdout[-1500:], result.stderr)
        if not accepted:
            assert "metadata mismatch" in result.stdout + result.stderr, label
print(f"T1_BUNDLE_CONTRACT_OK: {len(cases)} cases")
