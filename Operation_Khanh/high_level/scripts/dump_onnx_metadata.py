#!/usr/bin/env python3
"""Soi input/output shape + toàn bộ custom metadata của 1 file ONNX.

Dùng khi đổi policy mới: biết ngay model có đủ metadata (default_joint_pos,
action_scale, joint_stiffness, joint_damping...) hay chương trình C++ sẽ
phải dùng fallback cứng trong src/config/RobotSpec.hpp.

    python3 scripts/dump_onnx_metadata.py policies/flat/policy_r1_flat_2.onnx
    python3 scripts/dump_onnx_metadata.py policies/dance/policy_r1_dance.onnx
"""
import sys

import onnxruntime as ort

REQUIRED_KEYS = ["default_joint_pos", "action_scale", "joint_stiffness", "joint_damping"]


def main():
    if len(sys.argv) != 2:
        print(f"Dùng: {sys.argv[0]} <file.onnx>")
        sys.exit(1)

    path = sys.argv[1]
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])

    print(f"=== {path} ===")
    print("input :", [(i.name, i.shape) for i in sess.get_inputs()])
    print("output:", [(o.name, o.shape) for o in sess.get_outputs()])

    meta = sess.get_modelmeta().custom_metadata_map
    if not meta:
        print("\n[!] KHÔNG có custom metadata.")
        print("    -> Chương trình C++ sẽ dùng fallback trong RobotSpec.hpp")
        print("       cho toàn bộ default_joint_pos/action_scale/gains.")
        return

    print(f"\nCustom metadata ({len(meta)} key):")
    for k, v in meta.items():
        shown = v if len(v) <= 200 else v[:200] + " ...(cắt bớt)"
        print(f"  {k} = {shown}")

    missing = [k for k in REQUIRED_KEYS if k not in meta]
    if missing:
        print(f"\n[!] Thiếu key: {missing}")
        print("    -> Các giá trị này sẽ lấy fallback trong RobotSpec.hpp.")
    else:
        print("\n[OK] Đủ 4 key quan trọng — C++ sẽ tự nạp, không cần sửa RobotSpec.hpp.")


if __name__ == "__main__":
    main()
