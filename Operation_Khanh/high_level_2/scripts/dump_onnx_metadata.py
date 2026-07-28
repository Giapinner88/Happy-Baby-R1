#!/usr/bin/env python3
"""Inspect input/output shape and custom metadata of an ONNX file."""
import sys

import onnxruntime as ort

REQUIRED_KEYS = ["default_joint_pos", "action_scale", "joint_stiffness", "joint_damping"]


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <file.onnx>")
        sys.exit(1)

    path = sys.argv[1]
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])

    print(f"=== {path} ===")
    print("input :", [(i.name, i.shape) for i in sess.get_inputs()])
    print("output:", [(o.name, o.shape) for o in sess.get_outputs()])

    meta = sess.get_modelmeta().custom_metadata_map
    if not meta:
        print("\n[!] No custom metadata found.")
        print("    -> C++ runner will use hardcoded fallbacks in RobotSpec.hpp.")
        return

    print(f"\nCustom metadata ({len(meta)} keys):")
    for k, v in meta.items():
        shown = v if len(v) <= 200 else v[:200] + " ... (truncated)"
        print(f"  {k} = {shown}")

    missing = [k for k in REQUIRED_KEYS if k not in meta]
    if missing:
        print(f"\n[!] Missing keys: {missing}")
        print("    -> Missing values will use fallbacks in RobotSpec.hpp.")
    else:
        print("\n[OK] Found all 4 key metadata items.")


if __name__ == "__main__":
    main()
