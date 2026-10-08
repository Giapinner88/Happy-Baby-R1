#!/usr/bin/env python3
"""Backfill the four metadata fields required by the HB R1 mimic loader.

The policy graph and weights are kept unchanged. Values are copied from a
known-good policy with the same R1 training contract, then the output model is
checked and atomically replaced.
"""

from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

import onnx


REQUIRED_KEYS = (
    "default_joint_pos",
    "action_scale",
    "joint_stiffness",
    "joint_damping",
)


def _shape(value_info: onnx.ValueInfoProto) -> tuple[int | None, ...]:
    result: list[int | None] = []
    for dim in value_info.type.tensor_type.shape.dim:
        result.append(dim.dim_value if dim.HasField("dim_value") else None)
    return tuple(result)


def _metadata(model: onnx.ModelProto) -> dict[str, str]:
    return {entry.key: entry.value for entry in model.metadata_props}


def _check_r1_io(model: onnx.ModelProto, path: Path) -> None:
    if len(model.graph.input) != 1 or _shape(model.graph.input[0]) != (1, 129):
        raise ValueError(f"{path}: expected one input with shape [1, 129]")
    if len(model.graph.output) != 1 or _shape(model.graph.output[0]) != (1, 24):
        raise ValueError(f"{path}: expected one output with shape [1, 24]")


def _set_metadata(model: onnx.ModelProto, values: dict[str, str]) -> None:
    existing = {entry.key: entry for entry in model.metadata_props}
    for key, value in values.items():
        entry = existing.get(key)
        if entry is None:
            entry = model.metadata_props.add()
            entry.key = key
        entry.value = value


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path)
    parser.add_argument("reference", type=Path)
    args = parser.parse_args()

    target = onnx.load(str(args.target), load_external_data=True)
    reference = onnx.load(str(args.reference), load_external_data=True)
    _check_r1_io(target, args.target)
    _check_r1_io(reference, args.reference)

    reference_metadata = _metadata(reference)
    missing = [key for key in REQUIRED_KEYS if key not in reference_metadata]
    if missing:
        raise ValueError(f"{args.reference}: missing reference metadata {missing}")

    _set_metadata(
        target,
        {key: reference_metadata[key] for key in REQUIRED_KEYS},
    )
    onnx.checker.check_model(target)

    with tempfile.NamedTemporaryFile(
        dir=args.target.parent,
        prefix=f".{args.target.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        temporary = Path(handle.name)
    try:
        onnx.save(target, str(temporary))
        os.replace(temporary, args.target)
    finally:
        temporary.unlink(missing_ok=True)

    print(f"updated: {args.target}")
    for key in REQUIRED_KEYS:
        print(f"{key}: {reference_metadata[key]}")


if __name__ == "__main__":
    main()
