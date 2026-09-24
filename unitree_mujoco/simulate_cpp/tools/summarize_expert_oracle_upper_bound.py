#!/usr/bin/env python3
"""Summarize isolated experts and optimistic fixed-expert upper bounds."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


EXPERTS = ("flat", "slope_up", "slope_down")
KEYS = ("scene", "angle_deg", "category", "direction", "vx", "vy", "yaw")


def load(directory: Path):
    with (directory / "summary.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {tuple(row[key] for key in KEYS): row for row in rows}


def role_expert(row: dict[str, str]) -> str | None:
    if row["angle_deg"] == "mixed":
        return None
    if row["angle_deg"] == "0" or row["category"] in ("STAND", "CROSS", "TURN"):
        return "flat"
    if row["direction"] == "up":
        return "slope_up"
    if row["direction"] == "down":
        return "slope_down"
    return "flat"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("flat", type=Path)
    parser.add_argument("slope_up", type=Path)
    parser.add_argument("slope_down", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    matrices = {name: load(getattr(args, name)) for name in EXPERTS}
    keys = list(matrices["flat"])
    if any(set(matrix) != set(keys) for matrix in matrices.values()):
        raise ValueError("expert matrices do not contain identical cases")

    expert_pass = {
        name: sum(row["control_result"] == "PASS" for row in matrix.values())
        for name, matrix in matrices.items()
    }
    posthoc_pass = 0
    role_pass = 0
    role_cases = 0
    disagreements = []
    unresolved = []
    for key in keys:
        outcomes = {
            name: matrices[name][key]["control_result"] for name in EXPERTS
        }
        if "PASS" in outcomes.values():
            posthoc_pass += 1
        else:
            unresolved.append({
                **{name: matrices["flat"][key][name] for name in KEYS},
                "outcomes": outcomes,
            })
        exemplar = matrices["flat"][key]
        role = role_expert(exemplar)
        if role is not None:
            role_cases += 1
            if outcomes[role] == "PASS":
                role_pass += 1
        if len(set(outcomes.values())) > 1:
            disagreements.append({
                **{name: exemplar[name] for name in KEYS},
                "role_expert": role,
                "outcomes": outcomes,
            })

    report = {
        "cases": len(keys),
        "expert_control_pass": expert_pass,
        "posthoc_best_fixed_expert_upper_bound": {
            "pass": posthoc_pass,
            "cases": len(keys),
            "note": "Non-causal union: counts a case if any fixed expert passes.",
        },
        "role_mapped_fixed_expert_oracle": {
            "pass": role_pass,
            "cases": role_cases,
            "excluded": len(keys) - role_cases,
            "note": "Flat for flat/stand/cross/turn; Up or Down for directional slope cases. Mixed traversal is excluded because it requires within-run switching.",
        },
        "cases_with_expert_outcome_disagreement": len(disagreements),
        "cases_failed_by_all_fixed_experts": unresolved,
        "disagreements": disagreements,
    }
    rendered = json.dumps(report, indent=2) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)


if __name__ == "__main__":
    main()
