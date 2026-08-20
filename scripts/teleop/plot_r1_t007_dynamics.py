#!/usr/bin/env python3
"""Plot fixed-base root, joint and per-link COM traces from a T007 run."""
from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1.kinematics import load_arm_chain
from teleop.r1.upper_body_kinematics import load_r1_a5_upper_body_model


def _summary(values: np.ndarray) -> dict[str, float | int] | None:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if len(finite) == 0:
        return None
    return {
        "count": int(len(finite)),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.quantile(finite, 0.95)),
        "max": float(np.max(finite)),
    }


def _endpoint_tracking(run_dir: Path) -> dict[str, dict[str, np.ndarray]]:
    """Recover requested and observed virtual-EE positions from raw run data."""

    config_path = run_dir / "experiment_config.json"
    targets_path = run_dir / "targets.json"
    if not config_path.is_file() or not targets_path.is_file():
        return {}
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if int(config.get("schema_version", 0)) < 2:
        # Schema-1 runs controlled the wrist origin and often did not snapshot
        # their mutable T007 config.  Applying the current tool frame to them
        # would create a quantitatively false tracking plot.
        return {}
    rows = json.loads(targets_path.read_text(encoding="utf-8"))
    coupled = "whole_upper_body" in config
    upper_model = None
    if coupled:
        declared = config["whole_upper_body"]
        urdf_path = ROOT / str(declared["urdf_path"])
        # Read the deviation from the run's own snapshot: a 14-DoF run records
        # 14-vector joint targets and the 13-DoF model would reject them.
        upper_model = load_r1_a5_upper_body_model(
            urdf_path, control_waist_roll=bool(declared.get("control_waist_roll", False))
        )
    result: dict[str, dict[str, np.ndarray]] = {}
    for side in ("left", "right"):
        time_s: list[float] = []
        requested: list[list[float]] = []
        ik_target: list[np.ndarray] = []
        commanded: list[np.ndarray] = []
        achieved: list[np.ndarray] = []
        solver_residual: list[float] = []
        solution_projected: list[bool] = []
        chain = None if coupled else load_arm_chain(side)
        for row in rows:
            application = dict(
                row.get("whole_upper_body", {}) if coupled else row.get("arm_head", {})
            )
            target = application.get(
                f"{side}_target_position_pelvis_m" if coupled else f"{side}_target_position_m"
            )
            observed = (
                row.get("post_physics_whole_upper_body_position_rad")
                if coupled
                else dict(row.get("post_physics_arm_position_rad", {})).get(side)
            )
            if not application.get("accepted") or target is None or observed is None:
                continue
            ik_joint_target = application.get(
                "ik_joint_target_rad" if coupled else f"{side}_ik_joint_target_rad"
            )
            limited_joint_target = application.get(
                "limited_joint_target_rad" if coupled else f"{side}_limited_joint_target_rad"
            )

            def endpoint(values: object) -> np.ndarray:
                q = np.asarray(values, dtype=float)
                if coupled:
                    assert upper_model is not None
                    state = upper_model.forward_kinematics(q)
                    transform = state.left_end_effector if side == "left" else state.right_end_effector
                    return transform[:3, 3]
                assert chain is not None
                return chain.endpoint_position(q)

            time_s.append(float(row["elapsed_s"]))
            requested.append([float(value) for value in target])
            ik_target.append(
                endpoint(ik_joint_target)
                if ik_joint_target is not None else np.full(3, np.nan)
            )
            commanded.append(
                endpoint(limited_joint_target)
                if limited_joint_target is not None
                else endpoint(observed)
            )
            achieved.append(endpoint(observed))
            residual = (
                dict(application.get("ik", {})).get(f"{side}_position_residual_m", np.nan)
                if coupled
                else application.get(f"{side}_position_residual_m", np.nan)
            )
            solver_residual.append(float(residual))
            solution_projected.append(
                application.get("solver_solution_kind") == "best_effort"
                if coupled
                else application.get(f"{side}_solution_kind") == "projected"
            )
        if time_s:
            result[side] = {
                "time_s": np.asarray(time_s),
                "requested_m": np.asarray(requested),
                "ik_target_m": np.asarray(ik_target),
                "commanded_m": np.asarray(commanded),
                "achieved_m": np.asarray(achieved),
                "solver_residual_m": np.asarray(solver_residual),
                "projected": np.asarray(solution_projected, dtype=bool),
            }
    return result

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="T007 run directory containing dynamics_trace.npz")
    parser.add_argument("--output-dir", type=Path, help="Defaults to <run_dir>/figures")
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve(); trace_path = run_dir / "dynamics_trace.npz"
    if not trace_path.is_file(): raise SystemExit(f"Missing T007 dynamics trace: {trace_path}")
    output = (args.output_dir or run_dir / "figures").expanduser().resolve()
    if output.exists(): raise SystemExit(f"Refusing to overwrite figure directory: {output}")
    output.mkdir(parents=True)
    import matplotlib.pyplot as plt
    trace = np.load(trace_path, allow_pickle=False); time_s = trace["elapsed_s"]
    root, root_vel, joint_q = trace["root_position_m"], trace["root_linear_velocity_mps"], trace["joint_position_rad"]
    joint_names = [str(name) for name in trace["joint_names"].tolist()]
    body_com, body_names = trace["body_com_position_m"], [str(name) for name in trace["body_names"].tolist()]
    if len(time_s) == 0: raise SystemExit("T007 dynamics trace contains no samples.")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    for axis, label in enumerate(("x", "y", "z")):
        axes[0].plot(time_s, root[:, axis] - root[0, axis], label=label); axes[1].plot(time_s, root_vel[:, axis], label=label)
    axes[0].set_ylabel("root displacement (m)"); axes[1].set_ylabel("root velocity (m/s)"); axes[0].legend(ncol=3); axes[1].legend(ncol=3)
    for index, name in enumerate(joint_names):
        if name == "waist_yaw_joint" or name.startswith(("left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "head_")):
            axes[2].plot(time_s, joint_q[:, index], label=name)
    axes[2].set(xlabel="wall time (s)", ylabel="joint position (rad)"); axes[2].legend(fontsize=6, ncol=2)
    fig.savefig(output / "fixed_base_root_and_joint_trace.png", dpi=160); plt.close(fig)
    fig, axis = plt.subplots(figsize=(10, 5), layout="constrained")
    for index, name in enumerate(body_names): axis.plot(time_s, body_com[:, index, 2], linewidth=0.8, label=name)
    axis.set(xlabel="wall time (s)", ylabel="link COM z (m)", title="Per-link COM height (world frame)"); axis.legend(fontsize=5, ncol=3)
    fig.savefig(output / "per_link_com_height.png", dpi=160); plt.close(fig)
    endpoint = _endpoint_tracking(run_dir)
    endpoint_summary: dict[str, object] = {
        "available": bool(endpoint),
        "definition": "Requested, pre-limiter IK (when recorded), rate-limited command, and post-physics observed virtual-EE positions. Error components are unsmoothed Euclidean norms.",
    }
    if endpoint:
        endpoint_fig, endpoint_axes = plt.subplots(4, 2, figsize=(13, 10), sharex="col", layout="constrained")
        for column, side in enumerate(("left", "right")):
            data = endpoint[side]
            total_error = np.linalg.norm(data["requested_m"] - data["achieved_m"], axis=1)
            target_to_command = np.linalg.norm(data["requested_m"] - data["commanded_m"], axis=1)
            command_to_observed = np.linalg.norm(data["commanded_m"] - data["achieved_m"], axis=1)
            for axis, label in enumerate(("x", "y", "z")):
                endpoint_axes[axis, column].plot(data["time_s"], data["requested_m"][:, axis], label="requested", linewidth=1.0)
                endpoint_axes[axis, column].plot(data["time_s"], data["commanded_m"][:, axis], label="rate-limited command", linewidth=0.9)
                endpoint_axes[axis, column].plot(data["time_s"], data["achieved_m"][:, axis], label="observed FK", linewidth=1.0)
                endpoint_axes[axis, column].set_ylabel(f"{label} (m)")
            endpoint_axes[0, column].set_title(f"{side.capitalize()} vendor virtual EE")
            endpoint_axes[0, column].legend(fontsize=7)
            endpoint_axes[3, column].plot(data["time_s"], total_error, color="tab:red", label="requested→observed")
            endpoint_axes[3, column].plot(data["time_s"], target_to_command, color="tab:orange", label="requested→command")
            endpoint_axes[3, column].plot(data["time_s"], command_to_observed, color="tab:purple", label="command→observed")
            finite_solver = np.isfinite(data["solver_residual_m"])
            if np.any(finite_solver):
                endpoint_axes[3, column].plot(
                    data["time_s"][finite_solver], data["solver_residual_m"][finite_solver],
                    color="tab:blue", linestyle="--", linewidth=0.8, label="IK residual",
                )
            projected = data["projected"]
            if np.any(projected):
                endpoint_axes[3, column].scatter(data["time_s"][projected], total_error[projected], s=5, color="black", label="projected target")
            endpoint_axes[3, column].set(xlabel="wall time (s)", ylabel="position error (m)")
            endpoint_axes[3, column].legend(fontsize=7)
            endpoint_summary[side] = {
                "sample_count": int(len(total_error)),
                "projected_sample_count": int(np.count_nonzero(projected)),
                "solver_position_residual_m": _summary(data["solver_residual_m"]),
                "requested_to_rate_limited_command_m": _summary(target_to_command),
                "rate_limited_command_to_observed_m": _summary(command_to_observed),
                "requested_to_observed_m": _summary(total_error),
            }
        endpoint_fig.savefig(output / "endpoint_target_vs_achieved.png", dpi=160)
        plt.close(endpoint_fig)
    summary = {"source_run": str(run_dir), "sample_count": int(len(time_s)), "root_max_displacement_m": float(np.max(np.linalg.norm(root - root[0], axis=1))), "root_max_linear_velocity_mps": float(np.max(np.linalg.norm(root_vel, axis=1))), "body_com_definition": "Per-link center of mass in world frame; not a mass-weighted whole-robot COM.", "endpoint_tracking": endpoint_summary, "command": "python3 scripts/teleop/plot_r1_t007_dynamics.py <run-dir>"}
    (output / "dynamics_plot_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(output); return 0

if __name__ == "__main__": raise SystemExit(main())
