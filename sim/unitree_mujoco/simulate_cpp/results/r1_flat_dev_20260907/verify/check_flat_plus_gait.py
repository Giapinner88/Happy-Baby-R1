#!/usr/bin/env python3
"""Fail-closed checker for the `flat_plus_gait_h4_v1` artifacts.

Same contract as ``check_flat_plus.py`` and the same discipline: report what is
genuinely missing rather than skipping quietly, and exit non-zero only for real
failures unless ``--strict``.

What it checks that the H4 checker cannot:

  1. the actor group is still 332 and the gait group is 3 -- i.e. the gait ID
     was appended once, not stacked into a 344D actor;
  2. the exported ONNX normalizer is 332 wide. A 335-wide one means the stock
     MLP export path ran and the graph normalizes the gait one-hot, which
     changes every action and raises nothing;
  3. the FSM produces a valid one-hot, and reaches every declared mode when
     driven through a stop sequence. A mode that the FSM can never emit is a
     column the policy will never learn to use.

    python scripts/check_flat_plus_gait.py
    python scripts/check_flat_plus_gait.py --onnx path/to/policy.onnx --strict
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from src.tasks.velocity.config.r1 import gait_contract as gc
from src.tasks.velocity.config.r1 import history_contract as hc

REPO = Path(__file__).resolve().parents[1]

RED, GREEN, YELLOW, RESET = "\033[31m", "\033[32m", "\033[33m", "\033[0m"


class Report:
  def __init__(self) -> None:
    self.failed = 0
    self.skipped = 0

  def ok(self, what: str, detail: str = "") -> None:
    print(f"{GREEN}PASS{RESET} {what}" + (f"  ({detail})" if detail else ""))

  def fail(self, what: str, detail: str) -> None:
    print(f"{RED}FAIL{RESET} {what}\n       {detail}")
    self.failed += 1

  def skip(self, what: str, why: str) -> None:
    print(f"{YELLOW}SKIP{RESET} {what}\n       {why}")
    self.skipped += 1


def check_task(report: Report) -> None:
  try:
    import torch

    import mjlab.tasks  # noqa: F401
    import src.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.registry import list_tasks, load_env_cfg, load_rl_cfg
  except ImportError as error:
    report.skip("task registry / live dimensions", f"mjlab not importable: {error}")
    return

  if gc.TASK_ID not in list_tasks():
    report.fail("task registry", f"{gc.TASK_ID} is not registered")
    return
  report.ok("task registry", gc.TASK_ID)

  rl_cfg = load_rl_cfg(gc.TASK_ID)
  if tuple(rl_cfg.obs_groups["actor"]) != gc.ACTOR_OBS_GROUPS:
    report.fail(
      "actor obs group order",
      f"expected {gc.ACTOR_OBS_GROUPS}, got {tuple(rl_cfg.obs_groups['actor'])}; "
      "group order is the flat tensor layout",
    )
  elif not rl_cfg.actor.class_name.endswith("GaitConditionedMLPModel"):
    report.fail(
      "actor model class",
      f"{rl_cfg.actor.class_name} normalizes the gait one-hot with the prefix",
    )
  else:
    report.ok("obs group order and model class")

  cfg = load_env_cfg(gc.TASK_ID)
  cfg.scene.num_envs = 4
  env = ManagerBasedRlEnv(cfg=cfg, device="cpu")
  try:
    obs, _ = env.reset()
    actor = obs["actor"].shape[-1]
    critic = obs["critic"].shape[-1]
    gait = obs["gait"].shape[-1]
    action = env.action_manager.total_action_dim

    if actor != gc.ACTOR_PREFIX_DIM:
      report.fail(
        "actor prefix dimension",
        f"expected {gc.ACTOR_PREFIX_DIM}, got {actor}"
        + (
          f" -- {(hc.BASE_OBSERVATION_DIM + gc.GAIT_DIM) * 4} means the gait ID "
          "was stacked with the history"
          if actor == (hc.BASE_OBSERVATION_DIM + gc.GAIT_DIM) * 4
          else ""
        ),
      )
    elif gait != gc.GAIT_DIM:
      report.fail("gait dimension", f"expected {gc.GAIT_DIM}, got {gait}")
    elif critic != gc.CRITIC_PREFIX_DIM:
      report.fail("critic prefix dimension",
                  f"expected {gc.CRITIC_PREFIX_DIM}, got {critic}")
    elif action != gc.ACTION_DIM:
      report.fail("action dimension", f"expected {gc.ACTION_DIM}, got {action}")
    else:
      report.ok(
        "live dimensions",
        f"actor {actor}+{gait}={gc.ACTOR_INPUT_DIM} / "
        f"critic {critic}+{gait}={gc.CRITIC_INPUT_DIM} / action {action}",
      )

    terms = tuple(env.observation_manager.active_terms["actor"])
    if terms != hc.BASE_TERM_ORDER:
      report.fail("actor term order",
                  f"offset table assumes {hc.BASE_TERM_ORDER}, got {terms}")
    else:
      report.ok("actor term order")

    one_hot = obs["gait"]
    valid = (
      bool(torch.isfinite(one_hot).all())
      and bool(((one_hot == 0) | (one_hot == 1)).all())
      and bool(torch.allclose(one_hot.sum(-1), torch.ones(one_hot.shape[0])))
    )
    if not valid:
      report.fail("gait one-hot", f"not a strict one-hot: {one_hot}")
    else:
      report.ok("gait one-hot", "finite, 0/1, rows sum to 1")

    check_fsm(report, env)
  finally:
    env.close()


def check_fsm(report: Report, env) -> None:
  """Drive the FSM through a stop sequence and confirm every mode is reachable."""
  import torch

  from src.tasks.velocity.mdp.gait_command import (
    STAGE_C0,
    STAGE_C1,
    GaitConditionedVelocityCommand,
  )

  term = env.command_manager.get_term("twist")
  if not isinstance(term, GaitConditionedVelocityCommand):
    report.fail("gait FSM", f"'twist' is a {type(term).__name__}, not the gait command")
    return

  term.set_stage(STAGE_C1)
  try:
    seen = set()
    term._needs_init[:] = False
    term.hold_time[:] = 0.0
    term.mode[:] = gc.WALK
    # The predicate reads the low-pass filter, which advances in compute();
    # this drives the FSM directly, so seed it to a settled robot.
    term.ang_vel_filt.zero_()
    term.joint_vel_filt.zero_()

    term.vel_command_b[:] = 0.0
    term.vel_command_b[:, 0] = 0.5
    term._update_gait_fsm(env.step_dt)
    seen.update(term.gait_for_action.tolist())

    term.vel_command_b[:] = 0.0
    steps = int((gc.T_SETTLE_S + 0.5) / env.step_dt)
    for _ in range(steps):
      term._update_gait_fsm(env.step_dt)
      seen.update(term.gait_for_action.tolist())

    missing = [gc.mode_name(i) for i in range(gc.GAIT_DIM) if i not in seen]
    if missing:
      report.fail(
        "FSM reaches every mode",
        f"never emitted {missing}; those one-hot columns would stay constant "
        "and the policy could not learn what they mean",
      )
    else:
      report.ok("FSM reaches every mode", "STAND, WALK and W2S all emitted")

    if torch.any(term.settle_time < 0):
      report.fail("FSM timers", "negative settle time")
  finally:
    term.set_stage(STAGE_C0)


def check_checkpoint(report: Report, path: Path) -> None:
  if not path.exists():
    report.skip(
      "warm-start checkpoint",
      f"{path} not present (run init_flat_plus_gait_checkpoint.py)",
    )
    return
  try:
    import torch
  except ImportError as error:
    report.skip("warm-start checkpoint", f"torch not importable: {error}")
    return

  payload = torch.load(path, map_location="cpu", weights_only=False)
  actor = payload.get("actor_state_dict")
  if actor is None:
    report.fail("warm-start checkpoint", "no actor_state_dict")
    return

  weight = actor["mlp.0.weight"]
  if tuple(weight.shape)[1] != gc.ACTOR_INPUT_DIM:
    report.fail("warm-start first layer",
                f"expected [*,{gc.ACTOR_INPUT_DIM}], got {tuple(weight.shape)}")
    return
  report.ok("warm-start first layer", str(tuple(weight.shape)))

  gait_block = weight[:, gc.ACTOR_PREFIX_DIM :]
  if int(torch.count_nonzero(gait_block)) != 0:
    report.fail("gait columns start at zero",
                "non-zero weight on the gait one-hot in a fresh conversion")
  else:
    report.ok("gait columns start at zero")

  normalizer = actor.get("obs_normalizer._mean")
  if normalizer is not None and normalizer.shape[-1] != gc.ACTOR_PREFIX_DIM:
    report.fail(
      "warm-start normalizer width",
      f"{normalizer.shape[-1]}, expected {gc.ACTOR_PREFIX_DIM}: the gait "
      "one-hot must not be normalized",
    )
  else:
    report.ok("warm-start normalizer width", f"{gc.ACTOR_PREFIX_DIM}")

  if payload.get("iter", None) != 0:
    report.fail("warm-start lineage", f"iter={payload.get('iter')}; expected 0")
  elif payload.get("infos", {}).get("env_state", {}).get(
    "common_step_counter", None
  ) != 0:
    report.fail("warm-start lineage",
                "common_step_counter inherited; the gait curriculum would skip "
                "straight to the last stage")
  else:
    report.ok("warm-start lineage", "iter=0, curriculum counter reset")


def check_onnx(report: Report, path: Path) -> None:
  if not path.exists():
    report.skip("exported ONNX", f"{path} not present (nothing has been trained yet)")
    return
  try:
    import onnx
  except ImportError as error:
    report.skip("exported ONNX", f"onnx not importable: {error}")
    return

  model = onnx.load(str(path))
  meta = {entry.key: entry.value for entry in model.metadata_props}

  expected = gc.gait_metadata()
  missing = [k for k in expected if k not in meta]
  wrong = [
    f"{k}: expected {expected[k]!r}, got {meta[k]!r}"
    for k in expected
    if k in meta and meta[k] != str(expected[k])
  ]
  if missing:
    report.fail("ONNX gait metadata", f"missing keys: {missing}")
  elif wrong:
    report.fail("ONNX gait metadata", "; ".join(wrong))
  else:
    report.ok("ONNX gait metadata", f"{len(expected)} keys")

  for key in ("joint_names", "default_joint_pos", "action_scale",
              "joint_stiffness", "joint_damping"):
    if key not in meta:
      report.fail("ONNX common-PD metadata",
                  f"{key} missing; deploy must never guess gains")
      break
  else:
    report.ok("ONNX common-PD metadata")

  graph = model.graph
  if len(graph.input) != 1 or len(graph.output) != 1:
    report.fail("ONNX tensors",
                f"expected 1 input / 1 output, got "
                f"{len(graph.input)}/{len(graph.output)}")
    return

  def shape(value) -> list:
    return [d.dim_value if d.HasField("dim_value") else None
            for d in value.type.tensor_type.shape.dim]

  in_shape, out_shape = shape(graph.input[0]), shape(graph.output[0])
  float_type = onnx.TensorProto.FLOAT
  if in_shape != [1, gc.ACTOR_INPUT_DIM] or out_shape != [1, gc.ACTION_DIM]:
    report.fail("ONNX tensors",
                f"expected [1,{gc.ACTOR_INPUT_DIM}] -> [1,{gc.ACTION_DIM}], "
                f"got {in_shape} -> {out_shape}")
  elif graph.input[0].type.tensor_type.elem_type != float_type \
          or graph.output[0].type.tensor_type.elem_type != float_type:
    report.fail("ONNX tensors", "input/output must be float32")
  else:
    report.ok("ONNX tensors", f"{in_shape} -> {out_shape} float32")

  # The check the H4 contract does not need: which columns get normalized.
  widths = {
    tuple(init.dims)
    for init in graph.initializer
    if init.name.endswith(("_mean", "_std", "_var"))
  }
  if not widths:
    report.skip("ONNX normalizer width", "no normalizer in the graph")
  elif widths & {(1, gc.ACTOR_PREFIX_DIM), (gc.ACTOR_PREFIX_DIM,)}:
    report.ok("ONNX normalizer width", f"{gc.ACTOR_PREFIX_DIM} (prefix only)")
  else:
    report.fail(
      "ONNX normalizer width",
      f"{widths}; a {gc.ACTOR_INPUT_DIM}-wide normalizer means the stock MLP "
      "export ran and the gait one-hot is being normalized",
    )

  check_onnx_inference(report, path)


def check_onnx_inference(report: Report, path: Path) -> None:
  """Every declared mode must produce a finite action."""
  try:
    import numpy as np
    import onnxruntime as ort
  except ImportError as error:
    report.skip("ONNX inference per mode", f"onnxruntime not importable: {error}")
    return

  trace = REPO / hc.artifact_namespace(gc.HISTORY_LENGTH) / "golden_trace.txt"
  if not trace.exists():
    report.skip("ONNX inference per mode", f"{trace} not present")
    return

  packed = [
    [float(v) for v in line.split()[1:]]
    for line in trace.read_text().splitlines()
    if line.startswith("PACKED ")
  ]
  session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
  name = session.get_inputs()[0].name

  for mode_index, mode in enumerate(gc.MODE_ORDER):
    for step, prefix in enumerate(packed):
      vector = np.zeros((1, gc.ACTOR_INPUT_DIM), dtype=np.float32)
      vector[0, : gc.ACTOR_PREFIX_DIM] = prefix
      vector[0, gc.ACTOR_PREFIX_DIM + mode_index] = 1.0
      out = session.run(None, {name: vector})[0]
      if out.shape != (1, gc.ACTION_DIM) or not np.isfinite(out).all():
        report.fail(
          "ONNX inference per mode",
          f"mode {mode}, step {step}: shape {out.shape}, "
          f"finite={bool(np.isfinite(out).all())}",
        )
        return
  report.ok(
    "ONNX inference per mode",
    f"{len(packed)} steps x {len(gc.MODE_ORDER)} modes finite",
  )


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--checkpoint", type=Path, default=None)
  parser.add_argument("--onnx", type=Path, default=None)
  parser.add_argument("--strict", action="store_true",
                      help="treat skipped checks as failures (use for release gates)")
  args = parser.parse_args()

  artifacts = REPO / gc.ARTIFACT_NAMESPACE
  checkpoint = args.checkpoint or artifacts / "init_from_flat_plus_h4.pt"
  onnx_path = args.onnx or artifacts / f"policy_{gc.POLICY_CONTRACT}.onnx"

  report = Report()
  print(
    f"contract: {gc.POLICY_CONTRACT}  actor {gc.ACTOR_INPUT_DIM} "
    f"({gc.ACTOR_PREFIX_DIM}+{gc.GAIT_DIM}) / critic {gc.CRITIC_INPUT_DIM} "
    f"({gc.CRITIC_PREFIX_DIM}+{gc.GAIT_DIM}) / action {gc.ACTION_DIM}\n"
  )
  check_task(report)
  check_checkpoint(report, checkpoint)
  check_onnx(report, onnx_path)

  print(f"\n{report.failed} failed, {report.skipped} skipped")
  if report.failed:
    return 1
  if args.strict and report.skipped:
    print("strict mode: skipped checks are not acceptable for a release gate")
    return 1
  return 0


if __name__ == "__main__":
  sys.exit(main())
