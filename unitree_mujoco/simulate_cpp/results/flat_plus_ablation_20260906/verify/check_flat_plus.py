#!/usr/bin/env python3
"""Fail-closed checker for the `flat_plus_h4_v1` artifacts.

Runs the checks that must hold before an artifact is allowed near a simulator
or a robot, and reports what is genuinely missing rather than skipping quietly:

  1. task registers and reports 332/98/24 on a live environment;
  2. actor term order matches the offset table the C++ runtimes compile in;
  3. the converted warm-start checkpoint has the newest-slice mapping and zero
     older-slot weights;
  4. the exported ONNX declares the complete history contract and has the
     exact `[1,332] -> [1,24]` float32 tensors;
  5. ONNX inference on the golden trace is finite.

Checks whose input is absent are reported as SKIP with the reason, and the exit
code is non-zero only for real failures. `--strict` turns any skip into a
failure, which is what a release gate should use.

    python scripts/check_flat_plus.py
    python scripts/check_flat_plus.py --onnx path/to/policy.onnx --strict
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

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


def check_task(report: Report, history_length: int) -> None:
  try:
    import mjlab.tasks  # noqa: F401
    import src.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.registry import list_tasks, load_env_cfg
  except ImportError as error:
    report.skip("task registry / live dimensions", f"mjlab not importable: {error}")
    return

  task = hc.task_id(history_length)
  if task not in list_tasks():
    report.fail("task registry", f"{task} is not registered")
    return
  report.ok("task registry", task)

  cfg = load_env_cfg(task)
  cfg.scene.num_envs = 2
  env = ManagerBasedRlEnv(cfg=cfg, device="cpu")
  try:
    obs, _ = env.reset()
    actor = tuple(obs["actor"].shape)
    critic = tuple(obs["critic"].shape)
    action = env.action_manager.total_action_dim
    expected_actor = hc.actor_input_dim(history_length)
    if actor[-1] != expected_actor:
      report.fail("actor dimension", f"expected {expected_actor}, got {actor[-1]}")
    elif critic[-1] != hc.CRITIC_INPUT_DIM:
      report.fail("critic dimension",
                  f"expected {hc.CRITIC_INPUT_DIM}, got {critic[-1]} "
                  "(a stacked critic means the actor override leaked)")
    elif action != hc.ACTION_DIM:
      report.fail("action dimension", f"expected {hc.ACTION_DIM}, got {action}")
    else:
      report.ok("live dimensions", f"actor {actor[-1]} / critic {critic[-1]} / action {action}")

    terms = tuple(env.observation_manager.active_terms["actor"])
    if terms != hc.BASE_TERM_ORDER:
      report.fail("actor term order",
                  f"offset table assumes {hc.BASE_TERM_ORDER}, environment has {terms}")
    else:
      report.ok("actor term order")
  finally:
    env.close()


def check_checkpoint(report: Report, path: Path, history_length: int) -> None:
  if not path.exists():
    report.skip("warm-start checkpoint", f"{path} not present (run init_flat_plus_checkpoint.py)")
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

  expected_actor = hc.actor_input_dim(history_length)
  weight = actor["mlp.0.weight"]
  if tuple(weight.shape)[1] != expected_actor:
    report.fail("warm-start first layer",
                f"expected [*,{expected_actor}], got {tuple(weight.shape)}")
    return
  report.ok("warm-start first layer", str(tuple(weight.shape)))

  leaked = []
  for term in hc.BASE_TERM_ORDER:
    for lag in range(1, history_length):
      c0, c1 = hc.history_frame_slice(term, lag, history_length)
      if int(torch.count_nonzero(weight[:, c0:c1])) != 0:
        leaked.append(f"{term}@lag{lag}")
  if leaked:
    report.fail("older history slots start at zero", f"non-zero weight in {leaked}")
  else:
    report.ok("older history slots start at zero")

  if payload.get("iter", None) != 0:
    report.fail("warm-start lineage", f"iter={payload.get('iter')}; expected 0")
  elif payload.get("infos", {}).get("env_state", {}).get("common_step_counter", None) != 0:
    report.fail("warm-start lineage",
                "common_step_counter inherited; the command curriculum would skip ahead")
  else:
    report.ok("warm-start lineage", "iter=0, curriculum counter reset")


def check_onnx(report: Report, path: Path, history_length: int) -> None:
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

  expected = hc.history_metadata(history_length)
  missing, wrong = [], []
  for key, value in expected.items():
    if key not in meta:
      missing.append(key)
    elif meta[key] != str(value):
      wrong.append(f"{key}: expected {value!r}, got {meta[key]!r}")
  if missing:
    report.fail("ONNX history metadata", f"missing keys: {missing}")
  elif wrong:
    report.fail("ONNX history metadata", "; ".join(wrong))
  else:
    report.ok("ONNX history metadata", f"{len(expected)} keys")

  for key in ("joint_names", "default_joint_pos", "action_scale",
              "joint_stiffness", "joint_damping"):
    if key not in meta:
      report.fail("ONNX common-PD metadata",
                  f"{key} missing; deploy must never guess gains")
      break
  else:
    report.ok("ONNX common-PD metadata")

  def shape(value) -> list:
    return [d.dim_value if d.HasField("dim_value") else None
            for d in value.type.tensor_type.shape.dim]

  graph = model.graph
  if len(graph.input) != 1 or len(graph.output) != 1:
    report.fail("ONNX tensors",
                f"expected 1 input / 1 output, got {len(graph.input)}/{len(graph.output)}")
    return
  in_shape, out_shape = shape(graph.input[0]), shape(graph.output[0])
  float_type = onnx.TensorProto.FLOAT
  expected_actor = hc.actor_input_dim(history_length)
  if in_shape != [1, expected_actor] or out_shape != [1, hc.ACTION_DIM]:
    report.fail("ONNX tensors",
                f"expected [1,{expected_actor}] -> [1,{hc.ACTION_DIM}], "
                f"got {in_shape} -> {out_shape}")
  elif graph.input[0].type.tensor_type.elem_type != float_type \
          or graph.output[0].type.tensor_type.elem_type != float_type:
    report.fail("ONNX tensors", "input/output must be float32")
  else:
    report.ok("ONNX tensors", f"{in_shape} -> {out_shape} float32")

  check_onnx_inference(report, path, history_length)


def check_onnx_inference(report: Report, path: Path, history_length: int) -> None:
  trace = REPO / hc.artifact_namespace(history_length) / "golden_trace.txt"
  if not trace.exists():
    report.skip("ONNX inference on golden trace", f"{trace} not present")
    return
  try:
    import numpy as np
    import onnxruntime as ort
  except ImportError as error:
    report.skip("ONNX inference on golden trace", f"onnxruntime not importable: {error}")
    return

  packed = [
    [float(v) for v in line.split()[1:]]
    for line in trace.read_text().splitlines()
    if line.startswith("PACKED ")
  ]
  session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
  name = session.get_inputs()[0].name
  for index, vector in enumerate(packed):
    out = session.run(None, {name: np.asarray([vector], dtype=np.float32)})[0]
    if out.shape != (1, hc.ACTION_DIM) or not np.isfinite(out).all():
      report.fail("ONNX inference on golden trace",
                  f"step {index}: shape {out.shape}, finite={bool(np.isfinite(out).all())}")
      return
  report.ok("ONNX inference on golden trace", f"{len(packed)} steps finite")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--history-length", type=int, default=hc.HISTORY_LENGTH,
                      choices=hc.SUPPORTED_HISTORY_LENGTHS)
  parser.add_argument("--checkpoint", type=Path, default=None)
  parser.add_argument("--onnx", type=Path, default=None)
  parser.add_argument("--strict", action="store_true",
                      help="treat skipped checks as failures (use for release gates)")
  args = parser.parse_args()

  h = args.history_length
  artifacts = REPO / hc.artifact_namespace(h)
  checkpoint = args.checkpoint or artifacts / "init_from_common_pd.pt"
  onnx_path = args.onnx or artifacts / f"policy_{hc.contract_id(h)}.onnx"

  report = Report()
  print(f"contract: {hc.contract_id(h)}  actor {hc.actor_input_dim(h)} / "
        f"critic {hc.CRITIC_INPUT_DIM} / action {hc.ACTION_DIM}\n")
  check_task(report, h)
  check_checkpoint(report, checkpoint, h)
  check_onnx(report, onnx_path, h)

  print(f"\n{report.failed} failed, {report.skipped} skipped")
  if report.failed:
    return 1
  if args.strict and report.skipped:
    print("strict mode: skipped checks are not acceptable for a release gate")
    return 1
  return 0


if __name__ == "__main__":
  sys.exit(main())
