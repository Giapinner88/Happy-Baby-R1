"""Verify a transferred RMA-meta release directory before C++ simulation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import onnx
import yaml


ROOT = Path(__file__).resolve().parents[1]
BUNDLED_SOURCE = ROOT / "tools" / "python"
PROJECT_SOURCE = ROOT / "src" / "tasks" / "velocity"
sys.path.insert(
  0, str(BUNDLED_SOURCE if BUNDLED_SOURCE.is_dir() else PROJECT_SOURCE)
)

from rma_meta.checkpoint import load_checkpoint_models  # noqa: E402
from rma_meta.config import load_rma_meta_cfg  # noqa: E402


def sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as stream:
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def verify_sha256sums(release_dir: Path) -> int:
  checksum_path = release_dir / "SHA256SUMS"
  if not checksum_path.is_file():
    raise FileNotFoundError("release SHA256SUMS is missing")
  verified = 0
  for line in checksum_path.read_text().splitlines():
    expected, separator, relative_text = line.partition("  ")
    if not separator or len(expected) != 64:
      raise ValueError(f"invalid SHA256SUMS line: {line!r}")
    relative = Path(relative_text)
    if relative.is_absolute() or ".." in relative.parts:
      raise ValueError(f"unsafe SHA256SUMS path: {relative}")
    target = release_dir / relative
    if not target.is_file():
      raise FileNotFoundError(f"release file is missing: {relative}")
    if sha256_file(target) != expected:
      raise ValueError(f"release checksum mismatch: {relative}")
    verified += 1
  if verified == 0:
    raise ValueError("release SHA256SUMS contains no entries")
  return verified


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--release-dir", type=Path, required=True)
  args = parser.parse_args()
  release_dir = args.release_dir.expanduser().resolve()
  if not release_dir.is_dir():
    raise NotADirectoryError(f"release directory does not exist: {release_dir}")
  verified_count = verify_sha256sums(release_dir)

  required_verifier_files = (
    "tools/check_rma_meta_release.py",
    "tools/python/rma_meta/__init__.py",
    "tools/python/rma_meta/checkpoint.py",
    "tools/python/rma_meta/config.py",
    "tools/python/rma_meta/models.py",
    "tools/python/rma_meta/runtime.py",
  )
  for relative in required_verifier_files:
    if not (release_dir / relative).is_file():
      raise FileNotFoundError(f"self-contained release verifier is missing: {relative}")

  manifest = yaml.safe_load(
    (release_dir / "params/rma_meta_manifest.yaml").read_text()
  )
  if int(manifest.get("bundle_version", 0)) != 2:
    raise ValueError("release manifest bundle_version must be 2")
  if manifest.get("enabled") is not False:
    raise ValueError("release manifest must remain staging-disabled")

  checkpoint_path = release_dir / manifest["checkpoint"]
  if sha256_file(checkpoint_path) != manifest["checkpoint_sha256"]:
    raise ValueError("release checkpoint does not match manifest")
  cfg = load_rma_meta_cfg(release_dir / manifest["train_config"]["path"])
  payload, checkpoint_cfg, _, _, adaptation = load_checkpoint_models(
    checkpoint_path,
    map_location="cpu",
    expected_stage="deployment_finetune",
    expected_expert_names=cfg.expert_names,
  )
  if checkpoint_cfg != cfg or adaptation is None:
    raise ValueError("release checkpoint/config/adaptation contract differs")
  if payload["registry_sha256"] != manifest["registry_sha256"]:
    raise ValueError("release checkpoint and registry provenance differ")

  config_path = release_dir / manifest["train_config"]["path"]
  registry_path = release_dir / manifest["expert_registry"]["path"]
  if sha256_file(config_path) != manifest["train_config"]["sha256"]:
    raise ValueError("release train config does not match manifest")
  registry_hash = sha256_file(registry_path)
  if registry_hash != manifest["expert_registry"]["sha256"]:
    raise ValueError("release expert registry does not match manifest entry")
  if registry_hash != manifest["registry_sha256"]:
    raise ValueError("release expert registry provenance hash differs")

  evaluation_path = release_dir / manifest["evaluation"]["path"]
  if sha256_file(evaluation_path) != manifest["evaluation"]["sha256"]:
    raise ValueError("release evaluation does not match manifest")
  evaluation = json.loads(evaluation_path.read_text())
  if evaluation.get("checkpoint_sha256") != manifest["checkpoint_sha256"]:
    raise ValueError("release evaluation was produced from another checkpoint")
  if evaluation.get("registry_sha256") != manifest["registry_sha256"]:
    raise ValueError("release evaluation was produced from another registry")

  deploy = yaml.safe_load(
    (release_dir / "params/deploy.rma_meta.yaml").read_text()
  )
  if deploy["rma_meta"].get("enabled") is not False:
    raise ValueError("transferred rma_meta config must remain disabled")
  if deploy.get("meta_policy", {}).get("enabled") is not False:
    raise ValueError("transferred legacy meta_policy config must remain disabled")
  expected_models = {
    "rma_meta_adapter.onnx",
    "rma_meta_selector.onnx",
    *(f"{name}.onnx" for name in cfg.expert_names),
  }
  actual_models = {path.name for path in (release_dir / "exported").glob("*.onnx")}
  if actual_models != expected_models:
    raise ValueError(
      f"release ONNX set differs: expected {expected_models}, got {actual_models}"
    )
  for path in sorted((release_dir / "exported").glob("*.onnx")):
    onnx.checker.check_model(onnx.load(path, load_external_data=False))
  if sha256_file(release_dir / manifest["adapter"]["path"]) != manifest["adapter"]["sha256"]:
    raise ValueError("release adapter ONNX does not match manifest")
  if sha256_file(release_dir / manifest["selector"]["path"]) != manifest["selector"]["sha256"]:
    raise ValueError("release selector ONNX does not match manifest")
  for expert in manifest["experts"]:
    if sha256_file(release_dir / expert["path"]) != expert["sha256"]:
      raise ValueError(f"release expert ONNX does not match manifest: {expert['name']}")

  print(f"[PASS] complete RMA-meta release: {release_dir}")
  print(f"verified_files={verified_count}")
  print(f"checkpoint_stage={payload['stage']}")
  print(f"expert_names={','.join(cfg.expert_names)}")
  print(f"registry_sha256={manifest['registry_sha256']}")


if __name__ == "__main__":
  main()
