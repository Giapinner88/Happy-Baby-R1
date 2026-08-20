"""Compatibility gate for enabling simulator base velocity from an R1 policy."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


class PolicyGateError(ValueError):
    """Raised when a policy lacks the evidence required for simulator velocity."""


def _load_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise PolicyGateError(f"Cannot read {label}: {path}") from exc
    if not isinstance(value, dict):
        raise PolicyGateError(f"{label} must be a JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_isaaclab_velocity_policy(manifest_path: Path, r1_usd_path: Path) -> dict[str, Any]:
    """Return a compact verified identity, or fail closed.

    The selected manifest must be created by ``promote``.  Its linked
    evaluation manifest establishes the task, simulator evaluation result, R1
    asset compatibility, and observation/action signature.
    """

    manifest_path = manifest_path.expanduser().resolve()
    manifest = _load_object(manifest_path, "policy promotion manifest")
    if manifest.get("framework") != "rl_lab" or manifest.get("task") != "Unitree-R1-Velocity":
        raise PolicyGateError("Velocity requires a promoted rl_lab Unitree-R1-Velocity policy.")
    expected_asset_hash = _sha256(r1_usd_path)
    if manifest.get("r1_usd_sha256") != expected_asset_hash:
        raise PolicyGateError("Policy promotion manifest does not match the current R1 USD asset.")
    signature = manifest.get("observation_action_signature")
    if not isinstance(signature, dict) or not signature:
        raise PolicyGateError("Policy promotion manifest has no observation/action signature.")

    evaluation_ref = manifest.get("evaluation_manifest")
    if not isinstance(evaluation_ref, str) or not evaluation_ref:
        raise PolicyGateError("Policy promotion manifest has no linked evaluation manifest.")
    evaluation_path = Path(evaluation_ref).expanduser()
    if not evaluation_path.is_absolute():
        evaluation_path = manifest_path.parent / evaluation_path
    evaluation = _load_object(evaluation_path.resolve(), "policy evaluation manifest")
    if evaluation.get("status") != "passed":
        raise PolicyGateError("Velocity requires evaluation status='passed'.")
    if evaluation.get("framework") != "rl_lab" or evaluation.get("task") != "Unitree-R1-Velocity":
        raise PolicyGateError("Evaluation manifest is not for IsaacLab R1 velocity.")
    if evaluation.get("r1_usd_sha256") != expected_asset_hash:
        raise PolicyGateError("Evaluation manifest does not match the current R1 USD asset.")
    if evaluation.get("observation_action_signature") != signature:
        raise PolicyGateError("Policy and evaluation observation/action signatures differ.")

    return {
        "manifest": str(manifest_path),
        "framework": "rl_lab",
        "task": "Unitree-R1-Velocity",
        "r1_usd_sha256": expected_asset_hash,
        "observation_action_signature": signature,
    }
