"""Read-only binding to the team's effective HB checkout, including dirty files."""
import hashlib
from pathlib import Path
import subprocess


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def values(path):
    result = {}
    for raw in Path(path).read_text().splitlines():
        key, sep, value = raw.split("#", 1)[0].partition(":")
        if sep:
            result[key.strip()] = value.strip().strip("\"'")
    return result


def policy_record(root, name):
    relative = Path(name)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("unsafe policy bundle: " + name)
    bundle = Path(root) / "controller/policies/locomotion" / name
    meta = values(bundle / "policy.yaml")
    digest = sha(bundle / "policy.onnx")
    if digest != meta.get("sha256") or not meta.get("contract"):
        raise ValueError("team policy hash/contract mismatch: " + name)
    return {"policy_bundle": name, "policy_contract": meta["contract"],
            "policy_sha256": digest}


def snapshot(root, walk_policy=None):
    root = Path(root).resolve()
    selections = values(root / "controller/config/locomotion.yaml")
    policies = {}
    for key in ("walk_policy", "teleop_policy"):
        name = walk_policy if key == "walk_policy" and walk_policy else selections.get(key) or selections["walk_policy"]
        policies[key] = policy_record(root, name)
    files = {}
    for folder in ("controller/src", "controller/config", "integration", "voice/hb_voice", "voice/config"):
        for path in sorted((root / folder).rglob("*")):
            relative_parts = path.relative_to(root).parts
            generated = any(part in {"build", "__pycache__", ".cache", "logs", ".venv"}
                            or part.startswith(("build-", "build_")) for part in relative_parts)
            if path.is_file() and not generated and path.suffix != ".pyc":
                files[str(path.relative_to(root))] = sha(path)
    for name in ("controller/CMakeLists.txt", "voice/pyproject.toml", "voice/uv.lock"):
        if (root / name).is_file():
            files[name] = sha(root / name)
    for policy in policies.values():
        prefix = "controller/policies/locomotion/" + policy["policy_bundle"]
        for name in ("policy.yaml", "policy.onnx"):
            files[prefix + "/" + name] = sha(root / prefix / name)
    git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True)
    return {"schema": "r1_hb_baseline.v1", "source_root": str(root),
            "commit": git.stdout.strip(), "policies": policies, "files_sha256": files}


def validate(root, record):
    root = Path(root).resolve()
    for name, digest in record["files_sha256"].items():
        path = Path(name)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError("unsafe baseline path")
        target = root / path
        if not target.is_file() or sha(target) != digest:
            raise ValueError("HB differs from team baseline: " + name +
                             "; sync the team's checkout using ./hb deploy, then repack")
