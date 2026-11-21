"""Reproducible, source-only release. No credentials, weights or HB source copied."""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

from .config import write


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def merge(base, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def private_model_manifest(controller, team):
    """Derive the private manifest from the controller runtime, its source of truth."""
    controller, team = Path(controller), Path(team)
    binary = team / "controller/build/run_r1"
    if not binary.is_file():
        raise FileNotFoundError("team run_r1 is required to validate a walk-policy override: " + str(binary))
    temporary_links = []
    try:
        for name in ("policies", "thirdparty", "motions"):
            target = controller / name
            source = team / "controller" / name
            if source.exists() and not target.exists():
                target.symlink_to(source, target_is_directory=True)
                temporary_links.append(target)
        architecture = subprocess.run(["uname", "-m"], check=True, capture_output=True,
                                      text=True).stdout.strip()
        ort = team / "controller/thirdparty" / (
            "onnxruntime_aarch64/lib" if architecture in ("aarch64", "arm64") else "onnxruntime/lib")
        environment = os.environ.copy()
        environment["HB_PROJECT_DIR"] = str(controller)
        environment["LD_LIBRARY_PATH"] = str(ort) + (
            ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
        result = subprocess.run([str(binary), "--preflight"], env=environment,
                                capture_output=True, text=True)
        output = result.stdout + result.stderr
        if result.returncode:
            raise ValueError("private walk-policy runtime preflight failed:\n" + "\n".join(output.splitlines()[-20:]))
        line = next((line for line in output.splitlines()
                     if line.startswith("[Preflight] MODEL ")), "")
        fields = dict(re.findall(r"(contract|rel|input|output)=([^ ]+)", line))
        required = {"contract", "rel", "input", "output"}
        if set(fields) != required:
            raise ValueError("private runtime did not report a complete MODEL line")
        model = controller / fields["rel"]
        if not model.is_file():
            raise FileNotFoundError(model)
        return ("# Walking policy accepted by the current controller runner.\n"
                f"MODEL_REL={fields['rel']}\n"
                f"MODEL_SHA256={sha(model)}\n"
                f"MODEL_CONTRACT={fields['contract']}\n"
                f"MODEL_INPUT={fields['input']}\n"
                f"MODEL_OUTPUT={fields['output']}\n")
    finally:
        for path in temporary_links:
            path.unlink()


def verify(root):
    root = Path(root).resolve()
    record = json.loads((root / "manifest.json").read_text())
    if record.get("schema") != "r1_assistant_bundle.v1":
        raise ValueError("wrong release schema")
    for name, digest in record["files_sha256"].items():
        path = Path(name)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError("unsafe release path")
        target = root / path
        if not target.is_file() or sha(target) != digest:
            raise ValueError("release file missing or changed: " + name)
    return record


def build(source, output, install_root, calibration=None, hb_source_root=None, isolated_hb=False,
          walk_policy=None, tts_provider=None):
    source, output, install_root = Path(source).resolve(), Path(output).resolve(), Path(install_root)
    if not install_root.is_absolute() or any(c.isspace() for c in str(install_root)):
        raise ValueError("absolute install root without whitespace required")
    if install_root.parent.name != "releases":
        raise ValueError("install root must have layout <base>/releases/<release-id>")
    if output.exists():
        raise FileExistsError(output)
    project = source / "integrations/r1_assistant"
    baseline = None
    if hb_source_root is not None:
        from .baseline import snapshot
        baseline = snapshot(hb_source_root, walk_policy=walk_policy)
    if walk_policy and not isolated_hb:
        raise ValueError("walk policy override requires --isolated-hb")
    if tts_provider and not isolated_hb:
        raise ValueError("tts provider override requires --isolated-hb")
    if isolated_hb and baseline is None:
        raise ValueError("isolated HB requires --hb-source-root")
    calibration = Path(calibration or source / "data/runs/r1_camera/20261027_combined_charuco_30mm/evaluation_v1/calibration_all/calibration.npz")
    if not calibration.is_file():
        raise FileNotFoundError(calibration)
    output.mkdir(parents=True)
    shutil.copytree(project / "src/r1_assistant", output / "runtime/r1_assistant",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    shutil.copytree(project / "overlay", output / "overlay",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    shutil.copytree(project / "config", output / "defaults")
    shutil.copy2(project / "README.md", output / "README.md")
    shutil.copy2(project / "VALIDATION.md", output / "VALIDATION.md")
    if isolated_hb:
        team = Path(hb_source_root).resolve()
        def ignore(directory, names):
            excluded = {n for n in names if n in {"build", ".venv", "__pycache__", ".cache", "logs", ".git"}
                        or n.startswith(("build-", "build_")) or n.endswith(".pyc")}
            if Path(directory) == team / "controller":
                excluded.update({"policies", "thirdparty", "motions"} & set(names))
            return excluded
        for name in ("controller", "integration", "voice"):
            shutil.copytree(team / name, output / "hb_source" / name, symlinks=True, ignore=ignore)
        for name in ("hb", "Makefile"):
            shutil.copy2(team / name, output / "hb_source" / name)
        if walk_policy:
            config = output / "hb_source/controller/config/locomotion.yaml"
            lines = config.read_text().splitlines(keepends=True)
            selected = [i for i, line in enumerate(lines) if line.lstrip().startswith("walk_policy:")]
            if len(selected) != 1:
                raise ValueError("expected exactly one walk_policy selection")
            index = selected[0]
            indent = lines[index][:-len(lines[index].lstrip())]
            lines[index] = indent + "walk_policy: " + walk_policy + "\n"
            config.write_text("".join(lines))
            baseline["files_sha256"]["controller/config/locomotion.yaml"] = sha(config)
            baseline["variant"] = {"walk_policy_override": walk_policy,
                                   "team_source_root": str(team)}
            manifest = output / "hb_source/integration/config/model_manifest.conf"
            manifest.write_text(private_model_manifest(output / "hb_source/controller", team))
            baseline["files_sha256"]["integration/config/model_manifest.conf"] = sha(manifest)
    if tts_provider:
        tuning = output / "hb_source/voice/config/tuning.yaml"
        lines = tuning.read_text().splitlines(keepends=True)
        selected = [i for i, line in enumerate(lines) if line.lstrip().startswith("provider:")]
        if len(selected) != 1:
            raise ValueError("expected exactly one tts provider selection in voice/config/tuning.yaml")
        index = selected[0]
        indent = lines[index][:-len(lines[index].lstrip())]
        lines[index] = indent + 'provider: "%s"\n' % tts_provider
        tuning.write_text("".join(lines))
        baseline["files_sha256"]["voice/config/tuning.yaml"] = sha(tuning)
        baseline.setdefault("variant", {}).update(tts_provider_override=tts_provider,
                                                  team_source_root=str(Path(hb_source_root).resolve()))
    app = output / "app"
    sources = set()
    for folder in ("ai_modules/navigation", "ai_modules/vision", "experiments/r1_target_control",
                   "experiments/r1_object_grounding/p4_search", "experiments/r1_object_grounding/p6_robot_local"):
        sources.update(p.relative_to(source) for p in (source / folder).iterdir()
                       if p.is_file() and p.suffix in (".py", ".json") and not p.name.startswith("test_"))
    sources.update(Path(name) for name in (
        "assets/R1.urdf", "teleop/r1/kinematics.py",
        "experiments/r1_object_grounding/p2_bbox_baseline/config.json",
        "experiments/r1_object_grounding/p2_bbox_baseline/run.py",
        "experiments/r1_object_grounding/p2_bbox_baseline/p2_look_at_dryrun/capture_observation.py",
        "experiments/r1_object_grounding/p5_voice_search/server.py",
        "experiments/r1_object_grounding/p6_onboard/local_bridge.py",
        "experiments/r1_object_grounding/p6_onboard/requirements-py38.txt",
        "experiments/r1_object_grounding/p6_owner_adapter/prepare.py",
        "experiments/r1_object_grounding/p6_owner_adapter/test_release.cpp",
        "experiments/r1_object_grounding/p6_owner_adapter/ApproachReceiver.hpp"))
    for name in sorted(sources):
        destination = app / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / name, destination)
    mappings = {
        "sensor/camera_shadow_worker.py": "experiments/r1_camera/depth_capability_v1/camera_shadow_worker.py",
        "sensor/state_sync.py": "ai_modules/vision/state_sync.py",
        "sensor/odom_sync.py": "ai_modules/vision/odom_sync.py",
        "agent/robot_agent.py": "experiments/r1_object_grounding/p4_search/hardware/robot_agent.py",
        "agent/feedback.py": "experiments/r1_object_grounding/p4_search/feedback.py",
        "agent/target_control/control.py": "experiments/r1_target_control/control.py",
        "agent/target_control/config.json": "integrations/r1_assistant/config/target_control.robot.json",
        "agent/capture_observation.py": "experiments/r1_object_grounding/p2_bbox_baseline/p2_look_at_dryrun/capture_observation.py",
        "experiments/r1_object_grounding/p6_robot_local/bounded_trial.py": "experiments/r1_object_grounding/p6_owner_adapter/bounded_trial.py",
    }
    for name, original in mappings.items():
        destination = app / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source / original, destination)
    # Mapping inputs are deployment sources too; preserve their hashes even
    # when their destination differs from the repository path.
    sources.update(Path(original) for original in mappings.values())
    if baseline is not None:
        # Both import paths must use the same effective team policy.
        control = json.loads((app / "agent/target_control/config.json").read_text())
        control.update(baseline["policies"]["walk_policy"])
        for name in ("agent/target_control/config.json", "experiments/r1_target_control/config.json"):
            write(app / name, control)
        write(app / "hb_baseline.json", baseline)
    camera = app / "data/calibration/r1_camera.npz"
    camera.parent.mkdir(parents=True)
    shutil.copy2(calibration, camera)
    bbox_path = app / "experiments/r1_object_grounding/p2_bbox_baseline/config.json"
    bbox = json.loads(bbox_path.read_text())
    bbox["calibration"] = "data/calibration/r1_camera.npz"
    write(bbox_path, bbox)
    robot = json.loads((project / "config/robot.json").read_text())
    if isolated_hb:
        robot["hb_source_root"] = str(install_root.parent.parent / "baselines" / install_root.name)
    robot["hb_root"] = str(install_root.parent.parent / "owners" / install_root.name)
    robot["app_root"] = str(install_root / "app")
    robot["release_manifest"] = str(install_root / "manifest.json")
    robot["approach_config"] = str(install_root / "config/approach.json")
    for name, relative in (("adapter_argv", "config/adapter_argv.json"),
                           ("bbox_config", "app/experiments/r1_object_grounding/p2_bbox_baseline/config.json"),
                           ("vision_config", "app/experiments/r1_target_control/vision_config.json")):
        robot["search"][name] = str(install_root / relative)
    profile = json.loads((source / "experiments/r1_object_grounding/p6_robot_local/gait_aware_arc.json").read_text())
    merge(profile, json.loads((project / "config/approach.patch.json").read_text()))
    profile.update(execute=False, clear_corridor_confirmed=False)
    profile["planner"]["corridor"]["operator_confirmed"] = False
    profile["grounding"]["calibration"] = "data/calibration/r1_camera.npz"
    profile["gait_approach"]["execute_enabled"] = False
    profile["runs_dir"] = robot["runs_dir"] + "/approach"
    profile["camera_lock"] = robot["camera_lock"]
    write(output / "config/robot.json", robot)
    write(output / "config/approach.json", profile)
    write(output / "config/adapter_argv.json", ["/usr/bin/python3", str(install_root / "app/experiments/r1_object_grounding/p6_onboard/local_bridge.py")])
    launcher = output / "bin/r1-assistant"
    launcher.parent.mkdir()
    launcher.write_text("#!/usr/bin/env python3\nimport sys\nfrom pathlib import Path\nsys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime'))\nfrom r1_assistant.cli import main\nif __name__ == '__main__':\n    main()\n")
    launcher.chmod(0o755)
    # Parse all robot runtime source against Python 3.8 before shipping.
    for path in output.rglob("*.py"):
        ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))
    git = subprocess.run(["git", "rev-parse", "HEAD"], cwd=source, capture_output=True, text=True)
    files = {str(p.relative_to(output)): sha(p) for p in output.rglob("*") if p.is_file()}
    record = {"schema": "r1_assistant_bundle.v1", "install_root": str(install_root),
              "source_commit": git.stdout.strip(), "source_python": sys.version,
              "files_sha256": files, "calibration_sha256": sha(camera),
              "source_files_sha256": {str(p): sha(source / p) for p in sorted(sources)},
              "hardware_tested": False, "default_mode": "shadow"}
    write(output / "manifest.json", record)
    return record
