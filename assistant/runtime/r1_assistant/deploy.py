"""Isolated owner build, read-only diagnostics, transactional systemd installation."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import pwd
import shutil
import subprocess
import sys
import time

from .bundle import sha, verify
from .config import durable_write, load, write

UNITS = ("hb_r1_assistant.service", "hb_r1_assistant_agent.service", "hb_voice.service", "hb_high_level.service")
LEGACY = ("r1-p5-agent.service", "r1-p6-search.service", "hb_local_approach.service", "hb_teleop_runtime.service")
MARKERS = ("/etc/hb/local_approach_voice", "/etc/hb/object_search_enabled")
REQUIRED_CTEST = "^(state_motion_guard_test|application_motion_safety_test|teleop_h4_policy_gate_test|teleop_receiver_test|input_sample_progress_test|remote_health_test|gait_scheduler_test|tuning_include_test|config_reader_test|preflight_test)$"
REQUIRED_TARGETS = ("run_r1", "state_motion_guard_test", "application_motion_safety_test",
                    "teleop_h4_policy_gate_test", "teleop_receiver_test", "input_sample_progress_test",
                    "remote_health_test", "gait_scheduler_test", "tuning_include_test")


def run(argv, **kwargs):
    return subprocess.run([str(x) for x in argv], check=True, **kwargs)


def disarmed(path=Path("/run/hb/status.env"), now=None):
    values = dict(line.split("=", 1) for line in Path(path).read_text().splitlines() if "=" in line)
    age = (time.monotonic() if now is None else now) - int(values["updated_monotonic_ms"]) / 1000
    if (not 0 <= age <= 2 or values.get("high_alive") != "1"
            or values.get("high_armed") != "0" or not values.get("high_state", "").startswith("DISARMED")):
        raise RuntimeError("Refusing service changes: owner must be healthy, fresh and DISARMED, with robot supported per SOP")
    return values


def sole_owner(expected=None, proc=Path("/proc")):
    owners = []
    for path in proc.iterdir():
        if not path.name.isdigit():
            continue
        try:
            argv = path.joinpath("cmdline").read_bytes().split(b"\0")
            if argv and Path(argv[0].decode(errors="replace")).name == "run_r1":
                owners.append(path.joinpath("exe").resolve())
        except (OSError, ValueError):
            continue
    if len(owners) != 1 or (expected is not None and owners[0] != Path(expected).resolve()):
        raise RuntimeError("expected exactly one matching run_r1 owner; observed " + str(owners))
    return str(owners[0])


def prepare_baseline(root, shared=Path("/home/unitree/HB")):
    root, shared = Path(root).resolve(), Path(shared).resolve()
    verify(root)
    config = load(root / "config/robot.json")
    output = Path(config["hb_source_root"])
    expected = root.parent.parent / "baselines" / root.name
    if output != expected or output == shared or shared in output.parents:
        raise ValueError("private baseline must be <base>/baselines/<release-id>, outside shared HB")
    from .baseline import validate
    record = json.loads((root / "app/hb_baseline.json").read_text())
    if output.exists():
        validate(output, record)
        return output
    # Only dependencies/assets are shared read-only by convention. Source and
    # config are independent copies; no file in shared HB is written here.
    links = ("controller/policies", "controller/thirdparty", "controller/motions",
             "teleop", "voice_presets", "docs", "test", "voice/.venv",
             "voice/unitree_bridge/build", "integration/build")
    for policy in record["policies"].values():
        bundle = shared / "controller/policies/locomotion" / policy["policy_bundle"]
        if sha(bundle / "policy.onnx") != policy["policy_sha256"]:
            raise ValueError("shared policy artifact differs from private baseline")
    shutil.copytree(root / "hb_source", output, symlinks=True)
    for name in links:
        source, target = shared / name, output / name
        if source.exists() and not target.exists() and not target.is_symlink():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.symlink_to(source, target_is_directory=source.is_dir())
    validate(output, record)
    return output


def stage_owner(source, output, app, build=False):
    source, output, app = Path(source).resolve(), Path(output).resolve(), Path(app).resolve()
    baseline_path = app / "hb_baseline.json"
    if baseline_path.exists():
        from .baseline import validate
        validate(source, json.loads(baseline_path.read_text()))
    if output == source or source in output.parents:
        raise ValueError("owner overlay must be outside shared HB source")
    if output.exists():
        raise FileExistsError(output)
    before = {str(p.relative_to(source)): sha(p) for folder in ("controller/src", "controller/config")
              for p in (source / folder).rglob("*") if p.is_file()}
    before["controller/CMakeLists.txt"] = sha(source / "controller/CMakeLists.txt")
    output.mkdir(parents=True)
    def ignore(directory, names):
        excluded = {name for name in names if name == "__pycache__" or name == "build" or name.startswith("build-")}
        if Path(directory) == source / "controller":
            excluded.update({"policies", "thirdparty", "motions"} & set(names))
        return excluded
    shutil.copytree(source / "controller", output / "controller", symlinks=True, ignore=ignore)
    for name in ("policies", "thirdparty", "motions"):
        if (source / "controller" / name).exists():
            (output / "controller" / name).symlink_to(source / "controller" / name, target_is_directory=True)
    for name in ("teleop", "integration", "voice", "voice_presets", "docs", "test", "hb", "Makefile"):
        if (source / name).exists():
            (output / name).symlink_to(source / name, target_is_directory=(source / name).is_dir())
    header = source / "controller/src/input/ApproachReceiver.hpp"
    if not header.exists():
        hook = output / "hook"
        run([sys.executable, app / "experiments/r1_object_grounding/p6_owner_adapter/prepare.py",
             "--source-root", source, "--output-dir", hook], stdout=subprocess.DEVNULL)
        manifest = json.loads((hook / "manifest.json").read_text())
        for name in manifest["output_sha256"]:
            shutil.copy2(hook / name, output / name)
    elif "approach_receiver_.Apply" not in (source / "controller/src/app/ApplicationPolicy.cpp").read_text():
        raise ValueError("APV1 header exists without the expected owner hook")
    # The orchestrator needs explicit STOP -> Release handoff for a subsequent
    # search yaw task; update the header only in the isolated build tree.
    if header.exists() and sha(header) not in {
            "188aebac888bf0c2af59b5b058c861e9f0df2e95b90befb19979cfe38b4ad6c7",
            sha(app / "experiments/r1_object_grounding/p6_owner_adapter/ApproachReceiver.hpp")}:
        raise ValueError("team APV1 receiver changed; review before replacing the isolated copy")
    shutil.copy2(app / "experiments/r1_object_grounding/p6_owner_adapter/ApproachReceiver.hpp",
                 output / "controller/src/input/ApproachReceiver.hpp")
    if any(sha(source / name) != digest for name, digest in before.items()):
        raise ValueError("shared HB changed during staging")
    write(output / "owner_manifest.json", {"source_root": str(source), "base_sha256": before,
          "staged_source_sha256": {str(p.relative_to(output)): sha(p)
                                   for p in (output / "controller/src").rglob("*") if p.is_file()},
          "policy_or_gain_changed": False})
    if build:
        with (output / "build.log").open("w") as log:
            run(["cmake", "-S", output / "controller", "-B", output / "controller/build", "-DCMAKE_BUILD_TYPE=Release"], stdout=log, stderr=subprocess.STDOUT)
            run(["cmake", "--build", output / "controller/build", "--parallel", "2", "--target", *REQUIRED_TARGETS], stdout=log, stderr=subprocess.STDOUT)
            run(["ctest", "--test-dir", output / "controller/build", "-R", REQUIRED_CTEST, "--output-on-failure"], stdout=log, stderr=subprocess.STDOUT)
            run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O2",
                 app / "experiments/r1_object_grounding/p6_owner_adapter/test_release.cpp",
                 "-o", output / "controller/build/apv1_handoff_test"], stdout=log, stderr=subprocess.STDOUT)
            run([output / "controller/build/apv1_handoff_test"], stdout=log, stderr=subprocess.STDOUT)
        write(output / "build_metadata.json", {"binary_sha256": sha(output / "controller/build/run_r1"),
                                               "ctest_passed": True, "ctest_pattern": REQUIRED_CTEST,
                                               "apv1_handoff_passed": True})
    return output


def doctor(root, config_path=None, robot=False):
    root = Path(root).resolve()
    manifest = verify(root)
    config = load(config_path or root / "config/robot.json")
    report = {"bundle_verified": True, "files": len(manifest["files_sha256"]),
              "mode": config["mode"], "hardware_tested": False}
    if not robot:
        return report
    if str(root) != manifest["install_root"]:
        raise ValueError("release must be at its declared robot install path")
    baseline_path = Path(config["app_root"]) / "hb_baseline.json"
    if baseline_path.exists():
        from .baseline import validate
        validate(config["hb_source_root"], json.loads(baseline_path.read_text()))
    profile = json.loads(Path(config["approach_config"]).read_text())
    if profile["execute"] != (config["mode"] == "execute"):
        raise ValueError("assistant/approach mode mismatch")
    model = Path(profile["model_dir"])
    signature = json.loads((model / "signature.json").read_text())
    for filename, key in (("dac.onnx", "onnx_sha256"), ("geometry.npz", "geometry_sha256")):
        if sha(model / filename) != signature[key]:
            raise ValueError("depth model hash mismatch: " + filename)
    if signature["calibration_sha256"] != manifest["calibration_sha256"]:
        raise ValueError("camera calibration differs from the depth engine geometry")
    engine = json.loads((model / "engine.json").read_text())
    if sha(model / "dac.engine") != engine["sha256"] or not (model / "reference_input.npy").is_file():
        raise ValueError("depth engine/reference missing or changed")
    for executable, imports, pythonpath in (
            (config["search_python"], "import numpy,cv2,PIL", None),
            (config["approach_python"], "import numpy,cv2,tensorrt", None),
            ("/usr/bin/python3", "import numpy,cv2,unitree_sdk2py",
             os.pathsep.join((config["unitree_sdk_python_root"], config["unitree_python_site"])))):
        kwargs = {"stdout": subprocess.DEVNULL}
        if pythonpath:
            kwargs["env"] = dict(os.environ, PYTHONPATH=pythonpath)
        run([executable, "-c", imports], **kwargs)
    app = Path(config["app_root"])
    run([config["approach_python"], "-c",
         "import sys,json; sys.path[:0]=sys.argv[2:]; import runtime; runtime.validate_config(json.load(open(sys.argv[1])))",
         config["approach_config"], app, app / "experiments/r1_object_grounding/p6_robot_local"], stdout=subprocess.DEVNULL)
    owner = Path(config["hb_root"])
    metadata = json.loads((owner / "build_metadata.json").read_text())
    if (metadata.get("ctest_passed") is not True or metadata.get("apv1_handoff_passed") is not True
            or sha(owner / "controller/build/run_r1") != metadata["binary_sha256"]):
        raise ValueError("isolated owner binary is not verified")
    provenance = json.loads((owner / "owner_manifest.json").read_text())
    for name, digest in provenance["base_sha256"].items():
        if sha(Path(provenance["source_root"]) / name) != digest:
            raise ValueError("shared HB changed since staging: " + name)
    for name, digest in provenance["staged_source_sha256"].items():
        if sha(owner / name) != digest:
            raise ValueError("isolated owner source changed: " + name)
    sys.path.insert(0, str(Path(config["app_root"]) / "agent/target_control"))
    from control import runtime_info
    control_config = json.loads((Path(config["app_root"]) / "agent/target_control/config.json").read_text())
    runtime_info(owner, control_config)
    # Bind search's pinned policy to the selected R2+A policy, not just a file on disk.
    selections = {}
    for line in (owner / "controller/config/locomotion.yaml").read_text().splitlines():
        key, sep, value = line.split("#", 1)[0].partition(":")
        if sep:
            selections[key.strip()] = value.strip().strip("\"'")
    if selections.get("walk_policy") != control_config["policy_bundle"]:
        raise ValueError("search policy does not match selected walk_policy")
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", R1_ASSISTANT_VOICE="1",
               R1_P5_VOICE_ENABLED="0", PYTHONPATH=os.pathsep.join(str(p) for p in (
                   root / "overlay", root / "runtime", Path(config["hb_source_root"]) / "voice")))
    run([Path(config["hb_source_root"]) / "voice/.venv/bin/python", "-c",
         "from hb_voice.vision_tools import VisionToolController; c=VisionToolController(network_interface='lo'); assert callable(c._capture_frame); assert {'find_and_point','approach_object','cancel_task','task_status'} <= {t.name for t in c.tools}"], env=env, stdout=subprocess.DEVNULL)
    report.update(dependencies_verified=True, engine_verified=True, owner_binary_verified=True,
                  note="Read-only: no camera, cloud request or motor RPC")
    return report


def render_units(root, config_path, owner_env):
    root, config_path = Path(root), Path(config_path)
    config = load(config_path)
    source, owner = Path(config["hb_source_root"]), Path(config["hb_root"])
    common = "[Unit]\nAfter=hb_high_level.service network-online.target\nWants=network-online.target\n\n[Service]\nType=simple\nUser=unitree\nGroup=unitree\nEnvironmentFile=-/etc/hb/stack.env\nEnvironment=PYTHONDONTWRITEBYTECODE=1\nEnvironment=OPENBLAS_NUM_THREADS=1\nEnvironment=OMP_NUM_THREADS=1\nRestart=on-failure\nRestartSec=3\nKillMode=control-group\nTimeoutStopSec=8\nNoNewPrivileges=yes\nUMask=0077\n"
    assistant = common + "RuntimeDirectory=hb-approach\nRuntimeDirectoryMode=0755\nRuntimeDirectoryPreserve=yes\nNice=10\nCPUAffinity=4 5 6 7\nCPUQuota=250%\nMemoryMax=5G\nTasksMax=128\n"
    assistant = assistant.replace("After=hb_high_level.service network-online.target", "After=hb_high_level.service hb_r1_assistant_agent.service network-online.target\nRequires=hb_r1_assistant_agent.service")
    assistant += "WorkingDirectory=%s\nExecStart=/usr/bin/env PYTHONPATH=%s:%s R1_P5_VOICE_ENABLED=0 R1_ASSISTANT_VOICE=0 %s %s serve --config %s\n\n[Install]\nWantedBy=multi-user.target\n" % (
        config["app_root"], root / "runtime", config["app_root"], config["approach_python"], root / "bin/r1-assistant", config_path)
    agent = common + "WorkingDirectory=%s\nExecStart=/usr/bin/env PYTHONPATH=%s:%s:%s R1_P5_VOICE_ENABLED=0 R1_ASSISTANT_VOICE=0 /usr/bin/python3 %s serve --runtime-root %s --interface %s\n\n[Install]\nWantedBy=multi-user.target\n" % (
        config["app_root"], config["app_root"], config["unitree_sdk_python_root"], config["unitree_python_site"],
        Path(config["app_root"]) / "agent/robot_agent.py", owner, config["interface"])
    high = (source / "integration/systemd/hb_high_level.service.in").read_text()
    for key, value in (("@HB_ROOT@", str(owner)), ("@RUN_USER@", "unitree"), ("@INTERFACE@", config["interface"])):
        high = high.replace(key, value)
    high = high.replace("EnvironmentFile=-/etc/hb/stack.env", "EnvironmentFile=-/etc/hb/stack.env\nEnvironmentFile=" + str(owner_env))
    # Override voice only; the existing canonical voice runtime/persona/audio remain in use.
    voice = "[Unit]\nAfter=hb_r1_assistant.service\nRequires=hb_r1_assistant.service\n\n[Service]\nReadWritePaths=/run/hb-approach\nEnvironment=PYTHONDONTWRITEBYTECODE=1\nExecStart=\nExecStart=/usr/bin/env R1_ASSISTANT_VOICE=1 R1_P5_VOICE_ENABLED=0 R1_ASSISTANT_PORT=%s R1_ASSISTANT_CAMERA_LOCK=%s PYTHONPATH=%s:%s:%s %s -m hb_voice\n" % (
        config["port"], config["camera_lock"], root / "overlay", root / "runtime", source / "voice", source / "voice/.venv/bin/python")
    files = {
        "/etc/systemd/system/hb_r1_assistant.service": assistant,
        "/etc/systemd/system/hb_r1_assistant_agent.service": agent,
        "/etc/systemd/system/hb_high_level.service": high,
        "/etc/systemd/system/hb_voice.service.d/r1-assistant.conf": voice,
    }
    for unit in LEGACY:
        files["/etc/systemd/system/%s.d/r1-assistant.conf" % unit] = "[Unit]\nConditionPathExists=!/etc/r1-assistant/enabled\n"
    return files


def unit_state(unit):
    return {key: subprocess.run(["systemctl", command, "--quiet", unit]).returncode == 0
            for key, command in (("active", "is-active"), ("enabled", "is-enabled"))}


def install(root, execute=False, operator_ready=False):
    if os.geteuid() != 0:
        raise PermissionError("Run install with sudo on the robot")
    if execute and not operator_ready:
        raise ValueError("execute requires --operator-ready: supported robot, clear area, E-stop operator per SOP")
    root = Path(root).resolve()
    manifest = verify(root)
    if str(root) != manifest["install_root"]:
        raise ValueError("copy release to declared install_root first")
    disarmed()
    sole_owner()
    if Path("/etc/r1-assistant/enabled").exists():
        raise RuntimeError("assistant already installed: rollback the active release before reinstalling")
    if Path("/run/hb-approach/active").exists():
        raise RuntimeError("approach task marker present: finish/cancel existing task first")
    dropins = subprocess.check_output(["systemctl", "show", "hb_high_level.service", "-p", "DropInPaths", "--value"], text=True).strip()
    if dropins:
        raise RuntimeError("Existing high-level drop-ins must be reviewed before installing: " + dropins)
    config = load(root / "config/robot.json")
    if (root / "hb_source").is_dir():
        prepare_baseline(root)
    base = root.parent.parent
    activation = base / "activations" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    activation.mkdir(parents=True)
    config["mode"] = "execute" if execute else "shadow"
    config["approach_config"] = str(activation / "approach.json")
    profile = json.loads((root / "config/approach.json").read_text())
    profile["execute"] = profile["clear_corridor_confirmed"] = execute
    profile["planner"]["corridor"]["operator_confirmed"] = execute
    profile["gait_approach"]["execute_enabled"] = execute
    write(activation / "robot.json", config)
    write(activation / "approach.json", profile)
    durable_write(activation / "owner.env", "R1_APPROACH_PORT=5573\nR1_APPROACH_ALLOW_NONZERO=%d\n" % int(execute))
    owner = Path(config["hb_root"])
    if not owner.exists():
        stage_owner(config["hb_source_root"], owner, config["app_root"], build=True)
    doctor(root, activation / "robot.json", robot=True)
    files = render_units(root, activation / "robot.json", activation / "owner.env")
    files["/etc/r1-assistant/enabled"] = str(activation) + "\n"
    originals = {}
    for name in [*files, *MARKERS]:
        path = Path(name)
        if path.is_symlink():
            raise ValueError("refusing to replace symlink: " + name)
        originals[name] = path.read_text() if path.exists() else None
    states = {unit: unit_state(unit) for unit in (*UNITS, *LEGACY)}
    write(activation / "rollback.json", {"files": originals, "units": states,
          "config": str(activation / "robot.json"), "release": str(root)})
    disarmed()  # Recheck after dependency/build work, immediately before mutation.
    try:
        run(["systemctl", "stop", "hb_voice.service"])
        for unit in LEGACY:
            subprocess.run(["systemctl", "stop", unit], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        disarmed()
        run(["systemctl", "stop", "hb_high_level.service"])
        for name, content in files.items():
            path = Path(name)
            path.parent.mkdir(parents=True, exist_ok=True)
            durable_write(path, content)
        for marker in MARKERS:
            Path(marker).unlink(missing_ok=True)
        for directory in (Path(config["runs_dir"]), Path("/run/hb-approach")):
            directory.mkdir(parents=True, exist_ok=True)
            user = pwd.getpwnam("unitree")
            os.chown(directory, user.pw_uid, user.pw_gid)
        run(["systemctl", "daemon-reload"])
        restarted_after = time.monotonic()
        run(["systemctl", "start", "hb_high_level.service"])
        deadline = time.monotonic() + 45
        while True:
            try:
                state = disarmed()
                if int(state["updated_monotonic_ms"]) / 1000 < restarted_after:
                    raise RuntimeError("owner status predates this start")
                sole_owner(owner / "controller/build/run_r1")
                break
            except (OSError, ValueError, KeyError, RuntimeError):
                if time.monotonic() >= deadline:
                    raise RuntimeError("new owner did not become healthy DISARMED")
                time.sleep(.3)
        run(["systemctl", "enable", "--now", "hb_r1_assistant_agent.service", "hb_r1_assistant.service"])
        run(["systemctl", "restart", "hb_voice.service"])
        time.sleep(2)
        run(["systemctl", "is-active", "--quiet", *UNITS])
        from .voice import call
        os.environ["R1_ASSISTANT_PORT"] = str(config["port"])
        state = call("/status", {})
        if state.get("active") is not False or state.get("mode") != config["mode"]:
            raise RuntimeError("assistant health check failed")
        write(activation / "installed.json", {"mode": config["mode"], "status": "installed_disarmed", "hardware_tested": False})
        return activation
    except BaseException:
        rollback(activation, failed_install=True)
        raise


def rollback(activation, failed_install=False):
    if os.geteuid() != 0:
        raise PermissionError("Run rollback with sudo on the robot")
    record = json.loads((Path(activation) / "rollback.json").read_text())
    if not failed_install:
        disarmed()
    else:
        # An unhealthy replacement owner may not update status; fail closed if any status is ARMED.
        status = Path("/run/hb/status.env")
        if status.exists() and "high_armed=1" in status.read_text().splitlines():
            raise RuntimeError("rollback blocked: owner ARMED; support and DISARM per SOP")
    for unit in UNITS:
        stopped = subprocess.run(["systemctl", "stop", unit], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if stopped.returncode and unit_state(unit)["active"]:
            raise RuntimeError("could not stop " + unit)
    for name, original in record["files"].items():
        path = Path(name)
        if original is None:
            path.unlink(missing_ok=True)
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            durable_write(path, original)
    os.sync()  # removals/restores reach the disk before units change state
    run(["systemctl", "daemon-reload"])
    for unit, state in record["units"].items():
        command = "enable" if state["enabled"] else "disable"
        subprocess.run(["systemctl", command, unit], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for unit in ("hb_high_level.service", *LEGACY, "hb_r1_assistant_agent.service", "hb_r1_assistant.service", "hb_voice.service"):
        if record["units"][unit]["active"]:
            run(["systemctl", "start", unit])
    write(Path(activation) / "rolled_back.json", {"restored": True})
