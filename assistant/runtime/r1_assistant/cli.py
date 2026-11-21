"""Workstation packaging and robot runtime entry points."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import uuid

from .config import load


def serve(config):
    from .bundle import verify
    from .backends import RobotBackends
    from .service import Coordinator, make_server
    if config.get("release_manifest"):
        verify(Path(config["release_manifest"]).parent)
    Path(config["instance_lock"]).parent.mkdir(parents=True, exist_ok=True)
    instance = open(config["instance_lock"], "a")
    fcntl.flock(instance, fcntl.LOCK_EX | fcntl.LOCK_NB)
    coordinator = Coordinator(config, RobotBackends(config))
    server = make_server(coordinator)
    stopping = threading.Event()

    def watchdog():
        while not stopping.wait(.1):
            coordinator.tick()

    def stop(*_):
        stopping.set()
        coordinator.route("/cancel", {})
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    monitor = threading.Thread(target=watchdog, daemon=True)
    monitor.start()
    print(json.dumps({"status": "ready", "mode": config["mode"], "port": config["port"]}), flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    finally:
        stopping.set()
        coordinator.route("/cancel", {})
        record = coordinator.active
        if record and record["thread"]:
            record["thread"].join(timeout=5)
        server.server_close()
        instance.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("pack", help="Create a new source-only release; never deploy")
    p.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parents[4])
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--install-root", type=Path, required=True)
    p.add_argument("--calibration", type=Path)
    p.add_argument("--hb-source-root", type=Path, help="Read-only team HB checkout; pin effective policy and runtime hashes")
    p.add_argument("--isolated-hb", action="store_true", help="Include private HB source snapshot; leave shared robot HB untouched")
    p.add_argument("--walk-policy", help="Private isolated-HB walk policy bundle override")
    p.add_argument("--tts-provider", choices=("openai", "elevenlabs"),
                   help="Private isolated-HB voice: tts.provider in voice/config/tuning.yaml (keys stay in /etc/hb/stack.env)")
    p = sub.add_parser("prepare-baseline", help="Stage private HB source and link existing robot dependencies; do not activate")
    p.add_argument("--root", type=Path, required=True)
    p = sub.add_parser("check", help="Verify release hashes; --robot also checks dependencies/model/voice")
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--config", type=Path)
    p.add_argument("--robot", action="store_true")
    p = sub.add_parser("prepare-owner", help="Copy/patch/build an isolated HB owner; do not activate")
    p.add_argument("--source-root", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--app-root", type=Path, required=True)
    p.add_argument("--build", action="store_true")
    p = sub.add_parser("setup-env", help="Create only the separate search Python environment")
    p.add_argument("--root", type=Path, required=True)
    p = sub.add_parser("install", help="Robot-only transactional installation, requires DISARMED")
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--execute", action="store_true")
    p.add_argument("--operator-ready", action="store_true")
    p = sub.add_parser("rollback", help="Restore prior services/configuration while DISARMED")
    p.add_argument("--activation", type=Path, required=True)
    p = sub.add_parser("serve")
    p.add_argument("--config", type=Path, required=True)
    p = sub.add_parser("request", help="Exercise the same API used by voice")
    p.add_argument("action", choices=("gesture", "find_and_point", "approach", "status", "cancel"))
    p.add_argument("--query", default="")
    p.add_argument("--gesture-id")
    p.add_argument("--request-id", default=None)
    p.add_argument("--task-id")
    p.add_argument("--port", type=int, default=5595)
    args = parser.parse_args()
    if args.command == "pack":
        from .bundle import build
        record = build(args.source_root, args.output_dir, args.install_root, args.calibration,
                       args.hb_source_root, args.isolated_hb, args.walk_policy, args.tts_provider)
        result = {"output_dir": str(args.output_dir), "files": len(record["files_sha256"]), "mode": "shadow"}
    elif args.command == "prepare-baseline":
        from .deploy import prepare_baseline
        result = {"baseline": str(prepare_baseline(args.root)), "activated": False}
    elif args.command == "check":
        from .deploy import doctor
        result = doctor(args.root, args.config, args.robot)
    elif args.command == "prepare-owner":
        from .deploy import stage_owner
        result = {"owner_root": str(stage_owner(args.source_root, args.output_dir, args.app_root, args.build)), "activated": False}
    elif args.command == "setup-env":
        from .bundle import verify
        verify(args.root)
        config = load(args.root / "config/robot.json")
        environment = Path(config["search_python"]).parent.parent
        if not Path(config["search_python"]).exists():
            subprocess.run(["/usr/bin/python3", "-m", "venv", str(environment)], check=True)
        subprocess.run([config["search_python"], "-m", "pip", "install", "pip<25.1"], check=True)
        subprocess.run([config["search_python"], "-m", "pip", "install", "-r",
                        str(args.root / "app/experiments/r1_object_grounding/p6_onboard/requirements-py38.txt")], check=True)
        result = {"search_python": config["search_python"], "system_python_modified": False}
    elif args.command == "install":
        from .deploy import install
        result = {"activation": str(install(args.root, args.execute, args.operator_ready)), "hardware_tested": False}
    elif args.command == "rollback":
        from .deploy import rollback
        rollback(args.activation)
        result = {"restored": True}
    elif args.command == "serve":
        serve(load(args.config))
        return
    else:
        from .voice import call
        os.environ["R1_ASSISTANT_PORT"] = str(args.port)
        if args.action in ("status", "cancel"):
            result = call("/" + args.action, {"task_id": args.task_id} if args.task_id else {})
        else:
            result = call("/tasks", {"action": args.action, "query": args.query, "gesture_id": args.gesture_id,
                                     "request_id": args.request_id or uuid.uuid4().hex})
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
