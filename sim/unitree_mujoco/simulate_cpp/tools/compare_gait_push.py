"""Run C++ DDS/MuJoCo comparisons sequentially; save raw logs and physics CSVs.

Usage: python tools/compare_gait_push.py [--baseline-only|--push-only]
Pushes are two 0.2-s pulses at physics t=9 and 15 s, 40 N then 80 N.
They share an episode, so the second is not an independent trial.
"""
import argparse
import json
import os
import signal
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/gait_compare_20260909"
MODELS = {
    "new": "policy_flat_plus_gait_walkquality_v1.onnx",
    "old": "policy_flat_plus_gait_h4_v1.onnx",
    "h4": "policy_flat_plus_h4_v1.onnx",
}


def run(name, model, vx, schedule=None, hold=18, stop=12, extra=()):
    log = OUT / f"{name}.log"
    # Keep completed runs intact on resume, including measured failures.
    if log.exists() and ("RESULT=" in log.read_text() or "BLOCKED=FALLEN" in log.read_text()):
        print("EXISTING", name, flush=True)
        return
    env = dict(os.environ, MUJOCO_HEADLESS="1",
               MUJOCO_GAIT_TELEMETRY=str(OUT / f"{name}_physics.csv"))
    env.pop("MUJOCO_RECORD_PATH", None)
    env.pop("MUJOCO_PUSH_SCHEDULE", None)
    if schedule:
        schedule_path = OUT / f"{name}.push"
        schedule_path.write_text(schedule)
        env["MUJOCO_PUSH_SCHEDULE"] = str(schedule_path)
    cmd = ["bash", str(ROOT / "run_rough_stack.sh"), "0", "--locomotion-policy",
           str(ROOT / "policy/locomotion/flat_plus" / MODELS[model]),
           "--autotest", "compare", "gesture=0", f"vx={vx}", "warmup=5",
           f"hold={hold}", f"stop_after={stop}",
           f"csv={OUT / (name + '.csv')}", *extra]
    print("START", name, flush=True)
    with log.open("w") as stream:
        process = subprocess.Popen(cmd, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            code = process.wait(timeout=100)
        except subprocess.TimeoutExpired:
            # Let the stack shell's trap clean up its separately grouped simulator.
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            print("TIMEOUT: stop suite and inspect owned processes", name, flush=True)
            raise
    lines = log.read_text().splitlines()
    summary = [s for s in lines if any(k in s for k in ("RESULT=", "BLOCKED=", "[PUSH]", "max tilt", "Tilt"))]
    (OUT / f"{name}.run.json").write_text(json.dumps({
        "command": cmd, "returncode": code, "schedule": schedule,
        "summary": summary}, indent=2))
    print("DONE", name, code, summary, flush=True)
    if code not in (0, 2, 3, 4):
        raise RuntimeError(f"Infrastructure failure {name}: {code}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-only", action="store_true")
    parser.add_argument("--push-only", action="store_true")
    parser.add_argument("--followup-only", action="store_true",
                        help="0.6 m/s train boundary plus repeat 0.4 m/s gait comparison")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.followup_only:
        for model in MODELS:
            run(f"{model}_v06", model, .6)
        for model in ("new", "old"):
            run(f"{model}_v04_repeat", model, .4)
        raise SystemExit(0)
    if not args.push_only:
        for model in MODELS:
            for tag, vx in (("v04", .4), ("v08", .8), ("stand", 0)):
                run(f"{model}_{tag}", model, vx)
    if not args.baseline_only:
        for model in MODELS:
            for state, vx in (("stand", 0), ("walk", .4)):
                for direction, xy in (("front", (1, 0)), ("back", (-1, 0)),
                                      ("left", (0, 1)), ("right", (0, -1))):
                    schedule = "".join(f"{t} 0.2 pelvis {f*xy[0]} {f*xy[1]} 0\n"
                                       for t, f in ((9, 40), (15, 80)))
                    run(f"{model}_{state}_{direction}", model, vx, schedule,
                        hold=17, stop=-1, extra=("max_tilt_deg=45", "max_drop=0.2", "max_drift=2"))
