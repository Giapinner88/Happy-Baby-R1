#!/usr/bin/env python3
"""Run integration/scripts/robot_steps.sh against fake systemctl/sudo.

The robot-side deploy steps own the DISARMED guard, backup and restore; these
tests execute them for real (no robot, no systemd) instead of grepping text.
"""

import os
import subprocess
import tarfile
import tempfile
import time
import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]
STEPS = HB_ROOT / "integration" / "scripts" / "robot_steps.sh"

FAKE_SYSTEMCTL = r"""#!/usr/bin/env bash
U="$SIM_UNITS"; quiet=0; cmd=""; units=(); prop=""
for a in "$@"; do
  case "$a" in
    --quiet) quiet=1 ;; --value|--no-pager|--job-mode=*) ;;
    -p) prop=next ;;
    *) if [[ "$prop" == next ]]; then prop="$a"
       elif [[ -z "$cmd" ]]; then cmd="$a"
       else units+=("${a%.service}"); fi ;;
  esac
done
echo "$*" >>"$U/calls.log"
case "$cmd" in
  is-active) rc=0; for u in "${units[@]}"; do [[ -f "$U/$u.active" ]] || rc=3; done; exit $rc ;;
  is-enabled) exit 1 ;;
  show) u="${units[0]}"
    case "$prop" in
      MainPID) if [[ -f "$U/$u.active" ]]; then cat "$U/$u.pid"; else echo 0; fi ;;
      ActiveEnterTimestamp) cat "$U/$u.ts" 2>/dev/null || echo "" ;;
    esac ;;
  restart) for u in "${units[@]}"; do
      echo $(( $(cat "$U/$u.pid" 2>/dev/null || echo 100) + 1 )) >"$U/$u.pid"
      date '+%a %Y-%m-%d %H:%M:%S %Z' >"$U/$u.ts"; touch "$U/$u.active"; done ;;
esac
exit 0
"""

FAKE_SUDO = """#!/usr/bin/env bash
while [[ "${1:-}" == -* ]]; do shift; done
exec "$@"
"""


class RobotStepsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.bin = root / "bin"
        self.units = root / "units"
        self.dest = root / "home" / "HB"
        self.status = root / "status.env"
        for d in (self.bin, self.units, self.dest):
            d.mkdir(parents=True)
        for name, body in (("systemctl", FAKE_SYSTEMCTL), ("sudo", FAKE_SUDO)):
            path = self.bin / name
            path.write_text(body)
            path.chmod(0o755)
        (self.units / "calls.log").write_text("")
        self.env = {
            **os.environ,
            "PATH": f"{self.bin}:{os.environ['PATH']}",
            "SIM_UNITS": str(self.units),
            "HB_DEST": str(self.dest),
            "HB_STATUS_FILE": str(self.status),
        }

    def tearDown(self):
        self.tmp.cleanup()

    def run_step(self, *args):
        with STEPS.open() as script:
            return subprocess.run(["bash", "-s", "--", *args], stdin=script, env=self.env,
                                  capture_output=True, text=True, timeout=60)

    def high_running(self, armed):
        (self.units / "hb_high_level.active").touch()
        (self.units / "hb_high_level.pid").write_text("200\n")
        state = "LOCOMOTION" if armed else "DISARMED"
        self.status.write_text(f"high_alive=1\nhigh_armed={int(armed)}\nhigh_state={state}\n")

    def test_restart_high_refuses_while_armed(self):
        self.high_running(armed=True)
        result = self.run_step("restart-high")
        self.assertEqual(3, result.returncode, result.stdout + result.stderr)
        self.assertNotIn("restart", (self.units / "calls.log").read_text())

    def test_restart_high_when_disarmed_waits_for_new_process(self):
        self.high_running(armed=False)
        result = self.run_step("restart-high")
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertIn("restart hb_high_level.service", (self.units / "calls.log").read_text())
        self.assertEqual("201", (self.units / "hb_high_level.pid").read_text().strip())

    def test_probe_reports_armed_and_unknown_status_as_unsafe(self):
        self.high_running(armed=True)
        self.assertIn("high=armed", self.run_step("probe").stdout)
        self.status.unlink()
        self.assertIn("high=armed", self.run_step("probe").stdout)
        (self.units / "hb_high_level.active").unlink()
        self.assertIn("high=inactive", self.run_step("probe").stdout)

    def test_backup_skips_dance_and_build_rotates_and_keeps_prev_binary(self):
        ctrl = self.dest / "controller"
        for rel, text in (("config/tuning.yaml", "v1"), ("policies/dance/d1/policy.onnx", "big"),
                          ("build/run_r1", "bin-v1"), ("src/audio/a.mp3", "mp3")):
            (ctrl / rel).parent.mkdir(parents=True, exist_ok=True)
            (ctrl / rel).write_text(text)
        (ctrl / "build" / "run_r1").chmod(0o755)
        manifest = self.dest / "integration" / "config" / "model_manifest.conf"
        manifest.parent.mkdir(parents=True)
        manifest.write_text("MODEL_SHA256=old\n")
        for i in range(3):
            result = self.run_step("backup", f"id{i}", "2", "controller")
            self.assertEqual(0, result.returncode, result.stderr)
            time.sleep(1.1)
        backups = sorted((self.dest.parent / "HB_backups").glob("HB_*.tar.gz"))
        self.assertEqual(["HB_id1.tar.gz", "HB_id2.tar.gz"], [p.name for p in backups])
        with tarfile.open(backups[-1]) as tar:
            names = tar.getnames()
        self.assertIn("controller/config/tuning.yaml", names)
        self.assertFalse([n for n in names if "policies/dance" in n or "/build" in n], names)
        self.assertEqual("bin-v1", (ctrl / "build" / "run_r1.prev").read_text())

        # Deploy writes a new version, then restore returns config and binary.
        (ctrl / "config" / "tuning.yaml").write_text("v2")
        (ctrl / "build" / "run_r1").write_text("bin-v2")
        manifest.write_text("MODEL_SHA256=new\n")
        result = self.run_step("restore", str(backups[-1]), "controller")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertEqual("v1", (ctrl / "config" / "tuning.yaml").read_text())
        self.assertEqual("bin-v1", (ctrl / "build" / "run_r1").read_text())
        # Config and manifest return together, or ExecStartPre fails on the next start.
        self.assertEqual("MODEL_SHA256=old\n", manifest.read_text())

    def test_pending_marker_expires_after_unit_restart(self):
        self.high_running(armed=False)
        self.run_step("mark", "hb_high_level")
        self.assertIn("pending_hb_high_level=1", self.run_step("probe").stdout)
        time.sleep(1.1)
        self.run_step("restart-high")
        self.assertIn("pending_hb_high_level=0", self.run_step("probe").stdout)


if __name__ == "__main__":
    unittest.main()
