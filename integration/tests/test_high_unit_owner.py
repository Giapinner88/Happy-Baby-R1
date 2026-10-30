import os
import subprocess
import tempfile
import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]
CHECK = HB_ROOT / "integration" / "scripts" / "high_unit_owner.sh"
HIGH = "/home/unitree/HB/controller"
CLEAN = f"""LoadState=loaded
DropInPaths=
WorkingDirectory={HIGH}/build
Environment=DISPLAY= HB_ROOT=/home/unitree/HB HB_INTEGRATION_SOCKET=/run/hb/integration.sock
ExecStart={{ path={HIGH}/build/run_r1 ; argv[]={HIGH}/build/run_r1 ; ignore_errors=no ; start_time=[n/a] ; stop_time=[n/a] ; pid=0 ; code=(null) ; status=0/0 }}
"""
# Drop-in thật thấy trên robot 2026-10-29 (p5.conf).
P5 = """LoadState=loaded
DropInPaths=/etc/systemd/system/hb_high_level.service.d/p5.conf
WorkingDirectory=/home/unitree/r1_p4_owner/controller/build
Environment=DISPLAY= HB_ROOT=/home/unitree/HB HB_PROJECT_DIR=/home/unitree/r1_p4_owner/controller P4_YAW_PORT=5571
ExecStart={ path=/home/unitree/r1_p4_owner/controller/build/run_r1 ; argv[]=/home/unitree/r1_p4_owner/controller/build/run_r1 ; ignore_errors=no ; start_time=[n/a] ; stop_time=[n/a] ; pid=0 ; code=(null) ; status=0/0 }
"""


class HighUnitOwnerTest(unittest.TestCase):
    def run_check(self, show, project_dir=None):
        with tempfile.TemporaryDirectory() as tmp:
            fake = Path(tmp) / "systemctl"
            (Path(tmp) / "show.txt").write_text(show)
            fake.write_text(f'#!/bin/sh\ncat "{tmp}/show.txt"\n')
            fake.chmod(0o755)
            env = {k: v for k, v in os.environ.items() if k != "HB_PROJECT_DIR"}
            env["HB_SYSTEMCTL"] = str(fake)
            if project_dir is not None:
                env["HB_PROJECT_DIR"] = project_dir
            return subprocess.run(
                ["bash", "-c", f'source "{CHECK}"; hb_check_high_unit_owner "{HIGH}"'],
                env=env, capture_output=True, text=True)

    def test_installed_unit_passes(self):
        result = self.run_check(CLEAN)
        self.assertEqual(result.returncode, 0, result.stdout)

    def test_p5_dropin_redirecting_the_motor_owner_fails(self):
        result = self.run_check(P5)
        self.assertEqual(result.returncode, 1)
        for reason in ("drop-in", "ExecStart", "WorkingDirectory", "HB_PROJECT_DIR"):
            self.assertIn(reason, result.stdout)

    def test_any_dropin_fails_even_if_paths_match(self):
        show = CLEAN.replace("DropInPaths=", "DropInPaths=/etc/systemd/system/hb_high_level.service.d/x.conf")
        self.assertEqual(self.run_check(show).returncode, 1)

    def test_foreign_project_dir_in_process_env_fails(self):
        result = self.run_check(CLEAN, project_dir="/home/unitree/r1_p4_owner/controller")
        self.assertEqual(result.returncode, 1)
        self.assertIn("HB_PROJECT_DIR", result.stdout)

    def test_unit_not_installed_is_skipped(self):
        self.assertEqual(self.run_check("LoadState=not-found\n").returncode, 0)

    def test_preflight_checks_the_unit_and_pins_the_project_dir(self):
        preflight = (HB_ROOT / "integration" / "scripts" / "preflight.sh").read_text()
        self.assertIn("hb_check_high_unit_owner", preflight)
        self.assertIn('HB_PROJECT_DIR="$HIGH_DIR" "$BIN" --preflight', preflight)


if __name__ == "__main__":
    unittest.main()
