"""What `make teleop-arms` actually launches.

The point of this path is that nothing in this repository solves on it. That is
a claim about process wiring, which is easy to break silently: a stray default,
a profile that did not get swapped, a solver stage quietly dropped. These tests
assert the wiring rather than trusting it.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from teleop.r1.launcher import (
    BRIDGE_ENV,
    SIM_ENV,
    PilotLaunchSpec,
    _wait_for_quest_ready,
    build_commands,
)

ROOT = Path(__file__).resolve().parents[2]


def make_spec(**overrides) -> PilotLaunchSpec:
    defaults = dict(
        protocol="t007_whole_upper_body",
        run_root=ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T007" / "runs",
        repo_root=ROOT,
        host_ip="192.168.1.106",
        duration_s=10.0,
        cert_file=Path("/tmp/cert.pem"),
        key_file=Path("/tmp/key.pem"),
        physics_hz=200.0,
        control_hz=30.0,
        trigger_value_threshold=5.0,
        stop_file_dir=Path("/tmp"),
        disable_self_collisions=True,
    )
    defaults.update(overrides)
    return PilotLaunchSpec(**defaults)


class LauncherSolverStageTest(unittest.TestCase):
    def test_bridge_lifetime_includes_quest_readiness_window(self):
        bridge, _solver, sim = build_commands(
            make_spec(duration_s=10.0, quest_ready_timeout_s=12.0),
            Path("/tmp/out"),
            Path("/tmp/stop"),
            Path("/tmp/log"),
        )
        self.assertEqual(bridge[bridge.index("--duration-s") + 1], "22.0")
        self.assertEqual(sim[sim.index("--duration-s") + 1], "10.0")

    def test_ready_wait_accepts_connected_event(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "connection.jsonl"
            path.write_text(json.dumps({"event": "connected"}) + "\n", encoding="utf-8")
            process = type("Process", (), {"poll": lambda self: None})()
            self.assertTrue(_wait_for_quest_ready(path, process, 0.1))

    def test_no_solver_stage_by_default(self):
        _bridge, solver, _sim = build_commands(
            make_spec(), Path("/tmp/out"), Path("/tmp/stop"), Path("/tmp/log")
        )
        self.assertIsNone(solver, "pilots that solve inside the simulator must be unaffected")

    def test_solver_stage_runs_in_the_bridge_environment(self):
        _bridge, solver, _sim = build_commands(
            make_spec(solver_args=["scripts/teleop/run_r1_upstream_ik_stream.py", "--passthrough"]),
            Path("/tmp/out"),
            Path("/tmp/stop"),
            Path("/tmp/log"),
        )
        assert solver is not None
        # The solver needs CasADi and the Pinocchio 3 bindings, which live with
        # the Quest wrapper and not with Isaac.
        self.assertIn(BRIDGE_ENV, solver)
        self.assertNotIn(SIM_ENV, solver)
        self.assertIn("--passthrough", solver)


class TeleopArmsWiringTest(unittest.TestCase):
    """The dry run is the contract: it prints exactly what would be executed."""

    def setUp(self):
        # Tự tạo cert giả thay vì bỏ qua test khi không thấy file trong /tmp.
        # Trước đây ba test này im lặng skip mỗi khi /tmp bị dọn, tức là lớp
        # kiểm tra đường khởi chạy biến mất đúng lúc không ai để ý.
        self._tmp = tempfile.TemporaryDirectory()
        directory = Path(self._tmp.name)
        cert, key = directory / "cert.pem", directory / "key.pem"
        cert.write_text("placeholder", encoding="utf-8")
        key.write_text("placeholder", encoding="utf-8")
        self.addCleanup(self._tmp.cleanup)
        result = subprocess.run(
            [sys.executable, "scripts/teleop/run_t007_upper_body_pilot.py",
             "--host-ip", "192.168.1.106", "--upstream-solver", "--dry-run",
             "--cert-file", str(cert), "--key-file", str(key)],
            cwd=ROOT, capture_output=True, text=True,
        )
        self.output = result.stdout + result.stderr

    def test_three_stages_are_launched(self):
        self.assertIn("bridge:", self.output)
        self.assertIn("solver:", self.output)
        self.assertIn("sim:", self.output)

    def test_the_simulator_gets_the_upstream_profile_and_no_repo_solver(self):
        sim_line = next(
            line for line in self.output.splitlines() if line.startswith("sim:")
        )
        self.assertIn("--upstream-joint-stream-config", sim_line)
        # These select this repository's own solvers and must not appear.
        self.assertNotIn("--whole-upper-body-config", sim_line)
        self.assertNotIn("--offline-joint-trajectory", sim_line)

    def test_the_solver_stage_is_the_vendor_one(self):
        solver_line = next(
            line for line in self.output.splitlines() if line.startswith("solver:")
        )
        self.assertIn("run_r1_upstream_ik_stream.py", solver_line)
        self.assertIn("--passthrough", solver_line)


class MakefileTest(unittest.TestCase):
    def setUp(self):
        self.makefile = (ROOT / "Makefile").read_text(encoding="utf-8")

    def test_teleop_arms_uses_the_upstream_command(self):
        block = re.search(
            r"^teleop-arms(?:\s*:[^\n]*)?\n(.*?)(?=\n\S)",
            self.makefile,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(block, "teleop-arms target not found")
        self.assertIn("TELEOP_UPSTREAM_CMD", block.group(1))

    def test_the_upstream_command_passes_the_vendor_flag(self):
        self.assertIn("--upstream-solver", self.makefile)

    def test_the_differential_path_is_still_reachable(self):
        # Kept so the two solvers can still be compared on one trace.
        self.assertIn("teleop-arms-differential:", self.makefile)

    def test_host_ip_is_required_at_invocation_not_hardcoded(self):
        self.assertRegex(self.makefile, r"(?m)^HOST_IP\s+\?=\s*$")
        self.assertIn("HOST_IP=192.168.1.106", self.makefile)

    def test_certificate_directory_is_derived_from_host_ip(self):
        self.assertIn("HOST_IP_TAG  = $(subst .,_,$(strip $(HOST_IP)))", self.makefile)
        self.assertIn("happybaby_$(HOST_IP_TAG)/cert.pem", self.makefile)
        self.assertIn("happybaby_$(HOST_IP_TAG)/key.pem", self.makefile)


if __name__ == "__main__":
    unittest.main()
