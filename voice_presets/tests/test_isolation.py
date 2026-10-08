"""Regression guards for the preset service's strict scope."""

from __future__ import annotations

import unittest
from pathlib import Path


class IsolationTest(unittest.TestCase):
    def test_preset_runtime_has_no_motor_control_api(self) -> None:
        package = Path(__file__).resolve().parents[1]
        runtime = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (package / "app.py", package / "controller.py", package / "config.py")
        )
        for forbidden in ("LowCmd", "LocoClient", "ChannelPublisher", "robot_action"):
            self.assertNotIn(forbidden, runtime)

