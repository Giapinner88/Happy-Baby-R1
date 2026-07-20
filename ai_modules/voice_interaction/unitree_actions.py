"""Safe helper for calling high-level Unitree R1 bridge actions."""

import os
import subprocess
import time
from pathlib import Path

from loguru import logger

SAFE_ACTIONS = {"stand_up", "stop_move", "damp", "start", "zero_torque"}


class UnitreeActionBridge:
    """Run whitelisted high-level R1 actions with cooldown protection."""

    def __init__(self, *, bridge_path: str | Path, network_interface: str, cooldown_secs: float = 2.0):
        self._bridge_path = Path(bridge_path)
        self._network_interface = network_interface
        self._cooldown_secs = cooldown_secs
        self._last_action_at = 0.0

    def run(self, action: str) -> bool:
        if action not in SAFE_ACTIONS:
            logger.warning(f"Blocked unsafe or unknown Unitree action: {action}")
            return False
        if not self._bridge_path.exists():
            logger.warning(f"Unitree action bridge is not built: {self._bridge_path}")
            return False

        now = time.monotonic()
        if now - self._last_action_at < self._cooldown_secs:
            logger.info(f"Skipping Unitree action due to cooldown: {action}")
            return False

        self._last_action_at = now
        result = subprocess.run(
            [str(self._bridge_path), "action", self._network_interface, action],
            env=os.environ.copy(),
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            logger.warning(f"Unitree action failed ({action}): {result.stderr.strip()}")
            return False
        return True
