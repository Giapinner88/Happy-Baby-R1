from __future__ import annotations

import sys
import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT))

from voice_presets.app import CoordinatorStatus  # noqa: E402


class PresetProtocolTest(unittest.TestCase):
    def test_coordinator_status_keeps_raw_button_bits_and_cancel_epoch(self):
        status = CoordinatorStatus.parse(
            b"v=1 seq=7 remote_alive=1 buttons=4096 remote_seq=22 high_armed=0 "
            b"high_busy=0 preset_busy=1 cancel_epoch=3"
        )
        self.assertTrue(status.remote_alive)
        self.assertEqual(4096, status.buttons)
        self.assertEqual(22, status.remote_sequence)
        self.assertFalse(status.high_armed)
        self.assertFalse(status.high_busy)
        self.assertTrue(status.preset_busy)
        self.assertEqual(3, status.cancel_epoch)

    def test_unknown_protocol_is_rejected(self):
        with self.assertRaises(ValueError):
            CoordinatorStatus.parse(b"v=99 remote_alive=1")


if __name__ == "__main__":
    unittest.main()
