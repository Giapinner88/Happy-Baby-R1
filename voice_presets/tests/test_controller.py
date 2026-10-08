from __future__ import annotations

import sys
import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HB_ROOT))

from voice_presets.config import PresetConfig  # noqa: E402
from voice_presets.controller import (  # noqa: E402
    BUTTON_B,
    BUTTON_BY_KEY,
    BUTTON_L1,
    PresetController,
)


class PresetControllerTest(unittest.TestCase):
    def setUp(self):
        self.config = PresetConfig.load()
        self.controller = PresetController(self.config)

    def update(self, buttons: int, now: float, *, remote=True, armed=False):
        return self.controller.update(
            remote_alive=remote, high_armed=armed, buttons=buttons, now=now
        )

    def test_l1_alone_toggles_only_after_three_seconds_then_requires_release(self):
        self.assertEqual([], self.update(BUTTON_L1, 0.0))
        self.assertEqual([], self.update(BUTTON_L1, 2.99))
        self.assertEqual(
            [("mode", True)],
            [(event.kind, event.value) for event in self.update(BUTTON_L1, 3.0)],
        )
        self.assertEqual([], self.update(BUTTON_L1, 6.0))
        self.assertEqual([], self.update(0, 6.1))
        self.assertEqual([], self.update(BUTTON_BY_KEY["UP"], 6.2))

    def test_l1_direction_chord_requires_mode_and_a_new_press(self):
        chord = BUTTON_L1 | BUTTON_BY_KEY["UP"]
        self.assertEqual([], self.update(chord, 0.0))
        self.update(0, 0.1)
        self.update(BUTTON_L1, 1.0)
        self.assertEqual(
            [("mode", True)],
            [(event.kind, event.value) for event in self.update(BUTTON_L1, 4.0)],
        )
        self.update(0, 4.1)
        self.assertEqual(
            [("play", "UP")],
            [(event.kind, event.value) for event in self.update(chord, 4.2)],
        )
        self.assertEqual([], self.update(chord, 4.3))
        self.update(0, 4.4)
        self.assertEqual(
            [("play", "UP")],
            [(event.kind, event.value) for event in self.update(chord, 4.5)],
        )

    def test_b_remains_an_emergency_cancel(self):
        self.assertEqual(
            [("cancel", None)],
            [(event.kind, event.value) for event in self.update(BUTTON_B, 0.0)],
        )
        self.assertEqual(
            [("cancel", None)],
            [(event.kind, event.value) for event in self.update(BUTTON_B, 0.1)],
        )

        self.update(0, 0.2)
        self.assertEqual(
            [("cancel", None)],
            [(event.kind, event.value) for event in self.update(BUTTON_B | BUTTON_L1 | BUTTON_BY_KEY["UP"], 0.3)],
        )

    def test_armed_high_level_still_accepts_l1_direction_chord(self):
        self.update(BUTTON_L1, 0.0)
        self.assertEqual(
            [("mode", True)],
            [(event.kind, event.value) for event in self.update(BUTTON_L1, 3.0, armed=True)],
        )
        self.update(0, 3.1, armed=True)
        self.assertEqual(
            [("play", "LEFT")],
            [(event.kind, event.value) for event in self.update(BUTTON_L1 | BUTTON_BY_KEY["LEFT"], 3.2, armed=True)],
        )
        self.assertEqual(
            [("cancel", None)],
            [(event.kind, event.value) for event in self.update(BUTTON_B, 3.3, armed=True)],
        )

    def test_remote_loss_does_not_create_a_late_chord_on_reconnect(self):
        chord = BUTTON_L1 | BUTTON_BY_KEY["RIGHT"]
        self.update(BUTTON_L1, 0.0)
        self.update(BUTTON_L1, 3.0)
        self.update(0, 3.1)
        self.assertEqual([], self.update(chord, 3.2, remote=False))
        self.assertEqual([], self.update(chord, 3.3))
        self.update(0, 3.4)
        self.assertEqual(
            [("play", "RIGHT")],
            [(event.kind, event.value) for event in self.update(chord, 3.5)],
        )


if __name__ == "__main__":
    unittest.main()
