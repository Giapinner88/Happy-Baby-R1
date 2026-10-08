"""Pure R3-1 button state machine; intentionally independent from audio I/O."""

from __future__ import annotations

from dataclasses import dataclass

from .config import PresetConfig


BUTTON_L1 = 1 << 1
BUTTON_B = 1 << 9
BUTTON_BY_KEY = {
    "UP": 1 << 12,
    "RIGHT": 1 << 13,
    "DOWN": 1 << 14,
    "LEFT": 1 << 15,
}


@dataclass(frozen=True)
class ControllerEvent:
    kind: str  # mode | play | cancel
    value: str | bool | None = None


class PresetController:
    """Own the preset mode and translate L1+d-pad chords into audio requests."""

    def __init__(self, config: PresetConfig):
        self._config = config
        self.enabled = config.mode.default_enabled
        self._previous_buttons = 0
        self._l1_started_at: float | None = None
        self._l1_latched = False

    def update(
        self,
        *,
        remote_alive: bool,
        high_armed: bool,
        buttons: int,
        now: float,
    ) -> list[ControllerEvent]:
        """Return edge-triggered events without ever mutating the R3-1 command."""
        events: list[ControllerEvent] = []
        buttons &= 0xFFFF

        if not remote_alive:
            self._previous_buttons = buttons
            self._l1_started_at = None
            self._l1_latched = bool(buttons & BUTTON_L1)
            return events

        # ``high_armed`` is diagnostic only; this channel controls the speaker,
        # not the robot. The L1 modifier keeps these presses out of the
        # high-level bare-d-pad gesture detector.
        del high_armed

        # L1 alone toggles the mode after a deliberate hold. A direction held
        # with L1 never enters this path.
        if buttons == BUTTON_L1:
            if self._l1_started_at is None:
                self._l1_started_at = now
            if (
                not self._l1_latched
                and now - self._l1_started_at >= self._config.mode.toggle_hold_s
            ):
                self.enabled = not self.enabled
                self._l1_latched = True
                events.append(ControllerEvent("mode", self.enabled))
        else:
            self._l1_started_at = None
            if not (buttons & BUTTON_L1):
                self._l1_latched = False

        # B is an emergency cancel for the voice channel. Treat it as a level,
        # not only a rising edge: if one 10 Hz packet was lost while B was
        # pressed, the next packet still stops the stream.
        if buttons & BUTTON_B:
            events.append(ControllerEvent("cancel"))

        if self.enabled and buttons != self._previous_buttons:
            for key, bit in BUTTON_BY_KEY.items():
                if buttons == (BUTTON_L1 | bit):
                    events.append(ControllerEvent("play", key))
                    break

        self._previous_buttons = buttons
        return events
