# voice_presets — phát câu thu sẵn (offline)

This service only reads the R3-1 packet mirrored by `hb_integration` and plays
pre-recorded local files through the existing R1 audio bridge. It never imports
or publishes a motor-control API and needs no Internet. Hold `L1` alone for
three seconds to toggle preset mode, then hold `L1` and press a direction to
play its preset. It works in default/Built-in handover and while high-level is
armed.

## Operator controls

- Hold `L1` alone for three seconds to toggle preset mode; a high/low tone
  confirms ON/OFF. This also works after `R1+R2` arms high-level.
- While mode is ON, hold `L1` and press `UP`, `RIGHT`, `DOWN`, or `LEFT` to play
  the matching slot. Direction keys by themselves do not start a preset. The
  modifier chord does not trigger the high-level bare-direction gesture.
- Another preset replaces the active clip. Repeating its chord after the
  configured 1.5-second window stops it; a quicker repeat is ignored.
- `B` remains an optional emergency stop for a mistaken selection.

Playback cancels on remote loss, high-level `BUSY`, a repeated chord after the
cancel window, or `B` (the optional emergency fallback).
The player retries a decoder/bridge failure from the beginning of the selected
clip; if its worker crashes, systemd resumes that pending clip after restart
unless one of those cancellation conditions occurred. Put audio in `assets/`
and edit only `config/presets.yaml` to replace a voice, alter volume, tune the
post-clip `playback.end_tail_s` (default 0.4 s, to let the R1 speaker drain its
last buffer), or disable a slot.
