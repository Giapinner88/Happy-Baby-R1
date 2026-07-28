"""Fail-closed PTT and speaker gate shared by the robot voice processors."""

from __future__ import annotations

import asyncio
import os
import socket
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace
from pathlib import Path

try:
    from loguru import logger
except ModuleNotFoundError:  # Allows dependency-free coordinator unit tests.
    import logging

    class _Logger:
        def info(self, message, *args):
            logging.getLogger("hb.audio_gate").info(message.format(*args))

        def warning(self, message, *args):
            logging.getLogger("hb.audio_gate").warning(message.format(*args))

        def exception(self, message, *args):
            logging.getLogger("hb.audio_gate").exception(message.format(*args))

    logger = _Logger()


@dataclass(frozen=True)
class GateSnapshot:
    high_alive: bool = False
    high_busy: bool = True
    high_armed: bool = False
    remote_alive: bool = False
    ptt: bool = False
    rearm: bool = False
    coordinator_mic: bool = False
    coordinator_speaker: bool = False
    voice_speaking: bool = False

    @property
    def coordinator_ready(self) -> bool:
        return self.high_alive and self.remote_alive

    @property
    def mic_allowed(self) -> bool:
        return self.coordinator_mic and not self.voice_speaking

    @property
    def speaker_allowed(self) -> bool:
        return self.coordinator_speaker


GateCallback = Callable[[GateSnapshot, GateSnapshot], Awaitable[None]]


class AudioGate:
    def __init__(
        self,
        socket_path: str | Path,
        stale_after_s: float = 0.6,
        *,
        activation_mode: str = "both",
        allow_during_startup: bool = False,
        startup_grace_s: float = 0.0,
    ):
        if activation_mode not in {"both", "high_disarmed", "high_armed"}:
            raise ValueError(f"unsupported activation mode: {activation_mode}")
        self._socket_path = Path(socket_path)
        self._stale_after_s = stale_after_s
        self._activation_mode = activation_mode
        self._allow_during_startup = allow_during_startup
        self._startup_grace_s = startup_grace_s
        self._startup_deadline = 0.0
        self._high_seen = False
        self._snapshot = GateSnapshot()
        self._callbacks: list[GateCallback] = []
        self._socket: socket.socket | None = None
        self._task: asyncio.Task | None = None
        self._last_message_at = 0.0

    @property
    def snapshot(self) -> GateSnapshot:
        return self._snapshot

    def add_callback(self, callback: GateCallback) -> GateCallback:
        self._callbacks.append(callback)
        return callback

    async def start(self) -> None:
        if self._task:
            return
        self._socket_path.parent.mkdir(parents=True, exist_ok=True)
        self._startup_deadline = time.monotonic() + self._startup_grace_s
        self._high_seen = False
        try:
            self._socket_path.unlink()
        except FileNotFoundError:
            pass
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.bind(str(self._socket_path))
        os.chmod(self._socket_path, 0o660)
        self._socket = sock
        self._task = asyncio.create_task(self._listen(), name="hb-audio-gate")
        logger.info(f"Audio gate listening on {self._socket_path} (fail-closed)")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
            self._task = None
        if self._socket:
            self._socket.close()
            self._socket = None
        try:
            self._socket_path.unlink()
        except FileNotFoundError:
            pass
        await self._set_snapshot(GateSnapshot())

    async def set_voice_speaking(self, speaking: bool) -> None:
        if speaking == self._snapshot.voice_speaking:
            return
        await self._set_snapshot(replace(self._snapshot, voice_speaking=speaking))

    async def _set_snapshot(self, new: GateSnapshot) -> None:
        old = self._snapshot
        if new == old:
            return
        self._snapshot = new
        if (
            old.high_alive,
            old.high_busy,
            old.high_armed,
            old.remote_alive,
            old.ptt,
            old.rearm,
            old.coordinator_mic,
            old.coordinator_speaker,
        ) != (
            new.high_alive,
            new.high_busy,
            new.high_armed,
            new.remote_alive,
            new.ptt,
            new.rearm,
            new.coordinator_mic,
            new.coordinator_speaker,
        ):
            logger.info(
                "Gate: high_alive={} high_busy={} high_armed={} remote_alive={} ptt={} "
                "rearm={} mic={} speaker={}",
                new.high_alive,
                new.high_busy,
                new.high_armed,
                new.remote_alive,
                new.ptt,
                new.rearm,
                new.mic_allowed,
                new.speaker_allowed,
            )
        for callback in tuple(self._callbacks):
            try:
                await callback(old, new)
            except Exception as error:
                logger.exception(f"Audio gate callback failed: {error}")

    def _parse(self, message: str, voice_speaking: bool) -> GateSnapshot:
        values = {}
        for token in message.split():
            if "=" in token:
                key, value = token.split("=", 1)
                values[key] = value

        def flag(name: str, default: bool = False) -> bool:
            return values.get(name, "1" if default else "0") == "1"

        if values.get("v") != "1":
            raise ValueError("unsupported gate protocol")
        raw = GateSnapshot(
            high_alive=flag("high_alive"),
            high_busy=flag("high_busy", True),
            high_armed=flag("high_armed"),
            remote_alive=flag("remote_alive"),
            ptt=flag("ptt"),
            rearm=flag("rearm"),
            coordinator_mic=flag("mic"),
            coordinator_speaker=flag("speaker"),
            voice_speaking=voice_speaking,
        )
        return self._apply_policy(raw)

    def _apply_policy(self, raw: GateSnapshot) -> GateSnapshot:
        if raw.high_alive:
            self._high_seen = True
            mode_allowed = (
                self._activation_mode == "both"
                or (self._activation_mode == "high_armed" and raw.high_armed)
                or (self._activation_mode == "high_disarmed" and not raw.high_armed)
            )
            return replace(
                raw,
                coordinator_mic=raw.coordinator_mic and mode_allowed,
                coordinator_speaker=raw.coordinator_speaker and mode_allowed,
            )

        startup_allowed = (
            self._allow_during_startup
            and not self._high_seen
            and time.monotonic() <= self._startup_deadline
            and raw.remote_alive
        )
        return replace(
            raw,
            coordinator_mic=(startup_allowed and raw.ptt and not raw.rearm),
            coordinator_speaker=startup_allowed,
        )

    async def _listen(self) -> None:
        assert self._socket is not None
        loop = asyncio.get_running_loop()
        while True:
            try:
                raw = await asyncio.wait_for(
                    loop.sock_recv(self._socket, 2048), timeout=0.2
                )
                self._last_message_at = time.monotonic()
                parsed = self._parse(
                    raw.decode("utf-8", errors="replace"), self._snapshot.voice_speaking
                )
                await self._set_snapshot(parsed)
            except asyncio.TimeoutError:
                if (
                    self._last_message_at
                    and time.monotonic() - self._last_message_at > self._stale_after_s
                ):
                    await self._set_snapshot(
                        GateSnapshot(voice_speaking=self._snapshot.voice_speaking)
                    )
                    self._last_message_at = 0.0
            except ValueError as error:
                logger.warning(f"Ignoring invalid gate datagram: {error}")
