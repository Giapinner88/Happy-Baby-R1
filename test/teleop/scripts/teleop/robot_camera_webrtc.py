#!/usr/bin/env python3
"""Read-only R1 camera media process. Signaling stays on robot loopback.

No motor transport is imported. Only a latest-frame mailbox is kept; idle
passthrough does not fetch/decode video. aiortc 1.14.0 is pinned because the
REMB hook below uses its internal RTCP dispatch (covered by a loopback test).
"""
from __future__ import annotations
import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
from fractions import Fraction
import hashlib
import io
import json
import math
from pathlib import Path
import time
import uuid

from aiohttp import web
import cv2
import numpy as np
from PIL import Image, ImageFile
from video_adaptation import AdaptiveVideoRate

ImageFile.LOAD_TRUNCATED_IMAGES = False


def decode_jpeg(payload: bytes, width: int, height: int) -> np.ndarray:
    # OpenCV often accepts broken JPEGs with a warning. Force a strict decode
    # first so such a frame never reaches the media encoder.
    if not payload.startswith(b"\xff\xd8") or not payload.rstrip(b"\x00").endswith(b"\xff\xd9"):
        raise ValueError("videohub JPEG is incomplete")
    with Image.open(io.BytesIO(payload)) as image:
        image.load()
        rgb = np.asarray(image.convert("RGB"))
    return cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)


class CameraSource:
    def __init__(self, args):
        self.args = args
        self.active = False
        self.generation = 0
        self.client = None
        self.latest = None
        self.received_at = 0.0
        self.sequence = 0
        self.capture_s = 0.05
        self.error = None
        self.last_error = None
        self.last_error_log_at = 0.0
        self.on_error = None
        self.invalid_frames = 0
        self.duplicate_frames = 0
        self.last_digest = None
        self.policy = AdaptiveVideoRate(args.capture_hz)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera-capture")
        self.task = None

    def read(self):
        if self.client is None:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.video.video_client import VideoClient
            ChannelFactoryInitialize(0, self.args.interface)
            self.client = VideoClient()
            self.client.SetTimeout(0.3)
            self.client.Init()
        code, payload = self.client.GetImageSample()
        if int(code) != 0 or not payload:
            raise RuntimeError(f"videohub returned {code}")
        raw = bytes(payload)
        rgb = decode_jpeg(raw, self.args.width, self.args.height)
        return rgb, hashlib.blake2s(raw, digest_size=8).digest()

    async def capture(self):
        loop = asyncio.get_running_loop()
        while True:
            if not self.active:
                await asyncio.sleep(0.05)
                continue
            generation = self.generation
            started = time.monotonic()
            try:
                rgb, digest = await loop.run_in_executor(self.executor, self.read)
                elapsed = time.monotonic() - started
                self.capture_s = 0.8 * self.capture_s + 0.2 * elapsed
                if self.active and generation == self.generation:
                    if digest == self.last_digest:
                        self.duplicate_frames += 1
                    self.last_digest = digest
                    self.latest = rgb
                    self.sequence += 1
                    self.received_at = time.monotonic()
                    self.error = None
            except Exception as exc:
                self.invalid_frames += 1
                self.error = f"{type(exc).__name__}: {exc}"
                self.last_error = {"error": self.error, "monotonic_s": time.monotonic()}
                if self.on_error and time.monotonic() - self.last_error_log_at > 1.0:
                    self.last_error_log_at = time.monotonic()
                    self.on_error(error=self.error, invalid_frames=self.invalid_frames)
            await asyncio.sleep(max(0.001, 1.0 / self.policy.fps - (time.monotonic() - started)))

    def set_active(self, active):
        if active != self.active:
            self.generation += 1
            self.latest = None
            self.received_at = 0.0
        self.active = active

    def snapshot(self):
        return {"enabled": self.active, "frame_sequence": self.sequence,
                "frame_age_s": None if self.latest is None else time.monotonic() - self.received_at,
                "error": self.error, "last_capture_error": self.last_error, "invalid_frames": self.invalid_frames,
                "duplicate_frames": self.duplicate_frames,
                "target_fps": self.policy.fps, "resolution_scale": self.policy.scale,
                "capture_ms": 1000 * self.capture_s}

    async def close(self):
        if self.task:
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        self.executor.shutdown(wait=False, cancel_futures=True)


def make_app(args, source=None):
    from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription, RTCRtpSender, VideoStreamTrack
    from aiortc.mediastreams import MediaStreamError
    from aiortc.rtp import RtcpPsfbPacket, RTCP_PSFB_APP, unpack_remb_fci
    from av import VideoFrame

    source = source or CameraSource(args)
    peers = {}  # id -> (peer, track, monitor)
    log_path = Path(args.log) if args.log else None

    def log(event, **values):
        record = {"event": event, "monotonic_s": time.monotonic(), **values}
        print("[WEBRTC] " + json.dumps(record), flush=True)
        if log_path:
            with log_path.open("a") as f:
                f.write(json.dumps(record) + "\n")

    source.on_error = lambda **values: log("capture_error", **values)

    class Track(VideoStreamTrack):
        def __init__(self):
            super().__init__()
            self.last_sequence = -1
            self.start_at = time.monotonic()
            self.last_return_at = None
            self.cycle_s = 0.03
            self.frames = 0
            self.last_shape = None

        async def recv(self):
            entered_at = time.monotonic()
            if self.last_return_at is not None:
                self.cycle_s = 0.8 * self.cycle_s + 0.2 * max(0.001, entered_at - self.last_return_at)
            while self.readyState == "live":
                if not source.active:
                    raise MediaStreamError
                age = time.monotonic() - source.received_at
                if source.latest is not None and age <= args.stale_s and source.sequence != self.last_sequence:
                    now = time.monotonic()
                    image = source.latest
                    scale = source.policy.scale
                    if scale < 0.99:
                        w = max(2, int(args.width * scale) // 2 * 2)
                        h = max(2, int(args.height * scale) // 2 * 2)
                        image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
                    frame = VideoFrame.from_ndarray(image, format="rgb24")
                    frame.pts = int((now - self.start_at) * 90000)
                    frame.time_base = Fraction(1, 90000)
                    self.last_sequence = source.sequence
                    self.frames += 1
                    self.last_return_at = time.monotonic()
                    return frame
                await asyncio.sleep(0.005)
            raise MediaStreamError

    async def close_peer(peer_id):
        entry = peers.pop(peer_id, None)
        if entry:
            pc, track, monitor = entry
            track.stop()
            if monitor and monitor is not asyncio.current_task():
                monitor.cancel()
                await asyncio.gather(monitor, return_exceptions=True)
            await pc.close()
            log("peer_closed", peer=peer_id)

    async def monitor_peer(peer_id, pc, sender, track, feedback):
        previous_bytes = 0
        previous_at = time.monotonic()
        try:
            while True:
                await asyncio.sleep(1)
                now = time.monotonic()
                if not source.active or (pc.connectionState != "connected" and now - track.start_at > 8):
                    break
                if source.latest is None or now - source.received_at > args.stale_s:
                    if now - track.start_at > args.stale_s + 3:
                        break
                if pc.connectionState != "connected":
                    continue
                stats = await sender.getStats()
                sent = 0
                rtt, loss = 0.0, 0.0
                for stat in stats.values():
                    if stat.type == "outbound-rtp":
                        sent = stat.bytesSent
                    elif stat.type == "remote-inbound-rtp":
                        rtt = stat.roundTripTime or 0.0
                        loss = stat.fractionLost / 256.0
                bps = 8 * max(0, sent - previous_bytes) / (now - previous_at)
                previous_bytes, previous_at = sent, now
                reason = source.policy.update(capture_s=source.capture_s, cycle_s=track.cycle_s,
                    rtt_s=rtt, loss=loss, sent_bps=bps, budget_bps=feedback[0])
                log("video_feedback", peer=peer_id, frames=track.frames, sent_bps=bps,
                    estimated_bps=feedback[0], rtt_ms=rtt * 1000, loss=loss,
                    adaptation=reason, **source.snapshot())
        finally:
            await close_peer(peer_id)

    async def offer(request):
        if not source.active:
            raise web.HTTPConflict(text="Robot camera is not enabled")
        params = await request.json()
        if params.get("type") != "offer" or not isinstance(params.get("sdp"), str):
            raise web.HTTPBadRequest(text="Expected SDP offer")
        # Single operator: close obsolete sessions from previous A presses.
        for peer_id in list(peers):
            await close_peer(peer_id)
        pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        track = Track()
        peer_id = uuid.uuid4().hex
        peers[peer_id] = (pc, track, None)
        try:
            await pc.setRemoteDescription(RTCSessionDescription(**params))
            sender = pc.addTrack(track)
            # VP8's aiortc encoder supports REMB-driven target bitrate changes.
            for transceiver in pc.getTransceivers():
                if transceiver.kind == "video":
                    codecs = [c for c in RTCRtpSender.getCapabilities("video").codecs
                              if c.mimeType.lower() == "video/vp8"]
                    transceiver.setCodecPreferences(codecs)
            feedback = [None]
            original = sender._handle_rtcp_packet

            async def handle_rtcp(packet):
                if isinstance(packet, RtcpPsfbPacket) and packet.fmt == RTCP_PSFB_APP:
                    try:
                        bitrate, ssrcs = unpack_remb_fci(packet.fci)
                        if sender._ssrc in ssrcs:
                            feedback[0] = float(bitrate)
                    except ValueError:
                        pass
                await original(packet)

            sender._handle_rtcp_packet = handle_rtcp
            await pc.setLocalDescription(await pc.createAnswer())
            monitor = asyncio.create_task(monitor_peer(peer_id, pc, sender, track, feedback))
            peers[peer_id] = (pc, track, monitor)
            log("offer_answered", peer=peer_id, codec="VP8", ice="local_only")
            return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})
        except BaseException:
            await close_peer(peer_id)
            raise

    async def view(request):
        payload = await request.json()
        if type(payload.get("enabled")) is not bool:
            raise web.HTTPBadRequest(text="enabled must be boolean")
        source.set_active(payload["enabled"])
        if not source.active:
            for peer_id in list(peers):
                await close_peer(peer_id)
        log("view", enabled=source.active)
        return web.json_response(source.snapshot())

    async def health(request):
        return web.json_response({"transport": "webrtc", "peers": len(peers),
            "connected_peers": sum(pc.connectionState == "connected" for pc, _, _ in peers.values()),
            **source.snapshot()})

    async def start(app):
        source.task = asyncio.create_task(source.capture())

    async def cleanup(app):
        source.set_active(False)
        for peer_id in list(peers):
            await close_peer(peer_id)
        await source.close()

    app = web.Application(client_max_size=256 * 1024)
    app.router.add_post("/offer", offer)
    app.router.add_post("/view", view)
    app.router.add_get("/health", health)
    app.on_startup.append(start)
    app.on_cleanup.append(cleanup)
    return app


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=8013)
    p.add_argument("--interface", default="eth10")
    p.add_argument("--host-ip", required=True)
    p.add_argument("--capture-hz", type=float, default=0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    p.add_argument("--stale-s", type=float, default=1)
    p.add_argument("--log")
    args = p.parse_args()
    if not math.isfinite(args.capture_hz) or args.capture_hz < 0 or min(args.width, args.height) < 2:
        p.error("capture-hz must be >=0 (0=auto); dimensions must be >=2")
    # Advertise only the Quest-facing interface, not internal DDS or Tailscale.
    import aioice.ice
    aioice.ice.get_host_addresses = lambda use_ipv4, use_ipv6: [args.host_ip] if use_ipv4 else []
    cv2.setNumThreads(1)
    web.run_app(make_app(args), host="127.0.0.1", port=args.port)


if __name__ == "__main__":
    main()
