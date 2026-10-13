"""Video pacing policy, independent of motor/Quest deadman authority."""
from dataclasses import dataclass
import math


@dataclass
class AdaptiveVideoRate:
    # 0 selects automatic probing; a positive value is an explicit operator cap.
    configured_fps: float = 0.0
    fps: float = 12.0
    scale: float = 1.0
    healthy_windows: int = 0
    over_budget_windows: int = 0

    def __post_init__(self):
        if not math.isfinite(self.configured_fps) or self.configured_fps < 0:
            raise ValueError("configured_fps must be >=0")
        if self.configured_fps > 0:
            self.fps = min(self.fps, self.configured_fps)

    def update(self, *, capture_s: float, cycle_s: float, rtt_s: float,
               loss: float, sent_bps: float, budget_bps: float | None) -> str:
        values = (capture_s, cycle_s, rtt_s, loss, sent_bps)
        if not all(math.isfinite(v) and v >= 0 for v in values):
            return "invalid_feedback"
        source_capacity = 0.85 / max(capture_s, 0.001)
        cpu_capacity = 0.85 / max(cycle_s, 0.001)
        # Measured processing capacity bounds growth, rather than a fixed 6 FPS.
        ceiling = min(source_capacity, cpu_capacity)
        if self.configured_fps > 0:
            ceiling = min(ceiling, self.configured_fps)
        # REMB can start near the traffic rate on an otherwise idle network.
        # Do not interpret that as congestion below VP8's encoder bitrate floor,
        # or on a single burst (e.g. the initial keyframe).
        over_budget = (budget_bps is not None and math.isfinite(budget_bps)
                       and budget_bps >= 250000 and sent_bps > budget_bps * 1.2)
        self.over_budget_windows = self.over_budget_windows + 1 if over_budget else 0
        congested = loss > 0.03 or rtt_s > 0.15 or self.over_budget_windows >= 2
        if congested:
            self.healthy_windows = 0
            if self.scale > 0.5:
                self.scale = max(0.5, self.scale * 0.8)
                reason = "network_reduce_resolution"
            else:
                self.fps = max(1.0, self.fps * 0.8)
                reason = "network_reduce_fps"
        elif self.fps > ceiling * 1.15:
            self.fps = max(1.0, ceiling)
            self.healthy_windows = 0
            reason = "processing_capacity"
        else:
            self.healthy_windows += 1
            reason = "steady"
            if self.healthy_windows >= 3:
                if self.scale < 1.0:
                    self.scale = min(1.0, self.scale * 1.1)
                    reason = "recover_resolution"
                else:
                    self.fps = max(1.0, min(ceiling, self.fps * 1.1 + 1.0))
                    reason = "probe_fps"
                self.healthy_windows = 0
        if self.configured_fps > 0:
            self.fps = min(self.fps, self.configured_fps)
        return reason
