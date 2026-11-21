"""Metric 2-D approach baseline, standard library only.

Coordinates: odom x/y in metres, yaw in radians (positive counterclockwise).
All stamps use one caller-supplied monotonic clock. Grid: rows along +y,
columns along +x; 0 free, 1 occupied, -1 unknown. Unknown/outside are blocked.
The footprint is conservatively bounded by a disk including step envelope.
Inputs must already be registered in odom; this module does not infer depth,
estimate odometry, identify objects or grant hardware authority.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
from functools import lru_cache
import math
from typing import Optional, Tuple

Point = Tuple[float, float]
Cell = Tuple[int, int]


@lru_cache(maxsize=16)
def blocked_rectangles(rows,resolution,origin):
    """Exact rectangle union of occupied/unknown cells, cached for static maps."""
    active={};rectangles=[];height=len(rows)
    for j in range(height+1):
        spans=set()
        if j<height:
            start=None
            for i,value in enumerate((*rows[j],0)):
                if value!=0 and start is None:start=i
                if value==0 and start is not None:spans.add((start,i));start=None
        for span in list(active):
            if span not in spans:
                first=active.pop(span);rectangles.append((origin[0]+span[0]*resolution,origin[1]+first*resolution,origin[0]+span[1]*resolution,origin[1]+j*resolution))
        for span in spans:
            if span not in active:active[span]=j
    return tuple(rectangles)


def finite(*values):
    return all(math.isfinite(v) for v in values)


@dataclass(frozen=True)
class Config:
    robot_radius_m: float
    margin_m: float
    standoff_m: float
    goal_tolerance_m: float
    max_age_s: float
    max_position_error_m: float
    max_v_mps: float
    max_w_radps: float
    max_accel_mps2: float
    brake_mps2: float
    reaction_s: float
    turn_threshold_rad: float
    heading_gain: float
    distance_gain: float
    command_ttl_s: float
    task_timeout_s: float
    allow_detour: bool
    cruise_until_goal: bool = False
    max_advance_m: Optional[float] = None

    def __post_init__(self):
        for name, value in vars(self).items():
            if name=='max_advance_m' and value is None:continue
            if name in ('allow_detour', 'cruise_until_goal'):
                if type(value) is not bool:
                    raise ValueError(name + ' must be boolean')
            elif not finite(value) or value <= 0:
                raise ValueError(name + ' must be finite and positive')
        if self.turn_threshold_rad >= math.pi / 2:
            raise ValueError('turn threshold must be less than pi/2')
        if self.standoff_m <= self.robot_radius_m + self.margin_m + self.goal_tolerance_m:
            raise ValueError('standoff must exceed footprint, margin and tolerance')


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float
    stamp: float
    error_m: float = 0.0


@dataclass(frozen=True)
class Target:
    identity: str
    x: float
    y: float
    stamp: float
    error_m: float = 0.0


@dataclass(frozen=True)
class Grid:
    rows: tuple
    resolution_m: float
    origin: Point
    stamp: float

    def __post_init__(self):
        object.__setattr__(self,'rows',tuple(tuple(row) for row in self.rows))
        object.__setattr__(self,'origin',tuple(self.origin))
        if (not self.rows or not self.rows[0] or
                not finite(self.resolution_m, *self.origin, self.stamp) or self.resolution_m <= 0):
            raise ValueError('invalid grid geometry')
        width = len(self.rows[0])
        if any(len(row) != width or any(v not in (-1, 0, 1) for v in row) for row in self.rows):
            raise ValueError('grid must be rectangular with -1/0/1 cells')
        object.__setattr__(self,'_rectangles',blocked_rectangles(self.rows,self.resolution_m,self.origin))

    def cell(self, p: Point) -> Cell:
        return (math.floor((p[0] - self.origin[0]) / self.resolution_m),
                math.floor((p[1] - self.origin[1]) / self.resolution_m))

    def center(self, c: Cell) -> Point:
        return (self.origin[0] + (c[0] + .5) * self.resolution_m,
                self.origin[1] + (c[1] + .5) * self.resolution_m)

    def clearance(self, p: Point) -> float:
        """Distance to occupied/unknown cell rectangles and map exterior."""
        x, y = p
        r = self.resolution_m
        ox, oy = self.origin
        result = min(x - ox, y - oy, ox + len(self.rows[0]) * r - x,
                     oy + len(self.rows) * r - y)
        for x0,y0,x1,y1 in self._rectangles:
            dx=max(x0-x,0.,x-x1);dy=max(y0-y,0.,y-y1)
            result=min(result,math.hypot(dx,dy))
        return result

    def blocked(self, radius: float):
        # Half-diagonal protects every point in an accepted cell, not just its centre.
        padding = radius + self.resolution_m / math.sqrt(2)
        return {(i, j) for j in range(len(self.rows)) for i in range(len(self.rows[0]))
                if self.clearance(self.center((i, j))) <= padding}


def plan(grid: Grid, start: Point, goal: Point, radius: float):
    """Four-neighbour A*: no diagonal corner cutting; [] means no known route."""
    blocked = grid.blocked(radius)
    source, dest = grid.cell(start), grid.cell(goal)
    width, height = len(grid.rows[0]), len(grid.rows)
    def valid(c):
        return 0 <= c[0] < width and 0 <= c[1] < height and c not in blocked
    if not valid(source) or not valid(dest):
        return []
    frontier = [(0, source)]
    cost, parent = {source: 0}, {}
    while frontier:
        _, cell = heapq.heappop(frontier)
        if cell == dest:
            cells = [cell]
            while cell != source:
                cell = parent[cell]
                cells.append(cell)
            points = [grid.center(c) for c in reversed(cells)]
            return [start] + points + [goal]
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            other = (cell[0] + dx, cell[1] + dy)
            proposed = cost[cell] + 1
            if valid(other) and proposed < cost.get(other, math.inf):
                parent[other], cost[other] = cell, proposed
                h = abs(other[0] - dest[0]) + abs(other[1] - dest[1])
                heapq.heappush(frontier, (proposed + h, other))
    return []


def segment_clear(grid, start, end, radius):
    # Clearance is 1-Lipschitz: half a sample spacing bounds between samples.
    distance = math.dist(start, end)
    count = max(1, math.ceil(distance / (grid.resolution_m / 4)))
    return all(grid.clearance((start[0] + (end[0]-start[0])*i/count,
                               start[1] + (end[1]-start[1])*i/count))
               > radius + distance/count/2 for i in range(count+1))


@dataclass(frozen=True)
class Command:
    state: str
    reason: str
    vx_mps: float
    wz_radps: float
    expires_at: float


class Approach:
    """One pinned task. Abort is latched; a new instance is an explicit new task.

    Output is a proposal only. STOP/ABORTED is a zero desired velocity, never
    a torque-off command or proof the physical robot has finished stopping.
    """
    def __init__(self, config: Config):
        self.config = config
        self.target: Optional[Target] = None
        self.goal = None
        self.path = []
        self.state = 'IDLE'
        self.reason = 'not_started'
        self.started = None
        self.last_time = None
        self.last_v = 0.

    def _stop(self, now, reason):
        self.state, self.reason, self.last_v = 'ABORTED', reason, 0.
        return Command(self.state, reason, 0., 0., now)

    def step(self, now, pose: Pose, target: Target, grid: Grid, *,
             metric_valid=False, authority=False, cancelled=False, estop=False):
        c = self.config
        if not finite(now):
            return self._stop(0., 'invalid_clock')
        if self.state in ('ABORTED', 'DONE'):
            return Command(self.state, self.reason, 0., 0., now)
        if cancelled or estop or not authority or not metric_valid:
            return self._stop(now, 'cancelled' if cancelled else 'estop' if estop else
                              'no_authority' if not authority else 'metric_unverified')
        if not finite(pose.x, pose.y, pose.yaw, pose.stamp, pose.error_m,
                      target.x, target.y, target.stamp, target.error_m):
            return self._stop(now, 'nonfinite_input')
        if any(not 0 <= now - stamp <= c.max_age_s for stamp in
               (pose.stamp, target.stamp, grid.stamp)):
            return self._stop(now, 'stale_or_future_input')
        if not target.identity or any(not 0 <= e <= c.max_position_error_m for e in
                                      (pose.error_m, target.error_m)):
            return self._stop(now, 'invalid_identity_or_uncertainty')
        if self.last_time is not None and now <= self.last_time:
            return self._stop(now, 'clock_not_increasing')
        if self.last_time is not None and now-self.last_time > c.command_ttl_s:
            return self._stop(now, 'control_deadline_missed')
        if self.started is not None and now - self.started > c.task_timeout_s:
            return self._stop(now, 'task_timeout')
        if self.target is None:
            self.target, self.started = target, now
            dx, dy = pose.x-target.x, pose.y-target.y
            distance = math.hypot(dx, dy)
            if distance < c.standoff_m - c.goal_tolerance_m:
                return self._stop(now, 'inside_standoff')
            if distance == 0:
                return self._stop(now, 'coincident_target')
            advance=max(0.,distance-c.standoff_m)
            if c.max_advance_m is not None:advance=min(advance,c.max_advance_m)
            self.goal = (pose.x-dx/distance*advance,pose.y-dy/distance*advance)
        elif target.identity != self.target.identity:
            return self._stop(now, 'target_id_changed')
        elif math.hypot(target.x-self.target.x, target.y-self.target.y) > c.max_position_error_m:
            return self._stop(now, 'target_moved_reconfirm')
        elif target.stamp < self.target.stamp:
            return self._stop(now, 'target_out_of_order')
        else:
            # Anchor stays fixed; observation freshness advances independently.
            self.target = Target(target.identity, self.target.x, self.target.y,
                                 target.stamp, target.error_m)
        position = (pose.x, pose.y)
        radius = c.robot_radius_m + c.margin_m + pose.error_m + target.error_m
        if grid.clearance(position) <= radius:
            return self._stop(now, 'footprint_blocked')
        if math.dist(position, self.goal) <= c.goal_tolerance_m:
            self.state, self.reason, self.last_v = 'DONE', 'goal_reached_kinematic', 0.
            return Command(self.state, self.reason, 0., 0., now)
        direct = segment_clear(grid, position, self.goal, radius)
        if not direct and not c.allow_detour:
            return self._stop(now, 'corridor_blocked')
        # Cache static-map planning; full equality catches map updates even with reused stamps.
        key = (grid.rows, grid.resolution_m, grid.origin, self.goal, radius)
        if direct:
            self.path = [self.goal]
            self._key = None
        elif not self.path or getattr(self, '_key', None) != key:
            self.path = plan(grid, position, self.goal, radius)[1:]
            self._key = key
        while len(self.path) > 1 and math.dist(position, self.path[0]) < grid.resolution_m*.25:
            self.path.pop(0)
        if not self.path or not segment_clear(grid, position, self.path[0], radius):
            return self._stop(now, 'no_known_path')
        dx, dy = self.path[0][0]-pose.x, self.path[0][1]-pose.y
        heading = math.atan2(dy, dx) - pose.yaw
        heading = math.atan2(math.sin(heading), math.cos(heading))
        wz = max(-c.max_w_radps, min(c.max_w_radps, c.heading_gain*heading))
        desired = min(c.max_v_mps, c.distance_gain*math.hypot(dx, dy))
        # Explicit gait-aware experiment: do not taper into a policy deadband.
        # Clearance/braking and acceleration still bound the proposal below.
        if c.cruise_until_goal:
            desired = c.max_v_mps
        if abs(heading) > c.turn_threshold_rad:
            desired = 0.
        # Isotropic stopping envelope, conservative during turns as well as forward motion.
        clearance = max(0., grid.clearance(position)-radius)
        delay = c.reaction_s + (now-grid.stamp) + c.command_ttl_s
        safe_v = -c.brake_mps2*delay + math.sqrt(
            (c.brake_mps2*delay)**2 + 2*c.brake_mps2*clearance)
        dt = 0. if self.last_time is None else min(now-self.last_time, c.command_ttl_s)
        vx = min(desired, safe_v, self.last_v+c.max_accel_mps2*dt)
        self.last_time, self.last_v = now, vx
        self.state = 'ALIGN' if desired == 0 else 'APPROACH'
        self.reason = 'proposed_only'
        return Command(self.state, self.reason, vx, wz, now+c.command_ttl_s)
