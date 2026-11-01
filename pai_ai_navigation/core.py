"""Pure, testable LiDAR processing, expert policy and command supervision.

Sector order: rear/right (-pi) through front (0) to rear/left (+pi).
Distances are measured from the lidar origin, not the chassis surface.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

SECTORS = 12
MAX_RANGE = 8.0
MAX_V = 0.20
MAX_W = 0.80
GOAL_TOLERANCE = 0.22
STOP_DISTANCE = 0.42
SLOW_DISTANCE = 0.95


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class ScanFeatures:
    distances: tuple[float, ...]
    valid: bool
    minimum: float
    front: float
    reason: str = "ok"


def process_scan(ranges: Sequence[float], angle_min: float, angle_increment: float,
                 range_min: float, range_max: float) -> ScanFeatures:
    """Conservative sector minima; +inf means no hit, NaN/missing coverage fails closed."""
    invalid = ScanFeatures((0.0,) * SECTORS, False, 0.0, 0.0, "invalid_scan")
    if (not ranges or not all(math.isfinite(v) for v in
            (angle_min, angle_increment, range_min, range_max))
            or angle_increment <= 0 or not 0 <= range_min < range_max):
        return invalid
    limit = min(range_max, MAX_RANGE)
    buckets: list[list[float]] = [[] for _ in range(SECTORS)]
    front: list[float] = []
    invalid_count = 0
    for i, raw in enumerate(ranges):
        angle = wrap(angle_min + i * angle_increment)
        if raw == math.inf:
            value = limit
        elif math.isfinite(raw) and range_min <= raw <= range_max:
            value = min(float(raw), limit)
        else:
            invalid_count += 1
            # An invalid beam contributes zero instead of silently clearing an obstacle.
            value = 0.0
        sector = min(SECTORS - 1, int((angle + math.pi) / (2 * math.pi) * SECTORS))
        buckets[sector].append(value)
        if abs(angle) <= math.radians(35):
            front.append(value)
    if any(not bucket for bucket in buckets) or not front:
        return invalid
    distances = tuple(min(bucket) for bucket in buckets)
    valid = invalid_count == 0
    return ScanFeatures(distances, valid, min(distances), min(front),
                        "ok" if valid else "invalid_beam")


def goal_features(x: float, y: float, yaw: float, goal: tuple[float, float]) -> tuple[float, float]:
    if not all(math.isfinite(v) for v in (x, y, yaw, *goal)):
        raise ValueError("Non-finite robot or goal pose")
    dx, dy = goal[0] - x, goal[1] - y
    return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw)


def feature_vector(scan: ScanFeatures, distance: float, angle: float) -> list[float]:
    values = [*scan.distances, min(distance, 15.0), angle]
    if not scan.valid or len(values) != 14 or not all(math.isfinite(v) for v in values):
        raise ValueError("Invalid policy input")
    return values


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0

    def bounded(self) -> Command:
        if not math.isfinite(self.linear) or not math.isfinite(self.angular):
            raise ValueError("Non-finite velocity")
        return Command(float(clamp(self.linear, 0.0, MAX_V)), float(clamp(self.angular, -MAX_W, MAX_W)))


def expert(scan: ScanFeatures, distance: float, angle: float) -> Command:
    """Local gap selection, goal attraction and gradual speed reduction.

    No map/planner is hidden in this policy: its inputs exactly match the MLP.
    It may get stuck in layouts with concave obstacles.
    """
    if not scan.valid or not all(math.isfinite(v) for v in (distance, angle)):
        return Command()
    if distance <= GOAL_TOLERANCE or scan.minimum < STOP_DISTANCE:
        return Command()
    def free_path(heading: float) -> float:
        # Inflate each observed obstacle by the chassis circumradius plus a
        # margin. Account for the half-sector angular uncertainty as well.
        clearance = 3.0
        radius = .55
        for i, value in enumerate(scan.distances):
            direction = -math.pi + (i + .5) * math.pi / 6
            difference = max(0.0, abs(wrap(direction - heading)) - math.pi / 12)
            if difference >= math.pi / 2:
                continue
            perpendicular = value * math.sin(difference)
            if perpendicular < radius:
                along = value * math.cos(difference)
                clearance = min(clearance, max(0.0, along - math.sqrt(radius ** 2 - perpendicular ** 2)))
        return clearance

    candidates: list[tuple[float, float]] = []
    for step in range(-6, 7):
        heading = step * math.pi / 12
        clearance = free_path(heading)
        score = 2.0 * math.cos(wrap(heading - angle)) + .50 * min(clearance, 2.0)
        if clearance < .65:
            score -= 3.0 + 4.0 * (.65 - clearance)
        candidates.append((score, heading))
    heading = max(candidates, key=lambda item: (round(item[0], 7), item[1]))[1]
    if abs(angle) > math.radians(105):
        heading = angle
    angular = clamp(1.5 * heading, -MAX_W, MAX_W)
    linear = MAX_V * clamp((free_path(0.0) - .15) / .6, 0.0, 1.0)
    linear *= max(0.0, math.cos(heading)) ** 2
    if abs(heading) > math.radians(60):
        linear = 0.0
    return Command(min(linear, .6 * distance), angular).bounded()


@dataclass(frozen=True)
class ShieldResult:
    command: Command
    reason: str
    intervened: bool


def supervise(command: Command, scan: ScanFeatures | None, scan_age: float,
              command_age: float, enabled: bool = True) -> ShieldResult:
    """Independent watchdog; emergency stops both translation and rotation.

    Thresholds are conservative demonstration limits, not certified safety.
    The emergency guard is retained even for expert demonstrations. AI-only
    evaluation disables geometric limits but still keeps the data watchdog.
    """
    try:
        limited = command.bounded()
    except ValueError:
        return ShieldResult(Command(), "invalid_command", True)
    if (scan is None or not scan.valid or not math.isfinite(scan_age)
            or not math.isfinite(command_age) or scan_age < 0 or command_age < 0
            or scan_age > .6 or command_age > .5):
        return ShieldResult(Command(), "stale_or_invalid_data", limited != Command())
    if not enabled:
        return ShieldResult(limited, "watchdog_only", limited != command)
    if scan.minimum < STOP_DISTANCE:
        return ShieldResult(Command(), "emergency_stop", limited != Command())
    linear = min(limited.linear, MAX_V * clamp((scan.front - .50) / (SLOW_DISTANCE - .50), 0.0, 1.0))
    result = Command(linear, limited.angular)
    return ShieldResult(result, "slow" if result != limited else "clear", result != command)