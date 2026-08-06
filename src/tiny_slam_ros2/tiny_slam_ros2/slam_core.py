from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, pi
from typing import List, Tuple

import numpy as np


@dataclass
class Config:
    world_size: float = 10.0
    resolution: float = 0.10
    max_range: float = 4.5
    lidar_beams: int = 36
    ray_step: float = 0.05
    path_step: float = 0.15
    seed: int = 8

    @property
    def cells(self) -> int:
        return int(self.world_size / self.resolution)


def wrap_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def bresenham(a: Tuple[int, int], b: Tuple[int, int]) -> List[Tuple[int, int]]:
    x0, y0 = a
    x1, y1 = b
    points = []
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy

    while True:
        points.append((x0, y0))
        if (x0, y0) == (x1, y1):
            return points
        twice = 2 * error
        if twice >= dy:
            error += dy
            x0 += sx
        if twice <= dx:
            error += dx
            y0 += sy


class World:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.grid = np.zeros((cfg.cells, cfg.cells), dtype=np.uint8)
        self._make_world()

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(x / self.cfg.resolution), int(y / self.cfg.resolution)

    def occupied(self, x: float, y: float) -> bool:
        gx, gy = self.world_to_cell(x, y)
        if gx < 0 or gy < 0 or gx >= self.cfg.cells or gy >= self.cfg.cells:
            return True
        return bool(self.grid[gy, gx])

    def rectangle(self, x0: float, y0: float, x1: float, y1: float) -> None:
        gx0, gy0 = self.world_to_cell(x0, y0)
        gx1, gy1 = self.world_to_cell(x1, y1)
        self.grid[min(gy0, gy1): max(gy0, gy1) + 1,
                  min(gx0, gx1): max(gx0, gx1) + 1] = 1

    def _make_world(self) -> None:
        self.grid[:2, :] = 1
        self.grid[-2:, :] = 1
        self.grid[:, :2] = 1
        self.grid[:, -2:] = 1

        self.rectangle(2.8, 2.0, 3.2, 7.4)
        self.rectangle(3.0, 4.6, 6.0, 5.0)
        self.rectangle(6.6, 2.0, 7.0, 7.8)
        self.rectangle(4.4, 6.6, 5.4, 7.5)
        self.rectangle(7.6, 1.6, 8.6, 2.6)
        self.rectangle(1.1, 7.0, 2.0, 8.0)

    def scan(self, pose: np.ndarray, angles: np.ndarray) -> np.ndarray:
        x, y, heading = pose
        ranges = np.full(len(angles), self.cfg.max_range, dtype=float)

        for i, relative_angle in enumerate(angles):
            direction = heading + relative_angle
            distance = self.cfg.ray_step
            while distance <= self.cfg.max_range:
                px = x + cos(direction) * distance
                py = y + sin(direction) * distance
                if self.occupied(px, py):
                    ranges[i] = distance
                    break
                distance += self.cfg.ray_step
        return ranges


class OccupancyGrid:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.log_odds = np.zeros((cfg.cells, cfg.cells), dtype=float)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(x / self.cfg.resolution), int(y / self.cfg.resolution)

    def valid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.cfg.cells and 0 <= gy < self.cfg.cells

    def probabilities(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    def update(self, pose: np.ndarray, scan: np.ndarray, angles: np.ndarray) -> None:
        x, y, heading = pose
        start = self.world_to_cell(x, y)

        for distance, relative_angle in zip(scan, angles):
            end_x = x + cos(heading + relative_angle) * distance
            end_y = y + sin(heading + relative_angle) * distance
            end = self.world_to_cell(end_x, end_y)
            ray = bresenham(start, end)

            for gx, gy in ray[:-1]:
                if self.valid(gx, gy):
                    self.log_odds[gy, gx] -= 0.30

            hit = distance < self.cfg.max_range * 0.98
            if hit and ray:
                gx, gy = ray[-1]
                if self.valid(gx, gy):
                    self.log_odds[gy, gx] += 0.85

        np.clip(self.log_odds, -4.0, 4.0, out=self.log_odds)

    def score(self, pose: np.ndarray, scan: np.ndarray, angles: np.ndarray) -> float:
        """Higher score means LiDAR endpoints align with occupied map cells."""
        probabilities = self.probabilities()
        x, y, heading = pose
        total = 0.0
        count = 0

        for distance, relative_angle in zip(scan, angles):
            if distance >= self.cfg.max_range * 0.98:
                continue

            end_x = x + cos(heading + relative_angle) * distance
            end_y = y + sin(heading + relative_angle) * distance
            gx, gy = self.world_to_cell(end_x, end_y)

            if not self.valid(gx, gy):
                continue

            x0, x1 = max(0, gx - 1), min(self.cfg.cells, gx + 2)
            y0, y1 = max(0, gy - 1), min(self.cfg.cells, gy + 2)
            total += float(np.max(probabilities[y0:y1, x0:x1]))
            count += 1

        return total / count if count else 0.0


def make_path(step: float) -> np.ndarray:
    waypoints = [
        (0.9, 0.9),
        (9.0, 0.9),
        (9.0, 9.0),
        (0.9, 9.0),
        (0.9, 0.9),
    ]

    poses = []
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        dx, dy = x1 - x0, y1 - y0
        length = float(np.hypot(dx, dy))
        heading = float(np.arctan2(dy, dx))
        number = max(2, int(length / step))
        for t in np.linspace(0.0, 1.0, number, endpoint=False):
            poses.append([x0 + t * dx, y0 + t * dy, heading])
    poses.append([waypoints[-1][0], waypoints[-1][1], poses[-1][2]])
    return np.asarray(poses, dtype=float)


def odometry_increment(previous: np.ndarray, current: np.ndarray, rng) -> np.ndarray:
    distance = float(np.hypot(current[0] - previous[0], current[1] - previous[1]))
    turn = wrap_angle(float(current[2] - previous[2]))

    # Systematic bias plus random noise creates visible drift.
    measured_distance = distance * 1.008 + rng.normal(0.0, 0.004)
    measured_turn = turn + 0.0005 + rng.normal(0.0, 0.004)
    return np.array([measured_distance, measured_turn])


def move_pose(pose: np.ndarray, motion: np.ndarray) -> np.ndarray:
    distance, turn = motion
    heading = wrap_angle(float(pose[2] + turn))
    return np.array([
        pose[0] + cos(heading) * distance,
        pose[1] + sin(heading) * distance,
        heading,
    ])


def scan_match(
    prediction: np.ndarray,
    scan: np.ndarray,
    angles: np.ndarray,
    map_grid: OccupancyGrid,
    index: int,
) -> np.ndarray:
    """Search a small window around odometry and keep the best scan alignment."""
    if index < 12:
        return prediction.copy()

    best = prediction.copy()
    best_value = map_grid.score(prediction, scan, angles)

    for dx in np.linspace(-0.06, 0.06, 5):
        for dy in np.linspace(-0.06, 0.06, 5):
            for dtheta in np.linspace(-0.025, 0.025, 5):
                candidate = np.array([
                    prediction[0] + dx,
                    prediction[1] + dy,
                    wrap_angle(prediction[2] + dtheta),
                ])
                alignment = map_grid.score(candidate, scan, angles)
                movement_penalty = 0.05 * (
                    abs(dx) / 0.06 + abs(dy) / 0.06 + abs(dtheta) / 0.025
                )
                value = alignment - movement_penalty
                if value > best_value:
                    best = candidate
                    best_value = value
    return best


def run_slam(cfg: Config):
    rng = np.random.default_rng(cfg.seed)
    world = World(cfg)
    map_grid = OccupancyGrid(cfg)
    angles = np.linspace(-pi, pi, cfg.lidar_beams, endpoint=False)
    truth = make_path(cfg.path_step)

    odom_pose = truth[0].copy()
    slam_pose = truth[0].copy()

    odom_history = [odom_pose.copy()]
    slam_history = [slam_pose.copy()]
    scans = []

    first_scan = world.scan(truth[0], angles)
    map_grid.update(slam_pose, first_scan, angles)
    scans.append(first_scan)

    for index in range(1, len(truth)):
        motion = odometry_increment(truth[index - 1], truth[index], rng)
        odom_pose = move_pose(odom_pose, motion)
        prediction = move_pose(slam_pose, motion)

        scan = world.scan(truth[index], angles)
        slam_pose = scan_match(prediction, scan, angles, map_grid, index)
        map_grid.update(slam_pose, scan, angles)

        odom_history.append(odom_pose.copy())
        slam_history.append(slam_pose.copy())
        scans.append(scan)

    return {
        "world": world,
        "map": map_grid,
        "angles": angles,
        "truth": truth,
        "odom": np.asarray(odom_history),
        "slam": np.asarray(slam_history),
        "scans": scans,
    }


def rmse(truth: np.ndarray, estimate: np.ndarray) -> float:
    differences = truth[:, :2] - estimate[:, :2]
    return float(np.sqrt(np.mean(np.sum(differences * differences, axis=1))))
