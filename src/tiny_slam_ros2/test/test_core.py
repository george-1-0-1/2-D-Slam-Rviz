import numpy as np

from tiny_slam_ros2.slam_core import (
    Config,
    OccupancyGrid,
    World,
    bresenham,
    make_path,
    move_pose,
    odometry_increment,
    wrap_angle,
)


def test_bresenham_keeps_endpoints():
    line = bresenham((0, 0), (5, 3))
    assert line[0] == (0, 0)
    assert line[-1] == (5, 3)


def test_angle_wrap():
    assert -np.pi <= wrap_angle(5.0) <= np.pi


def test_virtual_scan_has_expected_size():
    cfg = Config(lidar_beams=12)
    world = World(cfg)
    scan = world.scan(
        np.array([1.0, 1.0, 0.0]),
        np.linspace(-np.pi, np.pi, 12, endpoint=False),
    )
    assert scan.shape == (12,)
    assert np.all(scan <= cfg.max_range)


def test_map_update_changes_cells():
    cfg = Config(lidar_beams=12)
    world = World(cfg)
    grid = OccupancyGrid(cfg)
    pose = np.array([1.0, 1.0, 0.0])
    angles = np.linspace(-np.pi, np.pi, 12, endpoint=False)
    scan = world.scan(pose, angles)
    grid.update(pose, scan, angles)
    assert np.any(grid.log_odds != 0.0)


def test_odometry_motion_runs():
    cfg = Config(path_step=0.3)
    path = make_path(cfg.path_step)
    rng = np.random.default_rng(cfg.seed)
    motion = odometry_increment(path[0], path[1], rng)
    moved = move_pose(path[0], motion)
    assert moved.shape == (3,)
