"""ROS 2 wrapper around the tiny educational SLAM algorithm.

Published interfaces:
  /scan                 sensor_msgs/LaserScan
  /odom                 nav_msgs/Odometry
  /map                  nav_msgs/OccupancyGrid
  /slam_path            nav_msgs/Path
  /odom_path            nav_msgs/Path
  /robot_marker         visualization_msgs/Marker
  /tiny_slam/status     std_msgs/String

TF tree:
  map -> odom -> base_link -> laser_frame

Service:
  /tiny_slam/reset      std_srvs/Trigger
"""

from __future__ import annotations

from math import cos, pi, sin
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

from .slam_core import (
    Config,
    OccupancyGrid,
    World,
    make_path,
    move_pose,
    odometry_increment,
    rmse,
    scan_match,
    wrap_angle,
)


def yaw_quaternion(yaw: float) -> Quaternion:
    message = Quaternion()
    message.z = sin(yaw * 0.5)
    message.w = cos(yaw * 0.5)
    return message


def map_to_odom_pose(
    slam_pose: np.ndarray,
    odom_pose: np.ndarray,
) -> np.ndarray:
    """Find map->odom so map->odom * odom->base equals the SLAM pose."""
    angle = wrap_angle(float(slam_pose[2] - odom_pose[2]))
    rotated_odom_x = cos(angle) * odom_pose[0] - sin(angle) * odom_pose[1]
    rotated_odom_y = sin(angle) * odom_pose[0] + cos(angle) * odom_pose[1]
    return np.array(
        [
            slam_pose[0] - rotated_odom_x,
            slam_pose[1] - rotated_odom_y,
            angle,
        ],
        dtype=float,
    )


class TinySlamNode(Node):
    def __init__(self) -> None:
        super().__init__("tiny_slam_node")

        self.declare_parameter("lidar_beams", 24)
        self.declare_parameter("update_period", 0.12)
        self.declare_parameter("loop_demo", True)

        beams = int(self.get_parameter("lidar_beams").value)
        period = float(self.get_parameter("update_period").value)
        self.loop_demo = bool(self.get_parameter("loop_demo").value)

        self.cfg = Config(lidar_beams=max(8, beams))
        self.angles = np.linspace(
            -pi,
            pi,
            self.cfg.lidar_beams,
            endpoint=False,
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.map_pub = self.create_publisher(
            OccupancyGridMsg,
            "/map",
            map_qos,
        )
        self.slam_path_pub = self.create_publisher(
            Path,
            "/slam_path",
            10,
        )
        self.odom_path_pub = self.create_publisher(
            Path,
            "/odom_path",
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            "/robot_marker",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/tiny_slam/status",
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_service(Trigger, "/tiny_slam/reset", self.on_reset)

        self._publish_static_laser_transform()
        self.reset_state()
        self.timer = self.create_timer(period, self.step)

        self.get_logger().info(
            "Tiny SLAM started. Open RViz with fixed frame 'map'."
        )

    def reset_state(self) -> None:
        self.world = World(self.cfg)
        self.map_grid = OccupancyGrid(self.cfg)
        self.truth = make_path(self.cfg.path_step)
        self.rng = np.random.default_rng(self.cfg.seed)

        self.index = 0
        self.finished_ticks = 0

        self.odom_pose = self.truth[0].copy()
        self.slam_pose = self.truth[0].copy()

        self.odom_history: List[np.ndarray] = [self.odom_pose.copy()]
        self.slam_history: List[np.ndarray] = [self.slam_pose.copy()]

        self.current_scan = self.world.scan(
            self.truth[0],
            self.angles,
        )
        self.map_grid.update(
            self.slam_pose,
            self.current_scan,
            self.angles,
        )

        self.publish_all()

    def on_reset(self, request, response):
        self.reset_state()
        response.success = True
        response.message = "Tiny SLAM simulation reset"
        self.get_logger().info(response.message)
        return response

    def step(self) -> None:
        if self.index >= len(self.truth) - 1:
            self.finished_ticks += 1
            self.publish_all()

            if self.finished_ticks == 1:
                odom_error = rmse(
                    self.truth[: len(self.odom_history)],
                    np.asarray(self.odom_history),
                )
                slam_error = rmse(
                    self.truth[: len(self.slam_history)],
                    np.asarray(self.slam_history),
                )
                self.get_logger().info(
                    "Run completed | odometry RMSE %.3f m | "
                    "SLAM RMSE %.3f m"
                    % (odom_error, slam_error)
                )

            if self.loop_demo and self.finished_ticks >= 25:
                self.reset_state()
            return

        self.index += 1

        motion = odometry_increment(
            self.truth[self.index - 1],
            self.truth[self.index],
            self.rng,
        )

        self.odom_pose = move_pose(self.odom_pose, motion)
        predicted_slam_pose = move_pose(self.slam_pose, motion)

        self.current_scan = self.world.scan(
            self.truth[self.index],
            self.angles,
        )

        self.slam_pose = scan_match(
            predicted_slam_pose,
            self.current_scan,
            self.angles,
            self.map_grid,
            self.index,
        )

        self.map_grid.update(
            self.slam_pose,
            self.current_scan,
            self.angles,
        )

        self.odom_history.append(self.odom_pose.copy())
        self.slam_history.append(self.slam_pose.copy())

        self.publish_all()

    def publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()

        self._publish_tf(stamp)
        self._publish_scan(stamp)
        self._publish_odometry(stamp)
        self._publish_map(stamp)
        self._publish_paths(stamp)
        self._publish_marker(stamp)
        self._publish_status()

    def _publish_static_laser_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "laser_frame"
        transform.transform.translation.x = 0.10
        transform.transform.translation.z = 0.15
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _publish_tf(self, stamp) -> None:
        correction = map_to_odom_pose(
            self.slam_pose,
            self.odom_pose,
        )

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.x = float(correction[0])
        map_to_odom.transform.translation.y = float(correction[1])
        map_to_odom.transform.rotation = yaw_quaternion(
            float(correction[2])
        )

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.translation.x = float(self.odom_pose[0])
        odom_to_base.transform.translation.y = float(self.odom_pose[1])
        odom_to_base.transform.rotation = yaw_quaternion(
            float(self.odom_pose[2])
        )

        self.tf_broadcaster.sendTransform(
            [map_to_odom, odom_to_base]
        )

    def _publish_scan(self, stamp) -> None:
        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = "laser_frame"
        message.angle_min = -pi
        message.angle_max = pi
        message.angle_increment = 2.0 * pi / self.cfg.lidar_beams
        message.scan_time = float(
            self.get_parameter("update_period").value
        )
        message.range_min = self.cfg.ray_step
        message.range_max = self.cfg.max_range
        message.ranges = [
            float(value) for value in self.current_scan
        ]
        self.scan_pub.publish(message)

    def _publish_odometry(self, stamp) -> None:
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = float(self.odom_pose[0])
        message.pose.pose.position.y = float(self.odom_pose[1])
        message.pose.pose.orientation = yaw_quaternion(
            float(self.odom_pose[2])
        )
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.08
        self.odom_pub.publish(message)

    def _publish_map(self, stamp) -> None:
        probabilities = self.map_grid.probabilities()
        unknown = np.abs(self.map_grid.log_odds) < 0.05
        values = np.rint(probabilities * 100.0).astype(np.int16)
        values[unknown] = -1

        message = OccupancyGridMsg()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.info.resolution = self.cfg.resolution
        message.info.width = self.cfg.cells
        message.info.height = self.cfg.cells
        message.info.origin.orientation.w = 1.0
        message.data = values.reshape(-1).astype(int).tolist()
        self.map_pub.publish(message)

    @staticmethod
    def _pose_stamped(
        stamp,
        frame_id: str,
        pose: np.ndarray,
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.pose.position.x = float(pose[0])
        message.pose.position.y = float(pose[1])
        message.pose.orientation = yaw_quaternion(float(pose[2]))
        return message

    def _publish_paths(self, stamp) -> None:
        slam_path = Path()
        slam_path.header.stamp = stamp
        slam_path.header.frame_id = "map"
        slam_path.poses = [
            self._pose_stamped(stamp, "map", pose)
            for pose in self.slam_history
        ]
        self.slam_path_pub.publish(slam_path)

        odom_path = Path()
        odom_path.header.stamp = stamp
        odom_path.header.frame_id = "odom"
        odom_path.poses = [
            self._pose_stamped(stamp, "odom", pose)
            for pose in self.odom_history
        ]
        self.odom_path_pub.publish(odom_path)

    def _publish_marker(self, stamp) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "map"
        marker.ns = "tiny_slam"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.slam_pose[0])
        marker.pose.position.y = float(self.slam_pose[1])
        marker.pose.position.z = 0.05
        marker.pose.orientation = yaw_quaternion(
            float(self.slam_pose[2])
        )
        marker.scale.x = 0.45
        marker.scale.y = 0.16
        marker.scale.z = 0.12
        marker.color.r = 0.15
        marker.color.g = 0.75
        marker.color.b = 0.25
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def _publish_status(self) -> None:
        truth_used = self.truth[: len(self.slam_history)]
        odom_error = rmse(
            truth_used,
            np.asarray(self.odom_history),
        )
        slam_error = rmse(
            truth_used,
            np.asarray(self.slam_history),
        )

        status = String()
        status.data = (
            f"step={self.index}/{len(self.truth) - 1} "
            f"odom_rmse={odom_error:.3f}m "
            f"slam_rmse={slam_error:.3f}m"
        )
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TinySlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
