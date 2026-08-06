import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tiny_slam_ros2")
    rviz_config = os.path.join(share, "rviz", "tiny_slam.rviz")

    return LaunchDescription(
        [
            Node(
                package="tiny_slam_ros2",
                executable="tiny_slam_node",
                name="tiny_slam_node",
                output="screen",
                parameters=[
                    {
                        "lidar_beams": 24,
                        "update_period": 0.12,
                        "loop_demo": True,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
