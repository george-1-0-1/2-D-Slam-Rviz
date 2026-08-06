from glob import glob
import os

from setuptools import find_packages, setup


package_name = "tiny_slam_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="George David",
    maintainer_email="george@example.com",
    description=(
        "Educational 2D LiDAR SLAM simulation exposed through ROS 2 topics, "
        "TF, messages, parameters and a reset service."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tiny_slam_node = tiny_slam_ros2.tiny_slam_node:main",
        ],
    },
)
