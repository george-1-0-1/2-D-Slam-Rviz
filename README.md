# Tiny SLAM ROS 2

This is the small 2D SLAM algorithm wrapped as one real ROS 2 Humble package.

It is intentionally much smaller than a Gazebo or Nav2 project. The algorithm
still uses a lightweight simulated world, but its inputs, outputs and frame
relationships are exposed through standard ROS 2 interfaces.

## What this demonstrates

### Algorithmic work

- 2D LiDAR ray simulation
- noisy odometry
- local correlative scan matching
- occupancy-grid mapping
- trajectory-error measurement

### ROS 2 work

- an `rclpy` node
- publishers
- a service
- parameters
- standard ROS messages
- TF transforms
- an RViz configuration
- a launch file
- an `ament_python` package
- a Colcon workspace

## Published topics

```text
/scan                 sensor_msgs/msg/LaserScan
/odom                 nav_msgs/msg/Odometry
/map                  nav_msgs/msg/OccupancyGrid
/slam_path            nav_msgs/msg/Path
/odom_path            nav_msgs/msg/Path
/robot_marker         visualization_msgs/msg/Marker
/tiny_slam/status     std_msgs/msg/String
```

## TF tree

```text
map
 └── odom
      └── base_link
           └── laser_frame
```

`odom -> base_link` contains the drifting odometry estimate.

`map -> odom` contains the correction calculated from the SLAM pose.

## Run on Ubuntu 22.04 with ROS 2 Humble

Check that ROS and RViz are available:

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
command -v rviz2
python3 -c "import numpy; print(numpy.__version__)"
```

Build:

```bash
cd ~/tiny_slam_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Launch:

```bash
ros2 launch tiny_slam_ros2 demo.launch.py
```

RViz should show:

- the occupancy map growing
- the current LiDAR points
- the green corrected SLAM path
- the orange noisy odometry path
- the robot arrow
- the TF frames

## Inspect the ROS system

Open another terminal:

```bash
cd ~/tiny_slam_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

List nodes and topics:

```bash
ros2 node list
ros2 topic list
```

Read the status:

```bash
ros2 topic echo /tiny_slam/status
```

Check the LiDAR rate:

```bash
ros2 topic hz /scan
```

Inspect the map message:

```bash
ros2 topic echo /map --once
```

Inspect the node's ROS interfaces:

```bash
ros2 node info /tiny_slam_node
```

Reset the simulation through a service:

```bash
ros2 service call /tiny_slam/reset std_srvs/srv/Trigger "{}"
```

Change ROS parameters from the launch command:

```bash
ros2 run tiny_slam_ros2 tiny_slam_node \
  --ros-args \
  -p lidar_beams:=18 \
  -p update_period:=0.18
```

## Tests

```bash
cd ~/tiny_slam_ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=src/tiny_slam_ros2 \
python3 -m pytest src/tiny_slam_ros2/test -q
```

## What to say in the interview

> I first implemented the SLAM loop independently in Python so I could understand
> scan matching and occupancy mapping. I then separated the algorithmic core from
> the middleware layer and wrapped it as a ROS 2 node. The node publishes a
> standard LaserScan, Odometry, OccupancyGrid and Path, maintains the
> map-to-odom-to-base-link TF chain, exposes configurable parameters and a reset
> service, and includes an RViz and launch configuration.

## Honest limitations

The robot and LiDAR are still simulated in Python. This package demonstrates
ROS 2 integration and SLAM fundamentals, not Gazebo physics or a production
SLAM implementation.

The next hardware-facing step would be to replace the simulated `/scan` and
`/odom` data with RPLIDAR and encoder/IMU drivers while keeping the mapping,
TF, visualisation and launch structure.
