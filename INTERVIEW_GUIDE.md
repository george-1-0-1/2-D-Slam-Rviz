# What this project proves

## What the original NumPy project proved

- you understand LiDAR geometry
- you understand odometry drift
- you can implement scan matching
- you can build an occupancy grid
- you can evaluate trajectory error
- you can structure and test Python code

It did not prove ROS 2 experience.

## What this ROS wrapper adds

- ROS node lifecycle and timers
- topic publication
- standard message construction
- TF frame relationships
- ROS parameters
- a ROS service
- launch files
- package metadata and Colcon builds
- RViz visualisation

## Thirty-second answer

> The algorithm is intentionally small, but it is not only an animation. The
> core performs pose prediction, scan matching and occupancy-grid mapping. I then
> exposed that core as a ROS 2 package using LaserScan, Odometry, OccupancyGrid
> and Path messages. I implemented the map-to-odom correction in TF, added node
> parameters and a reset service, and visualised the live system in RViz.

## Do not claim

Do not say it is:

- Gazebo
- Nav2
- SLAM Toolbox
- a physical LiDAR
- production-ready autonomous navigation

## Strong progression statement

> I started at the algorithmic level to understand what SLAM is doing, then
> moved the same system into ROS 2. The next progression would be replacing the
> simulated sensor publishers with real drivers or Gazebo plugins without
> changing the message interfaces.
