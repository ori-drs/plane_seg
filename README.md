# Plane Seg

[![ANYmal climbing a real staircase](https://img.youtube.com/vi/YYs4lJ9t-Xo/0.jpg)](https://www.youtube.com/watch?v=YYs4lJ9t-Xo)

*Plane Seg* is a library for fitting planes to LIDAR, depth camera data or elevation maps. It uses robust estimation to fit planes by clustering planar points with similar normals.

The source code is released under a [BSD 3-Clause license](LICENSE).

# Authors

Originally Developed as part of the DARPA Robotics Challenge project at MIT. Subsequent improvements and conversion to ROS was carried out at the DRS Group in Oxford during the MEMMO EU H2020 Project.

**Maintainers: DRS Group at Oxford Robotics Institute<br />
Original Author: Matt Antone (then at MIT)**

# Input

Input should be a point cloud or elevation map in the robot's odometry frame as well as the pose of the robot
in the odometry frame. The elevation map is assumed to be at a 1-2 Hz.

# Output

A series of planar convex hulls published at 1-2 Hz

# Other Images

Segmenting the DARPA Robotics Challenge terrain:

![drc terrain segmentation][./drc_terrain.png]

![drc terrain photo][drc_terrain_photo.jpg]

ANYmal climbing a simulated staircase in Gazebo:

[![ANYmal climbing a simulated staircase in Gazebo](https://img.youtube.com/vi/oXMB14HaFns/0.jpg)](https://www.youtube.com/watch?v=oXMB14HaFns)

