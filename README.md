# Plane Seg

[![ANYmal climbing a real staircase](https://img.youtube.com/vi/YYs4lJ9t-Xo/0.jpg)](https://www.youtube.com/watch?v=YYs4lJ9t-Xo)

*Plane Seg* is a library for fitting planes to LIDAR, depth camera data or elevation maps. It uses robust estimation to fit planes by clustering planar points with similar normals.

We assume the ROS build system (e.g. Melodic). The core library (plane-seg) is only dependent on PCL. 

The ROS application (plane-seg-ros) can process PointCloud2 and GridMap data types. A sample application can be run and visualized (in ROS) as follows:

```python
roslaunch plane_seg_ros  anymal.launch
roslaunch plane_seg_ros  view_plane_seg.launch
```

# Input

Input should be a point cloud or elevation map in the robot's odometry frame as well as the pose of the robot
in the odometry frame. The elevation map is assumed to be at a 1-2 Hz.

# Output

A series of planar convex hulls published at 1-2 Hz

# Performance

* The DRC LIDAR terrain scan below is very dense (76000 points). It takes 2.5 seconds to process.
* The ANYmal RGB-D (RealSense D435) terrain scan is very dense (76000 points). It takes 2.5 seconds to process.

# Authors

Originally Developed as part of the DARPA Robotics Challenge project at MIT. Subsequent improvements and conversion to ROS was carried out at the DRS Group in Oxford during the MEMMO EU H2020 Project.

**Maintainers: DRS Group at Oxford Robotics Institute<br />
Original Author: Matt Antone (then at MIT)**

The source code is released under a [BSD 3-Clause license](LICENSE).

## Publications

If you use this work in an academic context, please use the following publication(s):

* M. Fallon, M. Antone,
**"Plane Seg – Robustly and Efficiently Extracting Contact Regions from Depth Data"**,


        @misc{Fallon2019PlaneSeg,
            author = {Fallon, Maurice and Antone, Matt},
            url = {https://github.com/ori-drs/plane_seg},
            title = {{Free Gait – An Architecture for the Versatile Control of Legged Robots}},
            year = {2019},
        }

# Other Images

Segmenting the DARPA Robotics Challenge terrain:

![drc terrain segmentation](drc_terrain.png)

Actual DRC Finals terrain (not quite the same scene):

![drc terrain photo](drc_terrain_photo.jpg)

Video of ANYmal climbing a simulated staircase in Gazebo:

[![ANYmal climbing a simulated staircase in Gazebo](https://img.youtube.com/vi/oXMB14HaFns/0.jpg)](https://www.youtube.com/watch?v=oXMB14HaFns)
