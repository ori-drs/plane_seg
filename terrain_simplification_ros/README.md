# Description
See https://wiki.oxfordrobots.com/display/~oliwier/2021/01/07/Terrain+Simplification

# How to Run 
1. To launch the main ros node which computes the simplified terrain:
```
roslaunch terrain_simplification_ros terrain_simplification_ros.launch
```
2. To launch an auxiliry node which calls the gridmap publisher of the main node, a specified rate (default = 10 Hz):
```
roslaunch terrain_simplification_ros terrain_simplification_ros_service_caller.launch
```

# Parameters
To set the rate of the publisher:
```
rosparam set /terrain_simplification_ros_service_caller/rate 1
```

Some (most useful) ROS parameters (defaults are set in `config/config.yaml`):
```
/terrain_simplification_ros_node/map_res_scaling
/terrain_simplification_ros_node/gridmap_size_x
/terrain_simplification_ros_node/gridmap_size_y
/terrain_simplification_ros_node/h_nominal 
```
These can be set and re-read using:
```
rosservice call /terrain_simplification/read
```
