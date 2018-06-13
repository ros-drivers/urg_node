urg_node
===================

ROS wrapper for the Hokuyo urg_c library.

## Build instructions (from source)
Install ros2 from source as per instructions [here](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)

Create a folder for urg and clone this repo

```
mkdir -p ~/ros2_ws/src/urg
cd ~/ros2_ws/src/urg
git clone -b ros2-devel https://github.com/bponsler/urg_node
```
Clone dependencies
```
cd ~/ros2_ws/src/urg/urg_node
vcs import ../ < urg.repos
```

Finally build

```
cd ~/ros2_ws
src/ament/ament_tools/scripts/ament.py build

```

#### LaserScan Visualization


Until the launch API is sorted out, there are two ways to view the laserscan in RViz:

1) use the static_transform_publisher tool from tf2_ros to publish a static transform for a fixed frame simultaneously with rviz and urg_node.

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "world" "laser"
```

where "laser" is the frame id, which you can set in your urg_node executable as an argument using `--laser-frame-id`

2) Run the robot_state_publisher along with a urdf file so that a fixed frame can be loaded in rviz

```
ros2 run robot_state_publisher robot_state_publisher <path to urdf file>
```

A urdf file is already included and if you have succesfully ran an `ament build`, the path to urdf file will be :

```
<path to ros2_ws>/install/share/urg_node/launch/hokuyo_laser.urdf
```

The issue with using the first way is that the rate of publishing is hardcoded to only once every 30 seconds in the code for static_transform_publisher, which is very very low. If you still want to use the first option you can increase the rate in [this line](https://github.com/ros2/geometry2/blob/b4615514aa82f1f10d5b927d3fd4723aa31632ff/tf2_ros/src/static_transform_broadcaster_program.cpp#L156) in your local copy of ros2 repo.
