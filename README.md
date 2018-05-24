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

