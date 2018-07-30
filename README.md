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
To build everything :
```
cd ~/ros2_ws
colcon build --symlink-install
```

To build urg_node only :
```
cd ~/ros2_ws
colcon build --packages-select urg_node
```

To build urg_node and all its dependencies : 
```
cd ~/ros2_ws
colcon build --packages-up-to urg_node
```

For more information about colcon [here](https://media.readthedocs.org/pdf/colcon/latest/colcon.pdf)


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


#### Parameters


Urg-node parameters:
```
arg : --serial-port
type : string
default value : "/dev/ttyACM0"
```

```
arg : --user-latency
type : double
default value (in seconds) : 0
```

```
arg : --ip-addr
type : string
default value : ""
```

```
arg : --port
type : int
default value : 0
```

```
arg : --laser-frame-id
type : string
default value : "laser"
```

```
arg : --angle-min
type : double
default value (in radians) : -3.14
```

```
arg : --angle-max
type : double
default value (in radians) : 3.14
```

To give parameters to urg_node :
```
ros2 run urg_node urg_node --ip-addr "192.168.0.10" --port 10940
```

#### How ot use the ust-20lx (and other ethernet based laser)

To use ust-20lx, you need to be on the same subnet as the laser.
The ust-20lx default ip is 192.168.0.10, so you might need to change your ip, for something on the same subnet. 

On Ubuntu :
- disable your wifi 
- go to the network settings 
- settings of the wired connection 
- under IPv4, change the IPv4 method from automatic to Manual and under Addresses, set the values :
```
    Address : 192.168.0.15
    Netmask : 255.255.255.0
    Gateway : 192.168.0.1
```
- Turn off and on the connection or wait a bit for the change to apply
- To check if it worked, open a terminal and type
```
ifconfig
```

- Under eth0 (or maybe something like enpxxxxx), your IP should be 192.168.0.15.
You should now be able to ping the ust-20lx at its address (by default 192.168.0.10)
```
ping 192.168.0.10
```
If you don't receive an answer, you might have a connection problem or the IP of your laser might have been change, either find it and go on the same subnet or reset it.

- Once you can ping the laser, you can launch the urg_node :
```
ros2 run urg_node urg_node --ip-addr "192.168.0.10" --port 10940 --angle-min -2.36 --angle-max 2.36
```

then the static publisher :
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 "world" "laser"
```

then rviz :
```
ros2 run rviz2 rviz2
```

Add the laserScan topic, change the frame on rviz from map to world and you should be able to see the laser. It seems that there is a bug on Rviz 2 so you might not be able to visualize Laserscan outside a 1 meter radius but the data are correctly published. You can follow the issue on github [here](https://github.com/ros2/rviz/issues/341)

