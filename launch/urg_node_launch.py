# <launch>
#  <node name="urg_node" pkg="urg_node" type="urg_node" output="screen">
#    <param name="ip_address" value=""/>
#    <param name="serial_port" value="/dev/ttyACM0"/>
#    <param name="serial_baud" value="115200"/>
#    <param name="frame_id" value="laser"/>
#    <param name="calibrate_time" value="true"/>
#    <param name="publish_intensity" value="false"/>
#    <param name="publish_multiecho" value="false"/>
#    <param name="angle_min" value="-1.5707963"/>
#    <param name="angle_max" value="1.5707963"/>
#  </node>
#</launch>

from ros2_launch_util import add_node

def launch(descriptor, argv):
    args = []

    # Handle the optional serial port argument
    name = "serial_port"
    if name in argv:
        args.extend(["--serial-port", argv[name]])

    # Handle the optional user latency argument
    name = "user_latency"
    if name in argv:
        args.extend(["--user-latency", argv[name]])

    add_node(descriptor, "urg_node", "urg_node", args=args)
