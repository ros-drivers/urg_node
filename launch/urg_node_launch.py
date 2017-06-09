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
def launch(descriptor, argv):
    descriptor.add_process(["urg_node"])

