/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#include <ros/ros.h>

#include <tf/tf.h> // tf header for resolving tf prefix
#include <dynamic_reconfigure/server.h>
#include <urg_library_wrapper/URGConfig.h>

#include <urg_library_wrapper/urg_library_wrapper.h>

boost::shared_ptr<urg_library_wrapper::URGLibraryWrapper> urg_;

boost::shared_ptr<dynamic_reconfigure::Server<urg_library_wrapper::URGConfig> > srv_; ///< Dynamic reconfigure server

bool reconfigure_callback(urg_library_wrapper::URGConfig& config, uint32_t level){
  ROS_INFO("Reconfigure Callback");
  return true;
}

void update_reconfigure_limits(){
  ROS_INFO("Update reconfigure limits");
  urg_library_wrapper::URGConfig min, max;
  srv_->getConfigMin(min);
  srv_->getConfigMax(max);

  /// @TODO Figure out the minimum range between min and max
  min.angle_min = urg_->getMinAngle();
  min.angle_max = min.angle_min + 0.1;
  max.angle_max = urg_->getMaxAngle();
  max.angle_min = max.angle_max - 0.1;
  
  srv_->setConfigMin(min);
  srv_->setConfigMax(max);
}

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_node");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("first", 20);
  ros::Publisher echoes_pub = n.advertise<sensor_msgs::MultiEchoLaserScan>("echoes", 20);
  
  // Get parameters so we can change these later.
  std::string ip_address;
  pnh.param<std::string>("ip_address", ip_address, "");
  int ip_port;
  pnh.param<int>("ip_port", ip_port, 10940);

  std::string serial_port;
  pnh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
  int serial_baud;
  pnh.param<int>("serial_baud", serial_baud, 115200);

  std::string tf_prefix;
  pnh.param<std::string>("tf_prefix", tf_prefix, "");
  std::string frame_id;
  pnh.param<std::string>("frame_id", frame_id, "laser");
  frame_id = tf::resolve(tf_prefix, frame_id);

  bool calibrate_time;
  pnh.param<bool>("calibrate_time", calibrate_time, true);

  bool publish_intensity;
  pnh.param<bool>("publish_intensity", publish_intensity, true);
  
  // Set up the urgwidget
  try{
    if(ip_address != ""){
      ROS_INFO("Opening network Hokuyo");
      urg_.reset(new urg_library_wrapper::URGLibraryWrapper(ip_address, ip_port));
    } else {
      ROS_INFO("Opening serial Hokuyo");
      urg_.reset(new urg_library_wrapper::URGLibraryWrapper(serial_baud, serial_port));
    }
  } catch(std::runtime_error& e){
      ROS_FATAL("%s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ros::shutdown();
    }

  // Set up dynamic reconfigure
  srv_.reset(new dynamic_reconfigure::Server<urg_library_wrapper::URGConfig>());

  // Configure limits (Must do this after creating the urgwidget)
  update_reconfigure_limits();

  dynamic_reconfigure::Server<urg_library_wrapper::URGConfig>::CallbackType f;
  f = boost::bind(reconfigure_callback, _1, _2);
  srv_->setCallback(f);

  ros::spin();

  return EXIT_SUCCESS;
}