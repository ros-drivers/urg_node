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

///< @TODO Remove this and pass to the functions instead
boost::shared_ptr<urg_library_wrapper::URGLibraryWrapper> urg_;
boost::shared_ptr<dynamic_reconfigure::Server<urg_library_wrapper::URGConfig> > srv_; ///< Dynamic reconfigure server

bool reconfigure_callback(urg_library_wrapper::URGConfig& config, int level){
  if(level < 0){ // First call, initialize, laser not yet started
    urg_->setAngleLimitsAndCluster(config.angle_min, config.angle_max, config.cluster);
    urg_->setSkip(config.skip);
  } else if(level > 0){ // Must stop
    urg_->stop();
    ROS_INFO("Stopped data due to reconfigure.");
    
    // Change values that required stopping
    urg_->setAngleLimitsAndCluster(config.angle_min, config.angle_max, config.cluster);
    urg_->setSkip(config.skip);

    try{
      urg_->start();
      ROS_INFO("Streaming data after reconfigure.");
    } catch(std::runtime_error& e){
      ROS_FATAL("%s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ros::shutdown();
      return false;
    }
  }

  std::string frame_id = tf::resolve(config.tf_prefix, config.frame_id);
  urg_->setFrameId(frame_id);
  urg_->setUserLatency(config.time_offset);

  return true;
}

void update_reconfigure_limits(){
  urg_library_wrapper::URGConfig min, max;
  srv_->getConfigMin(min);
  srv_->getConfigMax(max);

  min.angle_min = urg_->getAngleMinLimit();
  min.angle_max = min.angle_min;
  max.angle_max = urg_->getAngleMaxLimit();
  max.angle_min = max.angle_max;
  
  srv_->setConfigMin(min);
  srv_->setConfigMax(max);
}

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_node");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 20);
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
  
  bool calibrate_time;
  pnh.param<bool>("calibrate_time", calibrate_time, false);

  bool publish_intensity;
  pnh.param<bool>("publish_intensity", publish_intensity, true);

  bool publish_multiecho; ///< @TODO Should we always get multiecho and publish through support library?
  pnh.param<bool>("publish_multiecho", publish_multiecho, true);
  
  // Set up the urgwidget
  try{
    if(ip_address != ""){
      urg_.reset(new urg_library_wrapper::URGLibraryWrapper(ip_address, ip_port, publish_intensity, publish_multiecho));
      ROS_INFO("Connected to network device with ID: %s", urg_->getDeviceID().c_str());
    } else {
      urg_.reset(new urg_library_wrapper::URGLibraryWrapper(serial_baud, serial_port, publish_intensity, publish_multiecho));
      ROS_INFO("Connected to serial device with ID: %s", urg_->getDeviceID().c_str());
    }
  } catch(std::runtime_error& e){
      ROS_FATAL("%s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ros::shutdown();
  }

  if(calibrate_time){
    try{
      ROS_INFO("Starting calibration. This will take a few seconds.");
      ROS_WARN("Time calibration is still experimental.");
      ros::Duration latency = urg_->computeLatency(10);
      ROS_INFO("Calibration finished. Latency is: %.4f.", latency.toSec());
    } catch(std::runtime_error& e){
      ROS_FATAL("Could not calibrate time offset:%s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ros::shutdown();
    }
  }

  // Set up dynamic reconfigure
  srv_.reset(new dynamic_reconfigure::Server<urg_library_wrapper::URGConfig>());

  // Configure limits (Must do this after creating the urgwidget)
  update_reconfigure_limits();

  dynamic_reconfigure::Server<urg_library_wrapper::URGConfig>::CallbackType f;
  f = boost::bind(reconfigure_callback, _1, _2);
  srv_->setCallback(f);

  try{
    urg_->start();
    ROS_INFO("Streaming data.");
  } catch(std::runtime_error& e){
    ROS_FATAL("%s", e.what());
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::shutdown();
  }

  while(ros::ok()){
    if(publish_multiecho){
      const sensor_msgs::MultiEchoLaserScanPtr msg(new sensor_msgs::MultiEchoLaserScan());
      if(urg_->grabScan(msg)){
        echoes_pub.publish(msg);
      } else {
        ROS_WARN("Could not grab multi echo scan.");
      }
    } else {
      const sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
      if(urg_->grabScan(msg)){
        laser_pub.publish(msg);
      } else {
        ROS_WARN("Could not grab single echo scan.");
      }
    }
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}