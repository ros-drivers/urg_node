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
#include <urg_node/URGConfig.h>

#include <urg_node/urg_c_wrapper.h>
#include <laser_proc/LaserTransport.h>

///< @TODO Remove this and pass to the functions instead
boost::shared_ptr<urg_node::URGCWrapper> urg_;
boost::shared_ptr<dynamic_reconfigure::Server<urg_node::URGConfig> > srv_; ///< Dynamic reconfigure server

bool reconfigure_callback(urg_node::URGConfig& config, int level){
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
  urg_node::URGConfig min, max;
  srv_->getConfigMin(min);
  srv_->getConfigMax(max);

  min.angle_min = urg_->getAngleMinLimit();
  min.angle_max = min.angle_min;
  max.angle_max = urg_->getAngleMaxLimit();
  max.angle_min = max.angle_max;
  
  srv_->setConfigMin(min);
  srv_->setConfigMax(max);
}

void calibrate_time_offset(){
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

int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  ros::init(argc, argv, "urg_node");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  
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

  bool publish_multiecho;
  pnh.param<bool>("publish_multiecho", publish_multiecho, true);

  int error_limit;
  pnh.param<int>("error_limit", error_limit, 4);
  
  // Set up the urgwidget
  try{
    if(ip_address != ""){
      urg_.reset(new urg_node::URGCWrapper(ip_address, ip_port, publish_intensity, publish_multiecho));
    } else {
      urg_.reset(new urg_node::URGCWrapper(serial_baud, serial_port, publish_intensity, publish_multiecho));
    }
  } catch(std::runtime_error& e){
      ROS_FATAL("%s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      ros::shutdown();
      return EXIT_FAILURE;
  }

  std::stringstream ss;
  ss << "Connected to";
  if(publish_multiecho){
    ss << " multiecho";
  }
  if(ip_address != ""){
    ss << " network";
  } else {
    ss << " serial";
  }
  ss << " device with";
  if(publish_intensity){
    ss << " intensity and";
  }
  ss << " ID: " << urg_->getDeviceID();
  ROS_INFO_STREAM(ss.str());

  // Set up publishers, we only need one
  ros::Publisher laser_pub;
  laser_proc::LaserPublisher echoes_pub;
  if(publish_multiecho){
    echoes_pub = laser_proc::LaserTransport::advertiseLaser(n, 20);
  } else {
    laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 20);
  }

  if(calibrate_time){
    calibrate_time_offset();
  }

  // Set up dynamic reconfigure
  srv_.reset(new dynamic_reconfigure::Server<urg_node::URGConfig>());

  // Configure limits (Must do this after creating the urgwidget)
  update_reconfigure_limits();

  dynamic_reconfigure::Server<urg_node::URGConfig>::CallbackType f;
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
    return EXIT_FAILURE;
  }

  int error_count = 0;
  while(ros::ok()){
    if(publish_multiecho){
      const sensor_msgs::MultiEchoLaserScanPtr msg(new sensor_msgs::MultiEchoLaserScan());
      if(urg_->grabScan(msg)){
        echoes_pub.publish(msg);
      } else {
        ROS_WARN("Could not grab multi echo scan.");
        error_count++;
      }
    } else {
      const sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
      if(urg_->grabScan(msg)){
        laser_pub.publish(msg);
      } else {
        ROS_WARN("Could not grab single echo scan.");
        error_count++;
      }
    }

    // Reestablish conneciton if things seem to have gone wrong.
    if(error_count > error_limit){
      error_count = 0;
      ROS_ERROR("Error count exceeded limit, reconnecting.");
      urg_->stop();
      ros::Duration(2.0).sleep();
      if(calibrate_time){
        calibrate_time_offset();
      }
      urg_->start();
    }
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}