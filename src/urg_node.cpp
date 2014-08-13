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
 * Author: Chad Rockey, Michael Carroll
 */

#include <ros/ros.h>

#include <tf/tf.h> // tf header for resolving tf prefix
#include <dynamic_reconfigure/server.h>
#include <urg_node/URGConfig.h>

#include <urg_node/urg_c_wrapper.h>
#include <laser_proc/LaserTransport.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

///< @TODO Remove this and pass to the functions instead
boost::shared_ptr<urg_node::URGCWrapper> urg_;
boost::shared_ptr<dynamic_reconfigure::Server<urg_node::URGConfig> > srv_; ///< Dynamic reconfigure server

// Useful typedefs
typedef diagnostic_updater::FrequencyStatusParam FrequencyStatusParam;
typedef diagnostic_updater::HeaderlessTopicDiagnostic TopicDiagnostic;
typedef boost::shared_ptr<TopicDiagnostic> TopicDiagnosticPtr;

boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
TopicDiagnosticPtr laser_freq_, echoes_freq_;

bool close_diagnostics_;
boost::thread diagnostics_thread_;

/* Non-const device properties.  If you poll the driver for these
 * while scanning is running, then the scan will probably fail.
 */
std::string device_status_;
std::string vendor_name_;
std::string product_name_;
std::string firmware_version_;
std::string firmware_date_;
std::string protocol_version_;
std::string device_id_;

int error_count_;
double freq_min_;

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

  // The publish frequency changes based on the number of skipped scans.
  // Update accordingly here.
  freq_min_ = 1.0/(urg_->getScanPeriod() * (config.skip + 1));

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


// Diagnostics update task to be run in a thread.
void updateDiagnostics() {
    while(!close_diagnostics_) {
        diagnostic_updater_->update();
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }
}

// Populate a diagnostics status message.
void populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    if(urg_->getIPAddress() != "")
    {
        stat.add("IP Address", urg_->getIPAddress());
        stat.add("IP Port", urg_->getIPPort());
    } else {
        stat.add("Serial Port", urg_->getSerialPort());
        stat.add("Serial Baud", urg_->getSerialBaud());
    }

    if(!urg_->isStarted())
    {
      stat.summary(2, "Not Connected: " + device_status_);
    }
    else if(device_status_ != std::string("Sensor works well.") &&
            device_status_ != std::string("Stable 000 no error.") &&
            device_status_ != std::string("sensor is working normally"))
    {
      stat.summary(2, "Abnormal status: " + device_status_);
    }
    else
    {
      stat.summary(0, "Streaming");
    }

    stat.add("Vendor Name", vendor_name_);
    stat.add("Product Name", product_name_);
    stat.add("Firmware Version", firmware_version_);
    stat.add("Firmware Date", firmware_date_);
    stat.add("Protocol Version", protocol_version_);
    stat.add("Device ID", device_id_);
    stat.add("Computed Latency", urg_->getComputedLatency());
    stat.add("User Time Offset", urg_->getUserTimeOffset());

    // Things not explicitly required by REP-0138, but still interesting.
    stat.add("Device Status", device_status_);
    stat.add("Error Count", error_count_);
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

  double diagnostics_tolerance;
  pnh.param<double>("diagnostics_tolerance", diagnostics_tolerance, 0.05);
  double diagnostics_window_time;
  pnh.param<double>("diagnostics_window_time", diagnostics_window_time, 5.0);

  // Set up publishers and diagnostics updaters, we only need one
  ros::Publisher laser_pub;
  laser_proc::LaserPublisher echoes_pub;

  close_diagnostics_ = true;

  // Set up the urgwidget
  while(ros::ok()){
    try{
    	// Stop diagnostics
    	if(!close_diagnostics_){
	      close_diagnostics_ = true;
	      diagnostics_thread_.join();
      }
    	urg_.reset(); // Clear any previous connections();
    	ros::Duration(1.0).sleep();
      if(ip_address != ""){
        urg_.reset(new urg_node::URGCWrapper(ip_address, ip_port, publish_intensity, publish_multiecho));
      } else {
        urg_.reset(new urg_node::URGCWrapper(serial_baud, serial_port, publish_intensity, publish_multiecho));
      }
    } catch(std::runtime_error& e){
      ROS_ERROR_THROTTLE(10.0, "Error connecting to Hokuyo: %s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      continue; // Return to top of master loop
    } catch(...){
      ROS_ERROR_THROTTLE(10.0, "Unknown error connecting to Hokuyo");
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      continue; // Return to top of master loop
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

	  // Set up publishers and diagnostics updaters, we only need one
	  if(publish_multiecho){
	  	if(!echoes_pub){
	    	echoes_pub = laser_proc::LaserTransport::advertiseLaser(n, 20);
	    }
	  } else {
	  	if(!laser_pub){
	    	laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 20);
	    }
	  }

	  device_status_ = urg_->getSensorStatus();
	  vendor_name_ = urg_->getVendorName();
	  product_name_ = urg_->getProductName();
	  firmware_version_ = urg_->getFirmwareVersion();
	  firmware_date_ = urg_->getFirmwareDate();
	  protocol_version_ = urg_->getProtocolVersion();
	  device_id_ = urg_->getDeviceID();

	  diagnostic_updater_.reset(new diagnostic_updater::Updater);
	  diagnostic_updater_->setHardwareID(urg_->getDeviceID());
	  diagnostic_updater_->add("Hardware Status", populateDiagnosticsStatus);
		close_diagnostics_ = true;
	  if(publish_multiecho){
	    echoes_freq_.reset(new TopicDiagnostic("Laser Echoes", *diagnostic_updater_,
	                 FrequencyStatusParam(&freq_min_, &freq_min_, diagnostics_tolerance, diagnostics_window_time)));
	  } else {
	    laser_freq_.reset(new TopicDiagnostic("Laser Scan", *diagnostic_updater_,
	                FrequencyStatusParam(&freq_min_, &freq_min_, diagnostics_tolerance, diagnostics_window_time)));
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

	  // Start the urgwidget
    try{
      urg_->start();
      ROS_INFO("Streaming data.");
    } catch(std::runtime_error& e){
      ROS_ERROR_THROTTLE(10.0, "Error starting Hokuyo: %s", e.what());
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      continue; // Return to top of main loop
    } catch(...){
      ROS_ERROR_THROTTLE(10.0, "Unknown error starting Hokuyo");
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      continue; // Return to top of main loop
    }

	  // Now that we are streaming, kick off diagnostics.
	  close_diagnostics_ = false;
	  diagnostics_thread_ = boost::thread(updateDiagnostics);

	  // Clear the error count.
	  error_count_ = 0;
	  while(ros::ok()){
	  	try{
		    if(publish_multiecho){
		      const sensor_msgs::MultiEchoLaserScanPtr msg(new sensor_msgs::MultiEchoLaserScan());
		      if(urg_->grabScan(msg)){
		        echoes_pub.publish(msg);
		        echoes_freq_->tick();
		      } else {
		        ROS_WARN_THROTTLE(10.0, "Could not grab multi echo scan.");
		        device_status_ = urg_->getSensorStatus();
		        error_count_++;
		      }
		    } else {
		      const sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
		      if(urg_->grabScan(msg)){
		        laser_pub.publish(msg);
		        laser_freq_->tick();
		      } else {
		        ROS_WARN_THROTTLE(10.0, "Could not grab single echo scan.");
		        device_status_ = urg_->getSensorStatus();
		        error_count_++;
		      }
		    }
		   } catch(...){
		   		ROS_ERROR_THROTTLE(10.0, "Unknown error grabbing Hokuyo scan.");
		   		error_count_++;
		   }

	    // Reestablish conneciton if things seem to have gone wrong.
	    if(error_count_ > error_limit){
	      ROS_ERROR_THROTTLE(10.0, "Error count exceeded limit, reconnecting.");
	      urg_->stop();
	      ros::Duration(2.0).sleep();
	      ros::spinOnce();
	      break; // Return to top of main loop
	    }
	    ros::spinOnce();
	  }
  }

  // Clean up our diagnostics thread.
  close_diagnostics_ = true;
  diagnostics_thread_.join();

  return EXIT_SUCCESS;
}
