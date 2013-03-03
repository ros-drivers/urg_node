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

#include <urg_node/urg_c_wrapper.h>
#include <ros/console.h>

using namespace urg_node;

URGCWrapper::URGCWrapper(const std::string& ip_address, const int ip_port, bool& using_intensity, bool& using_multiecho){
  // Store for comprehensive diagnostics
  ip_address_ = ip_address;
  ip_port_ = ip_port;
  serial_port_ = "";
  serial_baud_ = 0;


  long baudrate_or_port = (long)ip_port;
  const char *device = ip_address.c_str();

  int result = urg_open(&urg_, URG_ETHERNET, device, baudrate_or_port);
  if (result < 0) {
    std::stringstream ss;
    ss << "Could not open network Hokuyo:\n";
    ss << ip_address << ":" << ip_port << "\n";
    ss << urg_error(&urg_);
    throw std::runtime_error(ss.str());
  }

  initialize(using_intensity, using_multiecho);
}

URGCWrapper::URGCWrapper(const int serial_baud, const std::string& serial_port, bool& using_intensity, bool& using_multiecho){
  // Store for comprehensive diagnostics
  serial_baud_ = serial_baud;
  serial_port_ = serial_port;
  ip_address_ = "";
  ip_port_ = 0;

  long baudrate_or_port = (long)serial_baud;
  const char *device = serial_port.c_str();

  int result = urg_open(&urg_, URG_SERIAL, device, baudrate_or_port);
  if (result < 0) {
    std::stringstream ss;
    ss << "Could not open serial Hokuyo:\n";
    ss << serial_port << " @ " << serial_baud << "\n";
    ss << urg_error(&urg_);
    throw std::runtime_error(ss.str());
  }

  initialize(using_intensity, using_multiecho);
}

void URGCWrapper::initialize(bool& using_intensity, bool& using_multiecho){
  data_.resize(urg_max_data_size(&urg_) * URG_MAX_ECHO);
  intensity_.resize(urg_max_data_size(&urg_) * URG_MAX_ECHO);

  if(using_intensity){
  	using_intensity = isIntensitySupported();
  }

  if(using_multiecho){
  	using_multiecho = isMultiEchoSupported();
  }

  use_intensity_ = using_intensity;
	use_multiecho_ = using_multiecho;

	measurement_type_ = URG_DISTANCE;
	if(use_intensity_ && use_multiecho_){
		measurement_type_ = URG_MULTIECHO_INTENSITY;
	} else if(use_intensity_){
	    measurement_type_ = URG_DISTANCE_INTENSITY;
	} else if(use_multiecho_){
		measurement_type_ = URG_MULTIECHO;
	}

  started_ = false;
  frame_id_ = "";
  first_step_ = 0;
  last_step_ = 0;
  cluster_ = 1;
  skip_ = 0;
}

void URGCWrapper::start(){
	if(!started_){
		int result = urg_start_measurement(&urg_, measurement_type_, 0, skip_);
		if (result < 0) {
	      std::stringstream ss;
	      ss << "Could not start Hokuyo measurement:\n";
	      if(use_intensity_){
	      	ss << "With Intensity" << "\n";
	      }
	      if(use_multiecho_){
	      	ss << "With MultiEcho" << "\n";
	      }
	      ss << urg_error(&urg_);
	      throw std::runtime_error(ss.str());
	    }
    }
    started_ = true;
}

void URGCWrapper::stop(){
	urg_stop_measurement(&urg_);
	started_ = false;
}

URGCWrapper::~URGCWrapper(){
	stop();
	urg_close(&urg_);
}

bool URGCWrapper::grabScan(const sensor_msgs::LaserScanPtr& msg){
  msg->header.frame_id = frame_id_;
  msg->angle_min = getAngleMin();
  msg->angle_max = getAngleMax();
  msg->angle_increment = getAngleIncrement();
  msg->scan_time = getScanPeriod();
  msg->time_increment = getTimeIncrement();
  msg->range_min = getRangeMin();
  msg->range_max = getRangeMax();
  
  // Grab scan
  int num_beams = 0;
  long time_stamp = 0;
  unsigned long long system_time_stamp = 0;

  if(use_intensity_){
  	num_beams = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
  } else {
  	num_beams = urg_get_distance(&urg_, &data_[0], &time_stamp, &system_time_stamp);
  } 
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
  msg->header.stamp.fromNSec((uint64_t)system_time_stamp);
  msg->header.stamp = msg->header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
  msg->ranges.resize(num_beams);
  if(use_intensity_){
  	msg->intensities.resize(num_beams);
  }
  
  for (int i = 0; i < num_beams; i++) {
    if(data_[(i) + 0] != 0){
      msg->ranges[i] = (float)data_[i]/1000.0;
      if(use_intensity_){
      	msg->intensities[i] = intensity_[i];
  	  }
    } else {
      msg->ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }  
  }
  return true;
}

bool URGCWrapper::grabScan(const sensor_msgs::MultiEchoLaserScanPtr& msg){
  msg->header.frame_id = frame_id_;
  msg->angle_min = getAngleMin();
  msg->angle_max = getAngleMax();
  msg->angle_increment = getAngleIncrement();
  msg->scan_time = getScanPeriod();
  msg->time_increment = getTimeIncrement();
  msg->range_min = getRangeMin();
  msg->range_max = getRangeMax();
  
  // Grab scan
  int num_beams = 0;
  long time_stamp = 0;
  unsigned long long system_time_stamp;

  if(use_intensity_){
  	num_beams = urg_get_multiecho_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
  } else {
  	num_beams = urg_get_multiecho(&urg_, &data_[0], &time_stamp, &system_time_stamp);
  } 
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
  msg->header.stamp.fromNSec((uint64_t)system_time_stamp);
  msg->header.stamp = msg->header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
  msg->ranges.resize(num_beams);
  if(use_intensity_){
  	msg->intensities.resize(num_beams);
  }
  
  for (int i = 0; i < num_beams; i++) {
    if(data_[(URG_MAX_ECHO * i) + 0] != 0){
      msg->ranges[i].echoes.push_back((float)data_[(URG_MAX_ECHO * i) + 0]/1000.0);
      if(use_intensity_){
      	msg->intensities[i].echoes.push_back(intensity_[(URG_MAX_ECHO * i) + 0]);
  	  }
    } else {
      continue;
    }

    if(data_[(URG_MAX_ECHO * i) + 1] != 0){
      msg->ranges[i].echoes.push_back((float)data_[(URG_MAX_ECHO * i) + 1]/1000.0);
      if(use_intensity_){
        msg->intensities[i].echoes.push_back(intensity_[(URG_MAX_ECHO * i) + 1]);
      }
    } else {
      continue;
    }

    if(data_[(URG_MAX_ECHO * i) + 2] != 0){
      msg->ranges[i].echoes.push_back((float)data_[(URG_MAX_ECHO * i) + 2]/1000.0);
      if(use_intensity_){
        msg->intensities[i].echoes.push_back(intensity_[(URG_MAX_ECHO * i) + 2]);
      }
    } else {
      continue;
    }
      
  }
  return true;
}

double URGCWrapper::getRangeMin(){
  long minr;
  long maxr;
  urg_distance_min_max(&urg_, &minr, &maxr);
  return (double)minr/1000.0;
}

double URGCWrapper::getRangeMax(){
  long minr;
  long maxr;
  urg_distance_min_max(&urg_, &minr, &maxr);
  return (double)maxr/1000.0;
}

double URGCWrapper::getAngleMin(){
  return urg_step2rad(&urg_, first_step_);
}

double URGCWrapper::getAngleMax(){
  return urg_step2rad(&urg_, last_step_);
}

double URGCWrapper::getAngleMinLimit(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, min_step);
}

double URGCWrapper::getAngleMaxLimit(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, max_step);
}

double URGCWrapper::getAngleIncrement(){
  double angle_min = getAngleMin();
  double angle_max = getAngleMax();
  return cluster_*(angle_max-angle_min)/(double)(last_step_-first_step_);
}

double URGCWrapper::getScanPeriod(){
  long scan_usec = urg_scan_usec(&urg_);
  return 1.e-6*(double)(scan_usec);
}

double URGCWrapper::getTimeIncrement(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  double scan_period = getScanPeriod();
  double circle_fraction = (getAngleMaxLimit()-getAngleMinLimit())/(2.0*3.141592);
  return cluster_*circle_fraction*scan_period/(double)(max_step-min_step);
}


std::string URGCWrapper::getIPAddress(){
  return ip_address_;
}

int URGCWrapper::getIPPort(){
  return ip_port_;
}

std::string URGCWrapper::getSerialPort(){
  return serial_port_;
}

int URGCWrapper::getSerialBaud(){
  return serial_baud_;
}

std::string URGCWrapper::getVendorName(){
  return std::string(urg_sensor_vendor(&urg_));
}

std::string URGCWrapper::getProductName(){
  return std::string(urg_sensor_product_type(&urg_));
}

std::string URGCWrapper::getFirmwareVersion(){
  return std::string(urg_sensor_firmware_version(&urg_));
}

std::string URGCWrapper::getFirmwareDate(){
  return std::string(urg_sensor_firmware_date(&urg_));
}

std::string URGCWrapper::getProtocolVersion(){
  return std::string(urg_sensor_protocol_version(&urg_));
}

std::string URGCWrapper::getDeviceID(){
  return std::string(urg_sensor_serial_id(&urg_));
}

ros::Duration URGCWrapper::getComputedLatency(){
  return system_latency_;
}

ros::Duration URGCWrapper::getUserTimeOffset(){
  return user_latency_;
}

std::string URGCWrapper::getSensorStatus(){
  return std::string(urg_sensor_status(&urg_));
}

std::string URGCWrapper::getSensorState(){
  return std::string(urg_sensor_state(&urg_));
}

void URGCWrapper::setFrameId(const std::string& frame_id){
  frame_id_ = frame_id;
}

void URGCWrapper::setUserLatency(const double latency){
  user_latency_.fromSec(latency);
}

// Must be called before urg_start
bool URGCWrapper::setAngleLimitsAndCluster(double& angle_min, double& angle_max, int cluster){
  if(started_){
  	return false; // Must not be streaming
  }

  // Set step limits
  first_step_ = urg_rad2step(&urg_, angle_min);
  last_step_ = urg_rad2step(&urg_, angle_max);
  cluster_ = cluster;

  // Make sure step limits are not the same
  if(first_step_ == last_step_){
  	// Make sure we're not at a limit
  	int min_step;
	int max_step;
	urg_step_min_max(&urg_, &min_step, &max_step);
	if(first_step_ == min_step){ // At beginning of range
		last_step_ = first_step_ + 1;
	} else { // At end of range (or all other cases)
		first_step_ = last_step_ - 1;
	} 
  }

  // Make sure angle_max is greater than angle_min (should check this after end limits)
  if(last_step_ < first_step_){
  	double temp = first_step_;
  	first_step_ = last_step_;
  	last_step_ = temp;
  }

  angle_min = urg_step2rad(&urg_, first_step_);
  angle_max = urg_step2rad(&urg_, last_step_);
  int result = urg_set_scanning_parameter(&urg_, first_step_, last_step_, cluster);
  if(result < 0){
    return false;
  }
  return true;
}

bool URGCWrapper::setSkip(int skip){
	skip_ = skip;
}

bool URGCWrapper::isIntensitySupported(){
  if(started_){
  	return false; // Must not be streaming
  }

  std::vector<long> data;

  urg_start_measurement(&urg_, URG_DISTANCE_INTENSITY, 0, 0);
  int ret = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], NULL, NULL);
  if(ret <= 0){
  	return false; // Failed to start measurement with intensity: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

bool URGCWrapper::isMultiEchoSupported(){
  if(started_){
  	return false; // Must not be streaming
  }

  urg_start_measurement(&urg_, URG_MULTIECHO, 0, 0);
  int ret = urg_get_multiecho(&urg_, &data_[0], NULL, NULL);
  if(ret <= 0){
  	return false; // Failed to start measurement with multiecho: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

ros::Duration URGCWrapper::getAngularTimeOffset(){
  // Adjust value for Hokuyo's timestamps
  // Hokuyo's timestamps start from the rear center of the device (at Pi according to ROS standards)
  double circle_fraction = 0.0;
  if(first_step_ == 0 && last_step_ == 0){
    circle_fraction = (getAngleMinLimit()+3.141592)/(2.0*3.141592);
  } else {
    circle_fraction = (getAngleMin()+3.141592)/(2.0*3.141592);
  }
  return ros::Duration(circle_fraction * getScanPeriod());
}

ros::Duration URGCWrapper::computeLatency(size_t num_measurements){
  system_latency_.fromNSec(0);

	ros::Duration start_offset = getNativeClockOffset(1);
	ros::Duration previous_offset;

	std::vector<ros::Duration> time_offsets(num_measurements);
	for (size_t i = 0; i < num_measurements; i++){
		ros::Duration scan_offset = getTimeStampOffset(1);
		ros::Duration post_offset = getNativeClockOffset(1);
		ros::Duration adjusted_scan_offset = scan_offset - start_offset;
		ros::Duration adjusted_post_offset = post_offset - start_offset;
		ros::Duration average_offset;
		average_offset.fromSec((adjusted_post_offset.toSec() + previous_offset.toSec())/2.0);

		time_offsets[i] = adjusted_scan_offset - average_offset;

		previous_offset = adjusted_post_offset;
	}

  // Get median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(time_offsets.begin(), time_offsets.begin() + time_offsets.size()/2, time_offsets.end());
  system_latency_ = time_offsets[time_offsets.size()/2];
  return system_latency_ + getAngularTimeOffset(); // Angular time offset makes the output comparable to that of hokuyo_node
}

ros::Duration URGCWrapper::getNativeClockOffset(size_t num_measurements){
  if(started_){
    std::stringstream ss;
    ss << "Cannot get native clock offset while started.";
    throw std::runtime_error(ss.str());
  }

  if(urg_start_time_stamp_mode(&urg_) < 0){
    std::stringstream ss;
    ss << "Cannot start time stamp mode.";
    throw std::runtime_error(ss.str());
  }

  std::vector<ros::Duration> time_offsets(num_measurements);
  for (size_t i = 0; i < num_measurements; i++)
  {
  	ros::Time request_time = ros::Time::now();
  	ros::Time laser_time;
  	laser_time.fromNSec(1e6*(uint64_t)urg_time_stamp(&urg_)); // 1e6 * milliseconds = nanoseconds
  	ros::Time response_time = ros::Time::now();
  	ros::Time average_time;
  	average_time.fromSec((response_time.toSec()+request_time.toSec())/2.0);
  	time_offsets[i] = laser_time - average_time;
  }

  if(urg_stop_time_stamp_mode(&urg_) < 0){
    std::stringstream ss;
    ss << "Cannot stop time stamp mode.";
    throw std::runtime_error(ss.str());
  };

  // Return median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(time_offsets.begin(), time_offsets.begin() + time_offsets.size()/2, time_offsets.end());
  return time_offsets[time_offsets.size()/2];
}

ros::Duration URGCWrapper::getTimeStampOffset(size_t num_measurements){
  if(started_){
    std::stringstream ss;
    ss << "Cannot get time stamp offset while started.";
    throw std::runtime_error(ss.str());
  }

  start();

  std::vector<ros::Duration> time_offsets(num_measurements);
  for(size_t i = 0; i < num_measurements; i++){
  	long time_stamp;
    unsigned long long system_time_stamp;
  	int ret = 0;

   	if(measurement_type_ == URG_DISTANCE){
   		ret = urg_get_distance(&urg_, &data_[0], &time_stamp, &system_time_stamp);
   	} else if(measurement_type_ == URG_DISTANCE_INTENSITY){
  		ret = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
   	} else if(measurement_type_ == URG_MULTIECHO){
   		ret = urg_get_multiecho(&urg_, &data_[0], &time_stamp, &system_time_stamp);
   	} else if(measurement_type_ == URG_MULTIECHO_INTENSITY){
   		ret = urg_get_multiecho_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
   	}

   	if(ret <= 0){
   		std::stringstream ss;
  	  ss << "Cannot get scan to measure time stamp offset.";
  	  throw std::runtime_error(ss.str());
   	}

   	ros::Time laser_timestamp;
   	laser_timestamp.fromNSec(1e6*(uint64_t)time_stamp);
    ros::Time system_time;
    system_time.fromNSec((uint64_t)system_time_stamp);

   	time_offsets[i] = laser_timestamp - system_time;
  }

  stop();

  // Return median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(time_offsets.begin(), time_offsets.begin() + time_offsets.size()/2, time_offsets.end());
  return time_offsets[time_offsets.size()/2];
}