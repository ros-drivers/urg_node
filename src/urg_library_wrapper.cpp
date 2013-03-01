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

#include <urg_library_wrapper/urg_library_wrapper.h>

using namespace urg_library_wrapper;

URGLibraryWrapper::URGLibraryWrapper(const std::string& ip_address, const int ip_port, bool& using_intensity, bool& using_multiecho){
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

URGLibraryWrapper::URGLibraryWrapper(const int serial_baud, const std::string& serial_port, bool& using_intensity, bool& using_multiecho){
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

void URGLibraryWrapper::initialize(bool& using_intensity, bool& using_multiecho){
    data_.resize(urg_max_data_size(&urg_) * URG_MAX_ECHO);
    intensity_.resize(urg_max_data_size(&urg_));

    if(using_intensity){
    	using_intensity = isIntensitySupported();
    }

    if(using_multiecho){
    	using_multiecho = isMultiEchoSupported();
    }

    start(using_intensity, using_multiecho);
}

void URGLibraryWrapper::start(bool& using_intensity, bool& using_multiecho){
	use_intensity_ = using_intensity;
	use_multiecho_ = using_multiecho;

	urg_measurement_type_t type = URG_DISTANCE;
	if(use_intensity_ && use_multiecho_){
		type = URG_MULTIECHO_INTENSITY;
	} else if(use_intensity_){
	    type = URG_DISTANCE_INTENSITY;
	} else if(use_multiecho_){
		type = URG_MULTIECHO;
	} else {
	}
	int result = urg_start_measurement(&urg_, type, 0, 0);
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

URGLibraryWrapper::~URGLibraryWrapper(){
	urg_stop_measurement(&urg_);
	urg_close(&urg_);
}

bool URGLibraryWrapper::grabScan(const sensor_msgs::LaserScanPtr& msg){
  msg->header.stamp = ros::Time::now();
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

  if(use_intensity_){
  	num_beams = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp);
  } else {
  	num_beams = urg_get_distance(&urg_, &data_[0], &time_stamp);
  } 
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
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

bool URGLibraryWrapper::grabScan(const sensor_msgs::MultiEchoLaserScanPtr& msg){
  msg->header.stamp = ros::Time::now();
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

  if(use_intensity_){
  	num_beams = urg_get_multiecho_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp);
  } else {
  	num_beams = urg_get_multiecho(&urg_, &data_[0], &time_stamp);
  } 
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
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



double URGLibraryWrapper::getRangeMin(){
  long minr;
  long maxr;
  urg_distance_min_max(&urg_, &minr, &maxr);
  return (double)minr/1000.0;
}

double URGLibraryWrapper::getRangeMax(){
  long minr;
  long maxr;
  urg_distance_min_max(&urg_, &minr, &maxr);
  return (double)maxr/1000.0;
}

double URGLibraryWrapper::getAngleMin(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, min_step);
}

double URGLibraryWrapper::getAngleMax(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, max_step);
}

double URGLibraryWrapper::getAngleIncrement(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  double angle_min = getAngleMin();
  double angle_max = getAngleMax();
  return (angle_max-angle_min)/(double)(max_step-min_step);
}

double URGLibraryWrapper::getScanPeriod(){
  long scan_usec = urg_scan_usec(&urg_);
  return 1.e-6*(double)(scan_usec);
}

double URGLibraryWrapper::getTimeIncrement(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  double scan_period = getScanPeriod();
  double circle_fraction = (getAngleMax()-getAngleMin())/(2.0*3.141592);
  return circle_fraction*scan_period/(double)(max_step-min_step);
}

void URGLibraryWrapper::setFrameId(const std::string& frame_id){
	frame_id_ = frame_id;
}

bool URGLibraryWrapper::isIntensitySupported(){
  std::vector<long> data;
  long time_stamp;

  urg_start_measurement(&urg_, URG_DISTANCE_INTENSITY, 0, 0);
  int ret = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp);
  std::cout << "Ret: " << ret << std::endl;
  if(ret <= 0){
  	return false; // Failed to start measurement with intensity: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

bool URGLibraryWrapper::isMultiEchoSupported(){
  long time_stamp;

  urg_start_measurement(&urg_, URG_MULTIECHO, 0, 0);
  int ret = urg_get_multiecho(&urg_, &data_[0], &time_stamp);
  std::cout << "Ret: " << ret << std::endl;
  if(ret <= 0){
  	return false; // Failed to start measurement with multiecho: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}