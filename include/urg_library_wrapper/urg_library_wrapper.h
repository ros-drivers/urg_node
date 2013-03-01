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

#ifndef URG_LIBRARY_WRAPPER_H
#define URG_LIBRARY_WRAPPER_H

#include <stdexcept>
#include <sstream>
#include <limits>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>

#include <urg_sensor.h>
#include <urg_utils.h>

namespace urg_library_wrapper
{ 
  class URGLibraryWrapper
  {
  public:
    URGLibraryWrapper(const std::string& ip_address, const int ip_port, bool& using_intensity, bool& using_multiecho);

    URGLibraryWrapper(const int serial_baud, const std::string& serial_port, bool& using_intensity, bool& using_multiecho);

    ~URGLibraryWrapper();

    void start();

    void stop();

    double getRangeMin();

    double getRangeMax();

    double getAngleMin();

    double getAngleMax();

    double getAngleMinLimit();

    double getAngleMaxLimit();

    double getAngleIncrement();

    double getScanPeriod();

    double getTimeIncrement();

    void setFrameId(const std::string& frame_id);

    void setUserLatency(const double latency);

    bool setAngleLimitsAndSkip(double& angle_min, double& angle_max, int skip);

    bool grabScan(const sensor_msgs::LaserScanPtr& msg);

    bool grabScan(const sensor_msgs::MultiEchoLaserScanPtr& msg);

  private:
    void initialize(bool& using_intensity, bool& using_multiecho);

    bool isIntensitySupported();

    bool isMultiEchoSupported();

    std::string frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.

    urg_t urg_;
    bool started_;

    std::vector<long> data_;
    std::vector<unsigned short> intensity_;

    bool use_intensity_;
    bool use_multiecho_;
    urg_measurement_type_t measurement_type_;
    int first_step_;
    int last_step_;

    ros::Duration system_latency_;
    ros::Duration user_latency_;
  };
  
  
}; // urg_library_wrapper

#endif