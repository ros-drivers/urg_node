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

URGLibraryWrapper::URGLibraryWrapper(const std::string& ip_address, const int ip_port){
	data_ = NULL;
	intensity_ = NULL;

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
}

URGLibraryWrapper::URGLibraryWrapper(const int serial_baud, const std::string& serial_port){
	data_ = NULL;
	intensity_ = NULL;

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
}

URGLibraryWrapper::~URGLibraryWrapper(){
	free(data_);
	free(intensity_);
	urg_close(&urg_);
}

double URGLibraryWrapper::getMinAngle(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, min_step);
}

double URGLibraryWrapper::getMaxAngle(){
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, max_step);
}

double URGLibraryWrapper::getScanTime(){
  long scan_usec = urg_scan_usec(&urg_);
  return 1.e-6*(double)(scan_usec);
}