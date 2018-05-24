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
 * Author: Chad Rockey, Michael Carroll, Mike O'Driscoll
 */

#include "urg_node/urg_node_driver.h"

// boost headers
#include <boost/lexical_cast.hpp>

// rcutils headers
#include <rcutils/cmdline_parser.h>


int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("urg_node");

  // Support the optional serial port command line argument
  std::string serialPort = "/dev/ttyACM0";
  std::string option = "--serial-port";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    serialPort = rcutils_cli_get_option(argv, argv + argc, option.c_str());
  }

  // Support the optional user latency command line argument
  double userLatency = 0;
  option = "--user-latency";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    std::string strLatency = rcutils_cli_get_option(argv, argv + argc, option.c_str());
    userLatency = boost::lexical_cast<double>(strLatency);
  }

  // Support the optional IP address command line argument
  std::string ipAddress = "";
  option = "--ip-addr";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    ipAddress = rcutils_cli_get_option(argv, argv + argc, option.c_str());
  }

  // Support the optional IP port command line argument
  int ipPort = 0;
  option = "--port";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    std::string strIPPort = rcutils_cli_get_option(argv, argv + argc, option.c_str());
    ipPort = boost::lexical_cast<int>(strIPPort);
  }

  // Support the optional laser frame id command line argument
  std::string laserFrameId = "laser";
  option = "--laser-frame-id";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    laserFrameId = rcutils_cli_get_option(argv, argv + argc, option.c_str());
  }

  urg_node::UrgNode urgNode;

  // Update settings
  urgNode.setSerialPort(serialPort);
  urgNode.setUserLatency(userLatency);
  urgNode.setIPAdddress(ipAddress);
  urgNode.setIPPort(ipPort);

  // Run the urg node
  urgNode.run();

  rclcpp::spin(node);

  return 0;
}
