/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2017, Clearpath Robotics, Inc.
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
 * Author: Mike O'Driscoll
 */

#ifndef URG_NODE_URG_NODE_DRIVER_H
#define URG_NODE_URG_NODE_DRIVER_H

#include <string>
#include <rclcpp/rclcpp.hpp>
//#include <dynamic_reconfigure/server.h>
#include <laser_proc/LaserTransport.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
//#include <urg_node/URGConfig.h>
#include <std_srvs/srv/trigger.hpp>

#include "urg_node_msgs/msg/status.hpp"
#include "urg_node/urg_c_wrapper.h"

namespace urg_node
{
class UrgNode
{
public:
  UrgNode();
  UrgNode(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr private_nh);
  ~UrgNode();

  /**
   * @brief Set the serial port used by the lidar
   * Set the serial port used by the lidar
   * @param port is the new serial port
   */
  void setSerialPort(const std::string& port);

  /**
   * @brief Set the user latency
   * Set the amount of user latency (in seconds)
   * @param latency is the user latency in seconds
   */
  void setUserLatency(const double& latency);

  /**
   * @brief Set the laser frame id
   * Set the laser tf frame id.
   * @param laserFrameId is the laser tf frame id
   */
  void setLaserFrameId(const std::string& laserFrameId);

  /**
   * @brief Set the IP Address
   * Set the IP address used by the lidar
   * @param ipAddr is the IP address
   */
  void setIPAdddress(const std::string& ipAddr);

  /**
   * @brief Set the IP port
   * Set the IP port used by the lidar
   * @param ipPort is the IP port
   */
  void setIPPort(const int& ipPort);

  /**
   * @brief Start's the nodes threads to run the lidar.
   */
  void run();

  /**
   * @brief Trigger an update of the lidar's status
   * publish the latest known information about the lidar on latched topic.
   * @return True on update successful, false otherwise.
   */
  bool updateStatus();

private:
  void initSetup();
  bool connect();
  //bool reconfigure_callback(urg_node::URGConfig& config, int level);
  //void update_reconfigure_limits();
  void calibrate_time_offset();
  void updateDiagnostics();
  void populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void scanThread();

  void statusCallback(
      const std::shared_ptr<rmw_request_id_t> requestHeader,
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      const std_srvs::srv::Trigger::Response::SharedPtr res);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr pnh_;

  std::thread diagnostics_thread_;
  std::thread scan_thread_;

  std::shared_ptr<urg_node::URGCWrapper> urg_;
  //std::shared_ptr<dynamic_reconfigure::Server<urg_node::URGConfig> > srv_;  ///< Dynamic reconfigure server
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> laser_freq_;
  std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> echoes_freq_;

  std::mutex lidar_mutex_;

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
  uint16_t error_code_;
  bool lockout_status_;

  int error_count_;
  double freq_min_;
  bool close_diagnostics_;
  bool close_scan_;

  int ip_port_;
  std::string ip_address_;
  std::string serial_port_;
  int serial_baud_;
  bool calibrate_time_;
  bool publish_intensity_;
  bool publish_multiecho_;
  int error_limit_;
  double diagnostics_tolerance_;
  double diagnostics_window_time_;
  bool detailed_status_;

  /** The default user latency value. */
  double default_user_latency_;

  /** The laser tf frame id. */
  std::string laser_frame_id_;

  volatile bool service_yield_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  laser_proc::LaserPublisher echoes_pub_;
  rclcpp::Publisher<urg_node_msgs::msg::Status>::SharedPtr status_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
};

}  // namespace urg_node

#endif  // URG_NODE_URG_NODE_DRIVER_H
