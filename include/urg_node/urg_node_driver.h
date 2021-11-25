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
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <laser_proc/LaserTransport.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <urg_node/URGConfig.h>
#include <std_srvs/Trigger.h>
#include <bondcpp/bond.h>

#include "urg_node/urg_c_wrapper.h"

namespace urg_node
{
class UrgNode
{
public:
  UrgNode();
  UrgNode(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~UrgNode();

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
  bool reconfigure_callback(urg_node::URGConfig& config, int level);
  void update_reconfigure_limits();
  void calibrate_time_offset();
  void addDiagnostics();
  void updateDiagnostics();
  void populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void scanThread();

  bool statusCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  boost::thread diagnostics_thread_;
  boost::thread scan_thread_;

  boost::shared_ptr<bond::Bond> bond_ = nullptr;
  boost::shared_ptr<urg_node::URGCWrapper> urg_;
  boost::shared_ptr<dynamic_reconfigure::Server<urg_node::URGConfig> > srv_;  ///< Dynamic reconfigure server
  boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> laser_freq_;
  boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> echoes_freq_;

  boost::mutex lidar_mutex_;

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
  bool synchronize_time_;
  bool publish_intensity_;
  bool publish_multiecho_;
  int error_limit_;
  double diagnostics_tolerance_;
  double diagnostics_window_time_;
  bool detailed_status_;

  volatile bool service_yield_;

  ros::Publisher laser_pub_;
  laser_proc::LaserPublisher echoes_pub_;
  ros::Publisher status_pub_;

  ros::ServiceServer status_service_;
};

}  // namespace urg_node

#endif  // URG_NODE_URG_NODE_DRIVER_H
