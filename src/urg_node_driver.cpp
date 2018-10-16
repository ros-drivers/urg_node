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

#include "urg_node/urg_node_driver.hpp"

namespace urg_node
{

// Useful typedefs
typedef diagnostic_updater::FrequencyStatusParam FrequencyStatusParam;

UrgNode::UrgNode(const std::string & topic_name): Node(topic_name)
{
}

UrgNode::UrgNode(): Node("urg_node")
{
}

void UrgNode::initSetup()
{
  close_diagnostics_ = true;
  close_scan_ = true;
  service_yield_ = false;

  error_code_ = 0;
  lockout_status_ = false;

  // Get parameters so we can change these later.
  this->get_parameter_or("ip_address", ip_address_, std::string(""));
  this->get_parameter_or("ip_port", ip_port_, 10940);
  this->get_parameter_or("laser_frame_id", laser_frame_id_, std::string("laser"));
  this->get_parameter_or("serial_port", serial_port_, std::string("/dev/ttyACM0"));
  this->get_parameter_or("serial_baud", serial_baud_, 115200);
  this->get_parameter_or("calibrate_time", calibrate_time_, false);
  this->get_parameter_or("publish_intensity", publish_intensity_, false);
  this->get_parameter_or("publish_multiecho", publish_multiecho_, false);
  this->get_parameter_or("error_limit", error_limit_, 4);
  this->get_parameter_or("diagnostics_tolerance", diagnostics_tolerance_, 0.05);
  this->get_parameter_or("diagnostics_window_time", diagnostics_window_time_, 5.0);
  this->get_parameter_or("get_detailed_status", detailed_status_, false);
  this->get_parameter_or("default_user_latency", default_user_latency_, 0.0);
  this->get_parameter_or("angle_min", angle_min_, -3.14);
  this->get_parameter_or("angle_max", angle_max_, 3.14);
  this->get_parameter_or("skip", skip_, 0);
  this->get_parameter_or("cluster", cluster_, 1);

  // Get parameters so we can change these later.
  pnh_.param<std::string>("ip_address", ip_address_, "");
  pnh_.param<int>("ip_port", ip_port_, 10940);
  pnh_.param<std::string>("serial_port", serial_port_, "/dev/ttyACM0");
  pnh_.param<int>("serial_baud", serial_baud_, 115200);
  pnh_.param<bool>("calibrate_time", calibrate_time_, false);
  pnh_.param<bool>("synchronize_time", synchronize_time_, false);
  pnh_.param<bool>("publish_intensity", publish_intensity_, true);
  pnh_.param<bool>("publish_multiecho", publish_multiecho_, false);
  pnh_.param<int>("error_limit", error_limit_, 4);
  pnh_.param<double>("diagnostics_tolerance", diagnostics_tolerance_, 0.05);
  pnh_.param<double>("diagnostics_window_time", diagnostics_window_time_, 5.0);
  pnh_.param<bool>("get_detailed_status", detailed_status_, false);

  // Set up publishers and diagnostics updaters, we only need one
  if (publish_multiecho_)
  {
    auto nh = this->shared_from_this();
    echoes_pub_ = laser_proc::LaserTransport::advertiseLaser(nh, 20);
  }
  else
  {
    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 20);
  }

  status_service_ = this->create_service<std_srvs::srv::Trigger>("update_laser_status", std::bind(&UrgNode::statusCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // TODO: ros2 does not have latched topics yet, need to play with QOS
  status_pub_ = this->create_publisher<urg_node_msgs::msg::Status>("laser_status", 1);  // latched=true

  diagnostic_updater_.reset(new diagnostic_updater::Updater);
  diagnostic_updater_->add("Hardware Status", this, &UrgNode::populateDiagnosticsStatus);


  // The parameters client to catch modification of parameters
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this());
}

UrgNode::~UrgNode()
{
  if (diagnostics_thread_.joinable())
  {
    // Clean up our diagnostics thread.
    close_diagnostics_ = true;
    diagnostics_thread_.join();
  }
  if (scan_thread_.joinable())
  {
    close_scan_ = true;
    scan_thread_.join();
  }
}

bool UrgNode::updateStatus()
{
  bool result = false;
  service_yield_ = true;
  std::unique_lock<std::mutex> lock(lidar_mutex_);

  if (urg_)
  {
    device_status_ = urg_->getSensorStatus();

    if (detailed_status_)
    {
      URGStatus status;
      if (urg_->getAR00Status(status))
      {
        urg_node_msgs::msg::Status msg;
        msg.operating_mode = status.operating_mode;
        msg.error_status = status.error_status;
        msg.error_code = status.error_code;
        msg.lockout_status = status.lockout_status;

        lockout_status_ = status.lockout_status;
        error_code_ = status.error_code;

        UrgDetectionReport report;
        if (urg_->getDL00Status(report))
        {
          msg.area_number = report.area;
          msg.distance = report.distance;
          msg.angle = report.angle;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Failed to get detection report.");
        }
        // Publish the status on the latched topic for inspection.
        status_pub_->publish(msg);
        result = true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to retrieve status");
        urg_node_msgs::msg::Status msg;
        status_pub_->publish(msg);
      }
    }
  }
  return result;
}

void UrgNode::statusCallback(
    const std::shared_ptr<rmw_request_id_t> requestHeader,
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  RCLCPP_INFO(this->get_logger(), "Got update lidar status callback");
  std::cerr << "Got update lidar status callback" << std::endl;
  res->success = false;
  res->message = "Laser not ready";

  if (updateStatus())
  {
    res->message = "Status retrieved";
    res->success = true;
  }
  else
  {
    res->message = "Failed to update status";
    res->success = false;
  }
}

void UrgNode::reconfigure(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // For debug only
  std::stringstream ss;
  ss << "\nParameter event:\n ";//new parameters:";
  for (auto & new_parameter : event->new_parameters) {
    ss << "\n  " << new_parameter.name;
  }
  //ss << "\n changed parameters:";
  for (auto & changed_parameter : event->changed_parameters) {
    ss << "\n  " << changed_parameter.name;
  }
  //ss << "\n deleted parameters:";
  for (auto & deleted_parameter : event->deleted_parameters) {
    ss << "\n  " << deleted_parameter.name;
  }
  ss << "\n";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  std::vector<rcl_interfaces::msg::Parameter> parameter_vec = event->new_parameters;
  parameter_vec.insert(parameter_vec.end(), event->changed_parameters.begin(), event->changed_parameters.end());

  bool restart_required(false);

  for(auto parameter : parameter_vec){
    if(parameter.name.compare("ip_address") == 0){
      //ip_address_ = parameter.value.string_value;

    } else if(parameter.name.compare("ip_port") == 0){
      //ip_port_ = parameter.value.integer_value;

    } else if(parameter.name.compare("laser_frame_id") == 0){
      laser_frame_id_ = parameter.value.string_value;

    } else if(parameter.name.compare("serial_port") == 0){
      //serial_port_ = parameter.value.string_value;

    } else if(parameter.name.compare("serial_baud") == 0){
      //serial_baud_ = parameter.value.integer_value;

    } else if(parameter.name.compare("calibrate_time") == 0){
      //calibrate_time_ = parameter.value.bool_value;

    } else if(parameter.name.compare("publish_intensity") == 0){
      //publish_intensity_ = parameter.value.bool_value;

    } else if(parameter.name.compare("publish_multiecho") == 0){
      //publish_multiecho_ = parameter.value.bool_value;

    } else if(parameter.name.compare("error_limit") == 0){
      error_limit_ = parameter.value.integer_value;

    } else if(parameter.name.compare("default_user_latency") == 0){
      default_user_latency_ = parameter.value.integer_value + parameter.value.double_value;

    } else if(parameter.name.compare("angle_min") == 0){
      angle_min_ = parameter.value.integer_value + parameter.value.double_value;
      restart_required = true;

    } else if(parameter.name.compare("angle_max") == 0){
      angle_max_ = parameter.value.integer_value + parameter.value.double_value;
      restart_required = true;

    } else if(parameter.name.compare("cluster") == 0){
      cluster_ = parameter.value.integer_value;
      restart_required = true;

    } else if(parameter.name.compare("skip") == 0){
      skip_ = parameter.value.integer_value;
      restart_required = true;

    } else {
      RCLCPP_ERROR(this->get_logger(), "The parameter %s is not part of the reconfigurable parameters.", parameter.name);
    }
  }

  if(restart_required){
    std::unique_lock<std::mutex> lock(lidar_mutex_);
    urg_->stop();
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    urg_->setAngleLimitsAndCluster(angle_min_, angle_max_, cluster_);

    freq_min_ = 1.0 / (urg_->getScanPeriod() * (skip_ + 1));
    urg_->setSkip(skip_);

    try {
      urg_->start();
      RCLCPP_INFO(this->get_logger(), "Streaming data after reconfigure.");
    } catch (std::runtime_error& e) {
      RCLCPP_FATAL(this->get_logger(), "Error while reconfiguring : %s", e.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
      rclcpp::shutdown();
    }
  }

  urg_->setFrameId(laser_frame_id_);
  urg_->setUserLatency(default_user_latency_);
}

void UrgNode::calibrate_time_offset()
{
  std::unique_lock<std::mutex> lock(lidar_mutex_);
  if (!urg_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Unable to calibrate time offset. Not Ready.");
    return;
  }
  try
  {
    // Don't let outside interruption effect lidar offset.
    RCLCPP_INFO(this->get_logger(), "Starting calibration. This will take a few seconds.");
    RCLCPP_WARN(this->get_logger(), "Time calibration is still experimental.");
    rclcpp::Duration latency = urg_->computeLatency(10);
    RCLCPP_INFO(this->get_logger(), "Calibration finished. Latency is: %.4f sec.", (double)(latency.nanoseconds()*1e-9));
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_FATAL(this->get_logger(), "Could not calibrate time offset: %s", e.what());
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
  }
}

// Diagnostics update task to be run in a thread.
void UrgNode::updateDiagnostics()
{
  while (!close_diagnostics_)
  {
    diagnostic_updater_->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// Populate a diagnostics status message.
void UrgNode::populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!urg_)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Not Connected");
    return;
  }

  if (!urg_->getIPAddress().empty())
  {
    stat.add("IP Address", urg_->getIPAddress());
    stat.add("IP Port", urg_->getIPPort());
  }
  else
  {
    stat.add("Serial Port", urg_->getSerialPort());
    stat.add("Serial Baud", urg_->getSerialBaud());
  }

  if (!urg_->isStarted())
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Not Connected: " + device_status_);
  }
  else if (device_status_ != std::string("Sensor works well.") &&
           device_status_ != std::string("Stable 000 no error.") &&
           device_status_ != std::string("sensor is working normally"))
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Abnormal status: " + device_status_);
  }
  else if (error_code_ != 0)
  {
    stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Lidar reporting error code: %X",
        error_code_);
  }
  else if (lockout_status_)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Lidar locked out.");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Streaming");
  }

  stat.add("Vendor Name", vendor_name_);
  stat.add("Product Name", product_name_);
  stat.add("Firmware Version", firmware_version_);
  stat.add("Firmware Date", firmware_date_);
  stat.add("Protocol Version", protocol_version_);
  stat.add("Device ID", device_id_);
  stat.add("Computed Latency", urg_->getComputedLatency().nanoseconds());
  stat.add("User Time Offset", urg_->getUserTimeOffset().nanoseconds());

  // Things not explicitly required by REP-0138, but still interesting.
  stat.add("Device Status", device_status_);
  stat.add("Scan Retrieve Error Count", error_count_);

  stat.add("Lidar Error Code", error_code_);
  stat.add("Locked out", lockout_status_);
}

bool UrgNode::connect()
{
  // Don't let external access to retrieve
  // status during the connection process.
  std::unique_lock<std::mutex> lock(lidar_mutex_);

  try
  {
    urg_.reset();  // Clear any previous connections();
    if (!ip_address_.empty())
    {
      urg_.reset(new urg_node::URGCWrapper(ip_address_, ip_port_,
          publish_intensity_, publish_multiecho_, synchronize_time_));
    }
    else
    {
      urg_.reset(new urg_node::URGCWrapper(serial_baud_, serial_port_,
          publish_intensity_, publish_multiecho_, synchronize_time_));
    }

    std::stringstream ss;
    ss << "Connected to";
    if (publish_multiecho_)
    {
      ss << " multiecho";
    }
    if (!ip_address_.empty())
    {
      ss << " network";
    }
    else
    {
      ss << " serial";
    }
    ss << " device with";
    if (publish_intensity_)
    {
      ss << " intensity and";
    }
    ss << " ID: " << urg_->getDeviceID();
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    device_status_ = urg_->getSensorStatus();
    vendor_name_ = urg_->getVendorName();
    product_name_ = urg_->getProductName();
    firmware_version_ = urg_->getFirmwareVersion();
    firmware_date_ = urg_->getFirmwareDate();
    protocol_version_ = urg_->getProtocolVersion();
    device_id_ = urg_->getDeviceID();

    if (diagnostic_updater_ && urg_)
    {
      diagnostic_updater_->setHardwareID(urg_->getDeviceID());
    }

    // Configure initial properties (in place of initial dynamic reconfigure)

    // The publish frequency changes based on the number of skipped scans.
    // Update accordingly here.
    freq_min_ = 1.0 / (urg_->getScanPeriod() * (skip_ + 1));

    urg_->setAngleLimitsAndCluster(angle_min_, angle_max_, cluster_);
    urg_->setSkip(skip_);

    urg_->setFrameId(laser_frame_id_);
    urg_->setUserLatency(default_user_latency_);

    return true;
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Unknown error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }

  return false;
}

void UrgNode::scanThread()
{
  while (!close_scan_)
  {
    if (!urg_)
    {
      if (!connect())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;  // Connect failed, sleep, try again.
      }
    }

    // Configure limits (Must do this after creating the urgwidget)
    //update_reconfigure_limits();

    if (calibrate_time_)
    {
      calibrate_time_offset();
    }

    // Clear the dynamic reconfigure server
    //srv_.reset();
    // Spin once to de-register it's services before making a new
    // service next.
    //auto nh = this->shared_from_this();
    //rclcpp::spin_some(this->shared_from_this());

    if (!urg_ || !rclcpp::ok)
    {
      continue;
    }
    else
    {
#if 0
      // Set up dynamic reconfigure
      srv_.reset(new dynamic_reconfigure::Server<urg_node::URGConfig>(this));
      // Configure limits (Must do this after creating the urgwidget)
      update_reconfigure_limits();
      srv_->setCallback(std::bind(&UrgNode::reconfigure_callback, this, _1, _2));
#endif

      parameter_event_sub_ = parameters_client_->on_parameter_event(std::bind(&UrgNode::reconfigure, this, std::placeholders::_1));
      
    }

    // Before starting, update the status
    updateStatus();

    // Start the urgwidget
    try
    {
      // If the connection failed, don't try and connect
      // pointer is invalid.
      if (!urg_)
      {
        continue;  // Return to top of main loop, not connected.
      }
      device_status_ = urg_->getSensorStatus();
      urg_->start();
      RCLCPP_INFO(this->get_logger(), "Streaming data.");
      // Clear the error count.
      error_count_ = 0;
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error starting Hokuyo: %s", e.what());
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown error starting Hokuyo");
      urg_.reset();
      rclcpp::sleep_for(std::chrono::seconds(1));
      continue;  // Return to top of main loop
    }

    while (!close_scan_)
    {
      // Don't allow external access during grabbing the scan.
      try
      {
        std::unique_lock<std::mutex> lock(lidar_mutex_);
        if (publish_multiecho_)
        {
          const sensor_msgs::msg::MultiEchoLaserScan::SharedPtr msg(new sensor_msgs::msg::MultiEchoLaserScan());
          if (urg_->grabScan(msg))
          {
            echoes_pub_.publish(msg);
            echoes_freq_->tick();
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Could not grab multi echo scan.");
            device_status_ = urg_->getSensorStatus();
            error_count_++;
          }
        }
        else
        {
          const sensor_msgs::msg::LaserScan::SharedPtr msg(new sensor_msgs::msg::LaserScan());
          if (urg_->grabScan(msg))
          {
            laser_pub_->publish(msg);
            laser_freq_->tick();
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Could not grab single echo scan.");
            device_status_ = urg_->getSensorStatus();
            error_count_++;
          }
        }
      }
      catch (...)
      {
        RCLCPP_WARN(this->get_logger(), "Unknown error grabbing Hokuyo scan.");
        error_count_++;
      }

      if (service_yield_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        service_yield_ = false;
      }

      // Reestablish connection if things seem to have gone wrong.
      if (error_count_ > error_limit_)
      {
        RCLCPP_ERROR(this->get_logger(), "Error count exceeded limit, reconnecting.");
        urg_.reset();
        rclcpp::sleep_for(std::chrono::seconds(2));

        break;  // Return to top of main loop
      }
    }
  }
}

void UrgNode::run()
{
  // Setup initial connection
  connect();

  // Stop diagnostics
  if (!close_diagnostics_)
  {
    close_diagnostics_ = true;
    diagnostics_thread_.join();
  }

  if (publish_multiecho_)
  {
    echoes_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("Laser Echoes",
          *diagnostic_updater_,
          FrequencyStatusParam(&freq_min_, &freq_min_, diagnostics_tolerance_, diagnostics_window_time_)));
  }
  else
  {
    laser_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("Laser Scan",
          *diagnostic_updater_,
          FrequencyStatusParam(&freq_min_, &freq_min_, diagnostics_tolerance_, diagnostics_window_time_)));
  }

  // Now that we are setup, kick off diagnostics.
  close_diagnostics_ = false;
  diagnostics_thread_ = std::thread(std::bind(&UrgNode::updateDiagnostics, this));

  // Start scanning now that everything is configured.
  close_scan_ = false;
  scan_thread_ = std::thread(std::bind(&UrgNode::scanThread, this));
}
}  // namespace urg_node
