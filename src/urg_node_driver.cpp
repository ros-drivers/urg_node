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

#include <tf/tf.h>  // tf header for resolving tf prefix
#include <string>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <urg_node/Status.h>

namespace urg_node
{

// Useful typedefs
typedef diagnostic_updater::FrequencyStatusParam FrequencyStatusParam;

UrgNode::UrgNode(ros::NodeHandle nh, ros::NodeHandle private_nh) :
  nh_(nh),
  pnh_(private_nh)
{
  initSetup();
}

UrgNode::UrgNode():
  nh_(),
  pnh_("~")
{
  initSetup();
}

void UrgNode::initSetup()
{
  close_diagnostics_ = true;
  close_scan_ = true;
  service_yield_ = false;

  error_code_ = 0;
  lockout_status_ = false;
  // Initialize node and nodehandles

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
    echoes_pub_ = laser_proc::LaserTransport::advertiseLaser(nh_, 20);
  }
  else
  {
    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 20);
  }

  status_service_ = nh_.advertiseService("update_laser_status", &UrgNode::statusCallback, this);
  status_pub_ = nh_.advertise<urg_node::Status>("laser_status", 1, true);

  diagnostic_updater_.reset(new diagnostic_updater::Updater);
  diagnostic_updater_->add("Hardware Status", this, &UrgNode::populateDiagnosticsStatus);
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
  boost::mutex::scoped_lock lock(lidar_mutex_);

  if (urg_)
  {
    device_status_ = urg_->getSensorStatus();

    if (detailed_status_)
    {
      URGStatus status;
      if (urg_->getAR00Status(status))
      {
        urg_node::Status msg;
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
           ROS_WARN("Failed to get detection report.");
        }

        // Publish the status on the latched topic for inspection.
        status_pub_.publish(msg);
        result = true;
      }
      else
      {
        ROS_WARN("Failed to retrieve status");

        urg_node::Status msg;
        status_pub_.publish(msg);
      }
    }
  }
  return result;
}

bool UrgNode::statusCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Got update lidar status callback");
  res.success = false;
  res.message = "Laser not ready";

  if (updateStatus())
  {
    res.message = "Status retrieved";
    res.success = true;
  }
  else
  {
    res.message = "Failed to update status";
    res.success = false;
  }

  return true;
}

bool UrgNode::reconfigure_callback(urg_node::URGConfig& config, int level)
{
  if (!urg_)
  {
    ROS_ERROR("Reconfigure failed, not ready");
    return false;
  }

  if (level < 0)  // First call, initialize, laser not yet started
  {
    urg_->setAngleLimitsAndCluster(config.angle_min, config.angle_max, config.cluster);
    urg_->setSkip(config.skip);
  }
  else if (level > 0)   // Must stop
  {
    urg_->stop();
    ROS_INFO("Stopped data due to reconfigure.");

    // Change values that required stopping
    urg_->setAngleLimitsAndCluster(config.angle_min, config.angle_max, config.cluster);
    urg_->setSkip(config.skip);

    try
    {
      urg_->start();
      ROS_INFO("Streaming data after reconfigure.");
    }
    catch (std::runtime_error& e)
    {
      ROS_FATAL("%s", e.what());
      ros::Duration(1.0).sleep();
      ros::shutdown();
      return false;
    }
  }

  // The publish frequency changes based on the number of skipped scans.
  // Update accordingly here.
  freq_min_ = 1.0 / (urg_->getScanPeriod() * (config.skip + 1));

  std::string frame_id = tf::resolve(config.tf_prefix, config.frame_id);
  urg_->setFrameId(frame_id);
  urg_->setUserLatency(config.time_offset);

  return true;
}

void UrgNode::update_reconfigure_limits()
{
  if (!urg_ || !srv_)
  {
    ROS_DEBUG_THROTTLE(10, "Unable to update reconfigure limits. Not Ready.");
    return;
  }

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

void UrgNode::calibrate_time_offset()
{
  boost::mutex::scoped_lock lock(lidar_mutex_);
  if (!urg_)
  {
    ROS_DEBUG_THROTTLE(10, "Unable to calibrate time offset. Not Ready.");
    return;
  }
  try
  {
    // Don't let outside interruption effect lidar offset.
    ROS_INFO("Starting calibration. This will take a few seconds.");
    ROS_WARN("Time calibration is still experimental.");
    ros::Duration latency = urg_->computeLatency(10);
    ROS_INFO("Calibration finished. Latency is: %.4f.", latency.toSec());
  }
  catch (std::runtime_error& e)
  {
    ROS_FATAL("Could not calibrate time offset: %s", e.what());
    ros::Duration(1.0).sleep();
    ros::shutdown();
  }
}


// Diagnostics update task to be run in a thread.
void UrgNode::updateDiagnostics()
{
  while (!close_diagnostics_)
  {
    diagnostic_updater_->update();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
  }
}

// Populate a diagnostics status message.
void UrgNode::populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  if (!urg_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
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
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Not Connected: " + device_status_);
  }
  else if (device_status_ != std::string("Sensor works well.") &&
           device_status_ != std::string("Stable 000 no error.") &&
           device_status_ != std::string("sensor is working normally"))
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Abnormal status: " + device_status_);
  }
  else if (error_code_ != 0)
  {
    stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Lidar reporting error code: %X",
        error_code_);
  }
  else if (lockout_status_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Lidar locked out.");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
        "Streaming");
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
  stat.add("Scan Retrieve Error Count", error_count_);

  stat.add("Lidar Error Code", error_code_);
  stat.add("Locked out", lockout_status_);
}

bool UrgNode::connect()
{
  // Don't let external access to retrieve
  // status during the connection process.
  boost::mutex::scoped_lock lock(lidar_mutex_);

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
    ROS_INFO_STREAM(ss.str());

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

    return true;
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(10.0, "Error connecting to Hokuyo: %s", e.what());
    urg_.reset();
    return false;
  }
  catch (std::exception& e)
  {
    ROS_ERROR_THROTTLE(10.0, "Unknown error connecting to Hokuyo: %s", e.what());
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
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        continue;  // Connect failed, sleep, try again.
      }
    }

    // Configure limits (Must do this after creating the urgwidget)
    update_reconfigure_limits();

    if (calibrate_time_)
    {
      calibrate_time_offset();
    }

    // Clear the dynamic reconfigure server
    srv_.reset();
    // Spin once to de-register it's services before making a new
    // service next.
    ros::spinOnce();

    if (!urg_ || !ros::ok)
    {
      continue;
    }
    else
    {
      // Set up dynamic reconfigure
      srv_.reset(new dynamic_reconfigure::Server<urg_node::URGConfig>(pnh_));
      // Configure limits (Must do this after creating the urgwidget)
      update_reconfigure_limits();
      srv_->setCallback(boost::bind(&UrgNode::reconfigure_callback, this, _1, _2));
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
      ROS_INFO("Streaming data.");
      // Clear the error count.
      error_count_ = 0;
    }
    catch (std::runtime_error& e)
    {
      ROS_ERROR_THROTTLE(10.0, "Error starting Hokuyo: %s", e.what());
      urg_.reset();
      ros::Duration(1.0).sleep();
      continue;  // Return to top of main loop
    }
    catch (...)
    {
      ROS_ERROR_THROTTLE(10.0, "Unknown error starting Hokuyo");
      urg_.reset();
      ros::Duration(1.0).sleep();
      continue;  // Return to top of main loop
    }

    while (!close_scan_)
    {
      // Don't allow external access during grabbing the scan.
      try
      {
        boost::mutex::scoped_lock lock(lidar_mutex_);
        if (publish_multiecho_)
        {
          const sensor_msgs::MultiEchoLaserScanPtr msg(new sensor_msgs::MultiEchoLaserScan());
          if (urg_->grabScan(msg))
          {
            echoes_pub_.publish(msg);
            echoes_freq_->tick();
          }
          else
          {
            ROS_WARN_THROTTLE(10.0, "Could not grab multi echo scan.");
            device_status_ = urg_->getSensorStatus();
            error_count_++;
          }
        }
        else
        {
          const sensor_msgs::LaserScanPtr msg(new sensor_msgs::LaserScan());
          if (urg_->grabScan(msg))
          {
            laser_pub_.publish(msg);
            laser_freq_->tick();
          }
          else
          {
            ROS_WARN_THROTTLE(10.0, "Could not grab single echo scan.");
            device_status_ = urg_->getSensorStatus();
            error_count_++;
          }
        }
      }
      catch (...)
      {
        ROS_ERROR_THROTTLE(10.0, "Unknown error grabbing Hokuyo scan.");
        error_count_++;
      }

      if (service_yield_)
      {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
        service_yield_ = false;
      }

      // Reestablish conneciton if things seem to have gone wrong.
      if (error_count_ > error_limit_)
      {
        ROS_ERROR_THROTTLE(10.0, "Error count exceeded limit, reconnecting.");
        urg_.reset();
        ros::Duration(2.0).sleep();

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
  diagnostics_thread_ = boost::thread(boost::bind(&UrgNode::updateDiagnostics, this));

  // Start scanning now that everything is configured.
  close_scan_ = false;
  scan_thread_ = boost::thread(boost::bind(&UrgNode::scanThread, this));
}
}  // namespace urg_node
