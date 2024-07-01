// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file tello_platform.cpp
 *
 * Implements the functionality and communication with the tello drone.
 *
 * @authors Daniel Fernández Sánchez
 *          Pedro Arias Pérez
 *          Miguel Fernández Cortizas
 *          Rafael Pérez Seguí
 */

#include <Eigen/Dense>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "as2_msgs/msg/control_mode.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/frame_utils.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

#include "as2_platform_tello/as2_platform_tello.hpp"

namespace as2_tello_platform
{

TelloPlatform::TelloPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options), tf_handler_(this)
{
  // Get Tello parameters
  std::string tello_ip;
  int port_command, port_command_client, port_state;
  this->declare_parameter<std::string>("tello_ip", "192.168.10.1");
  this->declare_parameter<int>("port_command", 8889);
  this->declare_parameter<int>("port_command_client", 8889);
  this->declare_parameter<int>("port_state", 8890);
  this->get_parameter("tello_ip", tello_ip);
  this->get_parameter("port_command", port_command);
  this->get_parameter("port_command_client", port_command_client);
  this->get_parameter("port_state", port_state);

  // Connect to Tello
  RCLCPP_INFO(
    this->get_logger(), "Connecting to Tello at %s:%d -> :%d",
    tello_ip.c_str(), port_command, port_command_client);
  tello_command_sender_ptr_ = std::make_shared<tello::TelloCommandSender>(
    tello_ip, port_command, port_command_client);
  tello_state_receiver_ptr_ = std::make_unique<tello::TelloStateReceiver>(
    "0.0.0.0", port_state);

  RCLCPP_INFO(
    this->get_logger(), "Tello SDK version: %s",
    tello_command_sender_ptr_->getSDKVersion().c_str());

  // Set State Ports
  tello_command_sender_ptr_->setPort(port_state);
  RCLCPP_INFO(this->get_logger(), "Reading Tello State from :%d", port_state);

  // Get tf timeout
  double tf_timeout_threshold;
  this->declare_parameter<double>("tf_timeout_threshold", 0.1);
  this->get_parameter("tf_timeout_threshold", tf_timeout_threshold);
  tf_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(tf_timeout_threshold));

  // State read timer
  double state_read_freq;
  this->declare_parameter<double>("state_read_freq", 10.0);
  this->get_parameter("state_read_freq", state_read_freq);
  state_read_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / state_read_freq),
    std::bind(&TelloPlatform::readStateTimerCallback, this));

  // Ping timer
  double ping_freq;
  this->declare_parameter<double>("ping_freq", 0.5);
  this->get_parameter("ping_freq", ping_freq);
  ping_timer_ = this->create_timer(
    std::chrono::duration<double>(1.0 / ping_freq),
    std::bind(&TelloPlatform::pingTimerCallback, this));

  // TF frames id
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  imu_frame_id_ = as2::tf::generateTfName(this, "imu");

  // Video stream timer
  bool enable_video_stream;
  this->declare_parameter<bool>("camera.enable", false);
  this->get_parameter("camera.enable", enable_video_stream);
  if (enable_video_stream) {
    double camera_freq;
    this->declare_parameter<double>("camera.freq", 10.0);
    this->get_parameter("camera.freq", camera_freq);
    video_stream_timer_ = this->create_timer(
      std::chrono::duration<double>(1.0 / camera_freq),
      std::bind(&TelloPlatform::readCameraTimerCallback, this));

    // Create camera sensor
    camera_ptr_ = std::make_shared<as2::sensors::Camera>(this, "camera");

    // Enable video stream
    std::string stream_ip;
    uint16_t stream_port;
    this->declare_parameter<std::string>("camera.stream_ip", "0.0.0.0");
    this->declare_parameter<uint16_t>("camera.stream_port", 11111);
    this->get_parameter("camera.stream_ip", stream_ip);
    this->get_parameter("camera.stream_port", stream_port);

    std::string stream_url = "udp://" + stream_ip + ":" + std::to_string(stream_port);

    // Set State Ports
    tello_command_sender_ptr_->setPort(port_state, stream_port);
    RCLCPP_INFO(this->get_logger(), "Reading Camera Stream Port from %s", stream_url.c_str());

    tello_camera_manager_ptr_ = std::make_unique<tello::TelloCameraManager>(
      tello_command_sender_ptr_, stream_url);
  }

  // Configure sensors
  configureSensors();
}

TelloPlatform::~TelloPlatform() {}

// *********************************************************
// ***************** Aerial Platform Methods ***************
// *********************************************************

void TelloPlatform::configureSensors()
{
  // Tello state publisher
  std::string tello_state_topic = "_tello_state";
  this->declare_parameter<std::string>("tello_state_topic", tello_state_topic);
  this->get_parameter("tello_state_topic", tello_state_topic);
  tello_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    tello_state_topic, 10);

  odometry_sensor_ptr_ = std::make_shared<as2::sensors::Odometry>(
    as2_names::topics::sensor_measurements::odom, this);
  imu_sensor_ptr_ = std::make_shared<as2::sensors::Imu>(
    as2_names::topics::sensor_measurements::imu, this);
  battery_ptr_ = std::make_shared<as2::sensors::Battery>(
    as2_names::topics::sensor_measurements::battery, this);
  barometer_ptr_ = std::make_shared<as2::sensors::Barometer>("barometer", this);
}

bool TelloPlatform::ownSendCommand()
{
  const as2_msgs::msg::ControlMode current_control_mode = platform_info_msg_.current_control_mode;

  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double vyaw = 0.0;
  double yaw = 0.0;

  switch (current_control_mode.control_mode) {
    case as2_msgs::msg::ControlMode::UNSET:
    case as2_msgs::msg::ControlMode::HOVER:
      {
        if (!tello_command_sender_ptr_->speedMotionCommand(0, 0, 0, 0)) {
          RCLCPP_ERROR(this->get_logger(), "Error sending Hover control command");
          return false;
        }
        return true;
        break;
      }
    case as2_msgs::msg::ControlMode::POSITION:
      {
        // TODO(RPS98): Test this
        RCLCPP_ERROR(this->get_logger(), "Position control not implemented");
        geometry_msgs::msg::PoseStamped pose_msg = tf_handler_.convert(
          command_pose_msg_, base_link_frame_id_, tf_timeout_);
        yaw = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);

        // TODO(RPS98): Avoid sending the same position multiple times
        if (!tello_command_sender_ptr_->positionMotionCommand(
            pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
            getSpeedLimit()))
        {
          RCLCPP_ERROR(this->get_logger(), "Error sending Position control command");
          return false;
        }
        break;
      }
    case as2_msgs::msg::ControlMode::SPEED:
      {
        vx = command_twist_msg_.twist.linear.x;  // m/s
        vy = command_twist_msg_.twist.linear.y;  // m/s
        vz = command_twist_msg_.twist.linear.z;  // m/s
        vyaw = command_twist_msg_.twist.angular.z;  // rad/s
        break;
      }
    default:
      {
        RCLCPP_ERROR(this->get_logger(), "Control mode not implemented");
        return false;
      }
  }

  switch (current_control_mode.yaw_mode) {
    case as2_msgs::msg::ControlMode::YAW_ANGLE:
      {
        if (!tello_command_sender_ptr_->yawMotion(yaw)) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Tello Platform: Error sending control yaw angle command");
          return false;
        }
      }
    case as2_msgs::msg::ControlMode::YAW_SPEED:
      {
        clampSpeed(vx, vy, vz);
        if (!tello_command_sender_ptr_->speedMotionCommand(vx, vy, vz, vyaw)) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Tello Platform: Error sending control yaw speed command");
          return false;
        }
        break;
      }
    default:
      break;
  }
  return true;
}

bool TelloPlatform::ownSetArmingState(bool state)
{
  return tello_command_sender_ptr_->entrySDKMode();
}

bool TelloPlatform::ownSetOffboardControl(bool offboard)
{
  RCLCPP_DEBUG(this->get_logger(), "Offboard status changed to %d", offboard);
  return true;
}

bool TelloPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  RCLCPP_DEBUG(
    this->get_logger(), "New control mode: %s",
    as2::control_mode::controlModeToString(msg).c_str());
  return true;
}

bool TelloPlatform::ownTakeoff()
{
  for (int i = 0; i < 3; i++) {
    if (tello_command_sender_ptr_->takeoff()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));   // wait 500ms before retry
  }
  return false;
}

bool TelloPlatform::ownLand()
{
  for (int i = 0; i < 3; i++) {
    if (tello_command_sender_ptr_->land()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));   // wait 500ms before retry
  }
  RCLCPP_ERROR(this->get_logger(), "Tello Platform: Error sending land command");
  return false;
}

void TelloPlatform::ownKillSwitch()
{
  tello_command_sender_ptr_->emergency();
}

void TelloPlatform::ownStopPlatform()
{
  RCLCPP_INFO(this->get_logger(), "Stopping platform");
  // Send hover to platform here
  as2_msgs::msg::ControlMode control_mode_msg;
  control_mode_msg.control_mode = as2_msgs::msg::ControlMode::HOVER;
  setPlatformControlMode(control_mode_msg);
}

// **********************************************************
// ******************** CALLBACK METHODS ********************
// **********************************************************

void TelloPlatform::pingTimerCallback()
{
  std::string response;
  tello_command_sender_ptr_->getTime(response);

  // cast response to int
  int time = -1;
  try {
    time = std::stoi(response);
  } catch (std::invalid_argument) {
    RCLCPP_ERROR(this->get_logger(), "Ping response unknow integer: %s", response.c_str());
    return;
  }

  if (time > 0) {
    const auto & clock = this->get_clock();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock, 15000, "Flight Time: %s",
      response.c_str());
  }
}

void TelloPlatform::readStateTimerCallback()
{
  auto state = tello_state_receiver_ptr_->getTelloState();

  if (state.stamp == last_state_read_time_) {
    return;
  }
  rclcpp::Time stamp = this->get_clock()->now();

  // Tello state
  std_msgs::msg::String tello_state_msg;
  tello_state_msg.data = tello_state_receiver_ptr_->getTelloStateString();
  tello_state_pub_->publish(tello_state_msg);

  RCLCPP_DEBUG(
    this->get_logger(), "Tello State: %s", tello_state_msg.data.c_str());

  // Battery
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = stamp;
  battery_msg.percentage = state.bat;
  battery_ptr_->updateData(battery_msg);

  // Barometer
  sensor_msgs::msg::FluidPressure barometer_msg;
  barometer_msg.header.stamp = stamp;
  barometer_msg.fluid_pressure = state.baro;
  barometer_ptr_->updateData(barometer_msg);

  // IMU
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = stamp;
  imu_msg.header.frame_id = imu_frame_id_;
  as2::frame::eulerToQuaternion(
    state.pitch, state.roll, state.yaw, imu_msg.orientation);
  imu_msg.angular_velocity.x = 0.0;
  imu_msg.angular_velocity.y = 0.0;
  imu_msg.angular_velocity.z = 0.0;
  imu_msg.linear_acceleration.x = state.agx;
  imu_msg.linear_acceleration.y = state.agy;
  imu_msg.linear_acceleration.z = state.agz;
  imu_sensor_ptr_->updateData(imu_msg);

  // Odometry
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = odom_frame_id_;
  odom_msg.child_frame_id = base_link_frame_id_;
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = state.h;
  odom_msg.pose.pose.orientation = imu_msg.orientation;

  // Convert linear velocity from world to base_link frame
  Eigen::Vector3d linear_velocity_earth(state.vgx, state.vgy, state.vgz);
  Eigen::Vector3d linear_velocity_baselink =
    as2::frame::transform(imu_msg.orientation, linear_velocity_earth);
  odom_msg.twist.twist.linear.x = linear_velocity_baselink.x();
  odom_msg.twist.twist.linear.y = linear_velocity_baselink.y();
  odom_msg.twist.twist.linear.z = linear_velocity_baselink.z();
  odometry_sensor_ptr_->updateData(odom_msg);
}

void TelloPlatform::readCameraTimerCallback()
{
  auto frame = tello_camera_manager_ptr_->getFrame();
  if (!frame.empty()) {
    camera_ptr_->updateData(frame);
  }
}

double TelloPlatform::getSpeedLimit()
{
  // TODO(RPS98): Test this
  double speed_limit = 1.0;

  if (command_twist_msg_.twist.linear.x != 0.0) {
    speed_limit = std::min(speed_limit, command_twist_msg_.twist.linear.x);
  }
  if (command_twist_msg_.twist.linear.y != 0.0) {
    speed_limit = std::min(speed_limit, command_twist_msg_.twist.linear.y);
  }
  if (command_twist_msg_.twist.linear.z != 0.0) {
    speed_limit = std::min(speed_limit, command_twist_msg_.twist.linear.z);
  }
  return speed_limit;
}

void TelloPlatform::clampSpeed(double & vx, double & vy, double & vz)
{
  vx =
    limitSeed(
    vx, -std::abs(command_twist_msg_.twist.linear.x),
    std::abs(command_twist_msg_.twist.linear.x));
  vy =
    limitSeed(
    vy, -std::abs(command_twist_msg_.twist.linear.y),
    std::abs(command_twist_msg_.twist.linear.y));
  vz =
    limitSeed(
    vz, -std::abs(command_twist_msg_.twist.linear.z),
    std::abs(command_twist_msg_.twist.linear.z));
  return;
}

double TelloPlatform::limitSeed(
  const double & v, const double & min_speed, const double & max_speed)
{
  if (max_speed == 0.0 && min_speed == 0.0) {
    return v;
  }
  return std::clamp(v, min_speed, max_speed);
}

}  // namespace as2_tello_platform
