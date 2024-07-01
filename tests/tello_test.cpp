// Copyright 2023 Universidad Politécnica de Madrid
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
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
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
 * @file tello_test.cpp
 *
 * Tello test
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */


#include <iostream>
#include <memory>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "as2_platform_tello/as2_platform_tello.hpp"
#include "as2_core/mocks/aerial_platform/mock_aerial_platform.hpp"
#include "as2_core/mocks/executor_thread_util/executor_thread_util.hpp"
#include <as2_msgs/msg/platform_state_machine_event.hpp>

namespace as2_tello_platform
{
std::shared_ptr<TelloPlatform> get_node(
  const std::string & name_space = "test_tello_platform")
{
  const std::string package_path =
    ament_index_cpp::get_package_share_directory("as2_platform_tello");
  const std::string control_modes_config_file = package_path + "/config/control_modes.yaml";
  const std::string platform_config_file = package_path + "/config/platform_default.yaml";
  const std::string camera_calibration = package_path + "/config/camera_calibration.yaml";

  std::vector<std::string> node_args = {
    "--ros-args",
    "-r",
    "__ns:=/" + name_space,
    "-p",
    "namespace:=" + name_space,
    "-p",
    "control_modes_file:=" + control_modes_config_file,
    "--params-file",
    platform_config_file,
    "--params-file",
    camera_calibration,
  };

  rclcpp::NodeOptions node_options;
  node_options.arguments(node_args);

  return std::make_shared<TelloPlatform>(node_options);
}

void spinForTime(
  double seconds, rclcpp::executors::SingleThreadedExecutor & executor,
  std::shared_ptr<as2::mock::PlatformMockNode> test_node)
{
  // Current ros2 time
  auto start_time = test_node->getTime();
  while ((test_node->getTime() - start_time).seconds() < seconds) {
    // Spin
    executor.spin_some();

    // Print state
    test_node->printState(seconds * 0.5);

    // Sleep for 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return;
}

void spinForTime(
  double seconds,
  std::shared_ptr<as2::mock::PlatformMockNode> test_node)
{
  // Current ros2 time
  auto start_time = test_node->getTime();
  while ((test_node->getTime() - start_time).seconds() < seconds) {
    // Print state
    test_node->printState(seconds * 0.5);

    // Sleep for 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return;
}

int platform_test()
{
  std::string ros_namespace = "test_tello_platform";
  auto test_node = std::make_shared<as2::mock::PlatformMockNode>(ros_namespace);
  auto tello_node = get_node(ros_namespace);

  // Executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Add nodes to executor
  executor->add_node(test_node);
  executor->add_node(tello_node);

  // Executor thread
  as2::mock::ExecutorThreadUtil executor_thread_util(executor, 200.0);
  executor_thread_util.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Enable test command send timer
  test_node->setCommandSendTimerState(true, 100.0);

  // Wait 2 seconds
  spinForTime(2.0, test_node);

  // Takeoff
  RCLCPP_INFO(test_node->get_logger(), "Taking off");
  if (!test_node->takeoffPlatform(false)) {return 1;}

  // Set control mode to speed
  RCLCPP_INFO(test_node->get_logger(), "Setting control mode to speed");
  as2_msgs::msg::ControlMode desired_control_mode;
  desired_control_mode.control_mode = as2_msgs::msg::ControlMode::SPEED;
  desired_control_mode.yaw_mode = as2_msgs::msg::ControlMode::YAW_SPEED;
  desired_control_mode.reference_frame = as2_msgs::msg::ControlMode::LOCAL_ENU_FRAME;
  if (!test_node->setControlMode(desired_control_mode, false)) {
    return 1;
  }

  // Send command
  RCLCPP_INFO(test_node->get_logger(), "Sending command");
  geometry_msgs::msg::TwistStamped desired_twist;
  desired_twist.header.frame_id = as2::tf::generateTfName(
    test_node->get_namespace(), "base_link");
  desired_twist.twist.linear.x = 0.4;
  desired_twist.twist.linear.y = 0.0;
  desired_twist.twist.linear.z = 0.1;
  desired_twist.twist.angular.x = 0.0;
  desired_twist.twist.angular.y = 0.0;
  desired_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(desired_twist);
  spinForTime(2.0, test_node);

  // Send command
  RCLCPP_INFO(test_node->get_logger(), "Sending command");
  desired_twist.header.frame_id = as2::tf::generateTfName(
    test_node->get_namespace(), "base_link");
  desired_twist.twist.linear.x = -0.3;
  desired_twist.twist.linear.y = 0.0;
  desired_twist.twist.linear.z = 0.0;
  desired_twist.twist.angular.x = 0.0;
  desired_twist.twist.angular.y = 0.0;
  desired_twist.twist.angular.z = 0.0;
  test_node->setCommandTwist(desired_twist);
  spinForTime(2.0, test_node);

  // Send hover command
  RCLCPP_INFO(test_node->get_logger(), "Setting control mode to hover");
  as2_msgs::msg::ControlMode desired_hover_control_mode;
  desired_hover_control_mode.control_mode = as2_msgs::msg::ControlMode::HOVER;
  desired_hover_control_mode.yaw_mode = as2_msgs::msg::ControlMode::UNSET;
  desired_hover_control_mode.reference_frame = as2_msgs::msg::ControlMode::UNSET;
  if (!test_node->setControlMode(desired_hover_control_mode, false)) {
    return 1;
  }
  spinForTime(2.0, test_node);

  // Land
  RCLCPP_INFO(test_node->get_logger(), "Landing");
  if (!test_node->landPlatform(false)) {return 1;}

  // Clean
  executor_thread_util.stop();
  executor->remove_node(test_node);
  executor->remove_node(tello_node);

  return 0;
}
}  // namespace as2_tello_platform


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto result = as2_tello_platform::platform_test();

  rclcpp::shutdown();
  return result;
}
