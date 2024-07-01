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
 * @file tello_platform.hpp
 *
 * Implements the functionality and communication with the tello drone.
 *
 * @authors Daniel Fernández Sánchez
 *          Pedro Arias Pérez
 *          Miguel Fernández Cortizas
 *          Rafael Pérez Seguí
 */

#ifndef AS2_PLATFORM_TELLO__AS2_PLATFORM_TELLO_HPP_
#define AS2_PLATFORM_TELLO__AS2_PLATFORM_TELLO_HPP_

#include <string>
#include <memory>
#include <vector>

#include "tello/tello.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "as2_core/aerial_platform.hpp"
#include "as2_core/sensor.hpp"
#include "as2_core/utils/tf_utils.hpp"

namespace as2_tello_platform
{
/**
 * @brief Class that implements the Tello platform.
*/
class TelloPlatform : public as2::AerialPlatform
{
public:
  /**
   * @brief Construct a new Tello Platform object
  */
  explicit TelloPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Tello Platform object
  */
  ~TelloPlatform();

  /**
   * Configure the sensors of the platform.
  */
  void configureSensors() override;

  /**
   * @brief Send command to the drone.
  */
  bool ownSendCommand() override;

  /**
   * @brief Set the arming state of the drone.
  */
  bool ownSetArmingState(bool state) override;

  /**
   * @brief Set the offboard control of the drone.
  */
  bool ownSetOffboardControl(bool offboard) override;

  /**
   * @brief Set the platform control mode.
  */
  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg) override;

  /**
   * @brief Stop the platform flight.
  */
  void ownKillSwitch() override;

  /**
   * @brief Set the platform to hover.
  */
  void ownStopPlatform() override;

  /**
   * @brief Take off the platform.
  */
  bool ownTakeoff() override;

  /**
   * @brief Land the platform.
  */
  bool ownLand() override;

  /**
   * @brief Timer callback to send ping to the drone.
  */
  void pingTimerCallback();

  /**
   * @brief Timer callback to read high frequency sensors data.
  */
  void readStateTimerCallback();

  /**
   * @brief Timer callback to read camera data.
  */
  void readCameraTimerCallback();

private:
  std::shared_ptr<tello::TelloCommandSender> tello_command_sender_ptr_;
  std::unique_ptr<tello::TelloStateReceiver> tello_state_receiver_ptr_;
  std::unique_ptr<tello::TelloCameraManager> tello_camera_manager_ptr_;

  bool connected_ = false;
  double last_state_read_time_ = 0.0;

  // Timers
  rclcpp::TimerBase::SharedPtr ping_timer_;  // Ping timer
  rclcpp::TimerBase::SharedPtr state_read_timer_;  // Low frequency sensor read timer
  rclcpp::TimerBase::SharedPtr video_stream_timer_;  // Video stream timer

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tello_state_pub_;

  // as2 sensors
  std::shared_ptr<as2::sensors::Odometry> odometry_sensor_ptr_;
  std::shared_ptr<as2::sensors::Imu> imu_sensor_ptr_;
  std::shared_ptr<as2::sensors::Battery> battery_ptr_;
  std::shared_ptr<as2::sensors::Barometer> barometer_ptr_;
  std::shared_ptr<as2::sensors::Camera> camera_ptr_;

  // tf handler
  as2::tf::TfHandler tf_handler_;
  std::chrono::nanoseconds tf_timeout_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;

  // IMU data
  std::string imu_frame_id_;

private:
  /**
   * @brief Clamp the speed of the drone.
   *
   * @param vx Speed in x axis.
   * @param vy Speed in y axis.
   * @param vz Speed in z axis.
  */
  void clampSpeed(double & vx, double & vy, double & vz);

  /**
   * @brief Get the speed limit of the drone.
   *
   * @return double Speed limit.
  */
  double getSpeedLimit();

  /**
   * @brief Limit the seed of the drone.
   *
   * @param v Speed.
   * @param min_speed Minimum speed.
   * @param max_speed Maximum speed.
  */
  double limitSeed(const double & v, const double & min_speed, const double & max_speed);

  /**
   * @brief Convert a vector to an array.
   *
   * @param vec Vector to convert.
   * @param array Array to store the data.
   * @return true If the conversion was successful.
   */
  template<std::size_t N>
  bool convertVectorToArray(
    const std::vector<double> & vec, std::array<double, N> & array)
  {
    // Check if the vector has the same size as the array
    if (vec.size() != array.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The vector size is different from the array size. Vector size: %ld, Array size: %ld",
        vec.size(), array.size());
      return false;
    }

    // Copy the vector to the array
    std::memcpy(array.data(), vec.data(), array.size() * sizeof(double));
    return true;
  }
};  // class TelloPlatform

}  // namespace as2_tello_platform

#endif  // AS2_PLATFORM_TELLO__AS2_PLATFORM_TELLO_HPP_
