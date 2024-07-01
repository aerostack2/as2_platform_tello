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
 * @file tello.hpp
 *
 * Implements socket communication with the tello drone.
 *
 * @authors Daniel Fernández Sánchez
 *          Rafael Pérez Seguí
 */

#ifndef TELLO__TELLO_HPP_
#define TELLO__TELLO_HPP_

#include <memory>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "socket_udp.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#define COMMAND_SEND_TIMEOUT 220  // Timeout to send a command (ms)

namespace tello
{

struct coordinates
{
  double x = 0;
  double y = 0;
  double z = 0;
};

struct TelloState
{
  double stamp = 0;  // Time stamp (s) since connection
  double pitch = 0;  // Pitch angle (rad)
  double roll = 0;  // Roll angle (rad)
  double yaw = 0;  // Yaw angle (rad)
  double vgx = 0;  // Velocity in x (m/s)
  double vgy = 0;  // Velocity in y (m/s)
  double vgz = 0;  // Velocity in z (m/s)
  double templ = 0;  // Lower temperature (°C)
  double temph = 0;  // Higher temperature (°C)
  double tof = 0;  // Time of flight (s)
  double h = 0;  // Height (m)
  double bat = 0;  // Battery (%)
  double baro = 0;  // Barometer (m)
  double time = 0;  // Time (s)
  double agx = 0;  // Linear acceleration in x (m/s²) TODO(RPS98): Check units
  double agy = 0;  // Linear acceleration in y (m/s²) TODO(RPS98): Check units
  double agz = 0;  // Linear acceleration in z (m/s²) TODO(RPS98): Check units
};


class TelloStateReceiver
{
public:
  /**
   * @brief Construct a new TelloStateReceiver object
   * Note: Send a command before calling this constructor to make Tello start sending state
   *
   * @param state_ip IP of the host to receive the state
   * @param port_state Port of the host to receive the state
  */
  TelloStateReceiver(
    const std::string & state_ip = "0.0.0.0",
    const int port_state = 8890);

  /**
   * @brief Destroy the TelloStateReceiver object
  */
  ~TelloStateReceiver();

  /**
   * @brief Get the drone state non-blocking.
   * If not received, return the last state received
   *
   * @return TelloState State of the drone
  */
  const TelloState & getTelloState();

  /**
   * @brief Get the drone state as a string
   * Note, getTelloState() method must be called before this method to update the state
   *
   * @return std::string State of the drone as a string
  */
  const std::string & getTelloStateString();

private:
  // UPD sockets
  std::unique_ptr<SocketUDP> udp_socket_state_;  // UDP socket to receive state

  // Tello state
  std::string tello_state_raw_;  // Raw state received from the tello drone
  TelloState tello_state_;  // State of the tello drone

  // Hash map to store the state of the tello drone
  std::unordered_map<std::string, double> state_{
    {"pitch", 0.0}, {"roll", 0.0}, {"yaw", 0.0}, {"vgx", 0.0}, {"vgy", 0.0}, {"vgz", 0.0},
    {"templ", 0.0}, {"temph", 0.0}, {"tof", 0.0}, {"h", 0.0}, {"bat", 0.0}, {"baro", 0.0},
    {"time", 0.0}, {"agx", 0.0}, {"agy", 0.0}, {"agz", 0.0}};

  // Connection time stamp
  std::chrono::time_point<std::chrono::system_clock> connection_time_;

private:
  /**
   * @brief Parse the state message received from the tello drone
   *
   * @param data Message received from the tello drone
   * @return true if the message is parsed
  */
  bool parseState(const std::string & data);
};

class TelloCommandSender
{
public:
  /**
   * @brief Construct a new TelloCommandSender object
   *
   * @param tello_ip IP of the tello drone
   * @param port_command Port of the client to send the command
  */
  TelloCommandSender(
    const std::string & tello_ip,
    const int port_command = 8890,
    const int port_command_client = 8890);

  /**
   * @brief Destroy the TelloCommandSender object
  */
  ~TelloCommandSender();

  /**
   * @brief Get the SDK version from the tello drone
   *
   * @param response SDK version received from the tello drone
   * @return true if the command was sent
  */
  const std::string & getSDKVersion();

  /**
   * @brief Get the time from the tello drone
   *
   * @param response Time received from the tello drone
   * @return true if the command was sent
  */
  bool getTime(std::string & response);

  /**
   * @brief Enter SDK mode
   *
   * @return true if the command was sent
  */
  bool entrySDKMode();

  /**
   * @brief Set State and Camera Stream Ports
   *
   * @return true if command was sent
   */
  bool setPort(
    const int port_state = 8890,
    const int port_camera = 11111);

  /**
   * @brief Takeoff the tello drone
   *
   * @return true if the command was sent
  */
  bool takeoff();

  /**
   * @brief Land the tello drone
   *
   * @return true if the command was sent
  */
  bool land();

  /**
   * @brief Emergency stop the tello drone
   *
   * @return true if the command was sent
  */
  bool emergency();

  /**
   * @brief Send position command to the tello drone
   *
   * @param x X position in base link frame (m)
   * @param y Y position in base link frame (m)
   * @param z Z position in base link frame (m)
   * @param speed Speed of the drone (m/s)
   * @return true if the command was sent
  */
  bool positionMotionCommand(const double x, const double y, const double z, const double speed);

  /**
   * @brief Send speed command to the tello drone
   *
   * @param vx Velocity in x (m/s)
   * @param vy Velocity in y (m/s)
   * @param vz Velocity in z (m/s)
   * @param yaw Yaw angle (rad)
   * @return true if the command was sent
  */
  bool speedMotionCommand(const double vx, const double vy, const double vz, const double yaw);

  /**
   * @brief Send yaw motion command to the tello drone
   *
   * @param yaw Yaw angle (rad)
   * @return true if the command was sent
  */
  bool yawMotion(double yaw);

  /**
 * @brief Send a control command to the tello drone for enabling/disabling the camera
 *
 * @param enable Command to enable/disable the camera
 * @param timeout_milis Timeout in milliseconds.
 *                      If zero, dont wait for a response.
 *                      If negative, wait indefinitely.
 *                      Default: -1 (wait indefinitely)
 * @return true if the command was sent
 *         If waiting for a response, true if a response was received and was 'ok', false otherwise.
*/
  bool sendCameraCommand(
    const bool enable, const int timeout_milis = -1);

private:
  // UPD sockets
  std::unique_ptr<SocketUDP> udp_socket_command_;  // UDP socket to send commands

  // Tello info
  std::string tello_version_ = "";  // Tello version

  /**
   * @brief Ask the SDK version to the tello drone
   *
   * @param response SDK version received from the tello drone
   * @return true if the command was sent
  */
  bool askSDKVersion(std::string & response);

  /**
   * @brief Send a control command to the tello drone
   *
   * @param command Command to send
   * @param timeout_milis Timeout in milliseconds.
   *                      If zero, dont wait for a response.
   *                      If negative, wait indefinitely.
   *                      Default: -1 (wait indefinitely)
   * @return true if the command was sent
   *         If waiting for a response, true if a response was received and was 'ok', false otherwise.
  */
  bool sendControlCommand(
    const std::string & command, const int timeout_milis = -1);

  /**
   * @brief Send a read command to the tello drone
   *
   * @param command Command to send
   * @param response Response received from the tello drone
   * @param timeout_milis Timeout in milliseconds.
   *                      If zero, dont wait for a response.
   *                      If negative, wait indefinitely.
   *                      Default: -1 (wait indefinitely)
   * @return true if the command was sent
   *         If waiting for a response, true if a response was received, false otherwise.
  */
  bool sendReadCommands(
    const std::string & command, std::string & response,
    const int timeout_milis = -1);
};

class TelloCameraManager
{
public:
  /**
   * @brief Construct a new TelloCameraManager object
   *
   * @param tello_ip IP of the tello drone
   * @param port_command Port of the client to send the command
  */
  TelloCameraManager(
    std::shared_ptr<TelloCommandSender> tello_command_sender,
    const std::string & stream_url = "udp://0.0.0.0:11111");

  /**
   * @brief Destroy the TelloCameraManager object
  */
  ~TelloCameraManager();

  /**
   * @brief Enable/Disable the camera video stream
   *
   * @param enable Enable/Disable the camera
   * @param stream_url URL of the video stream
   * @return true if was correctly set
  */
  bool setVideoStream(
    const bool enable, const std::string & stream_url);

  /**
   * @brief Get the frame from the video stream
   *
   * @return cv::Mat Frame from the video stream
  */
  const cv::Mat & getFrame();

private:
  // Tello command sender
  std::shared_ptr<TelloCommandSender> tello_command_sender_;  // Tello command sender

  // Camera
  bool camera_enabled_ = false;  // Camera enabled
  cv::VideoCapture video_stream_capture_;  // Video stream capture
  cv::Mat video_stream_frame_;  // Video stream frame
};

}  // namespace tello

#endif  // TELLO__TELLO_HPP_
