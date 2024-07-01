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
 * @file tello.cpp
 *
 * Implements socket communication with the tello drone.
 *
 * @authors Daniel Fernández Sánchez
 *          Rafael Pérez Seguí
 */

#include "tello/tello.hpp"
#include <sys/socket.h>
#include "stdexcept"

namespace tello
{

TelloStateReceiver::TelloStateReceiver(const std::string & tello_ip, const int port_state)
{
  udp_socket_state_ = std::make_unique<SocketUDP>("0.0.0.0", port_state, port_state);

  // Get the state
  std::string tello_state_msg = "";
  udp_socket_state_->receive(tello_state_msg, -1);  // blocking
  parseState(tello_state_msg);

  // Get the connection time
  connection_time_ = std::chrono::system_clock::now();
}

TelloStateReceiver::~TelloStateReceiver()
{
  udp_socket_state_->closeSocket();
}

const TelloState & TelloStateReceiver::getTelloState()
{
  std::string tello_state_msg = "";

  if (!udp_socket_state_->receive(tello_state_msg, 0)) {
    return tello_state_;
  }

  if (!parseState(tello_state_msg)) {
    return tello_state_;
  }
  tello_state_raw_ = tello_state_msg;

  // Calculate the elapsed time since connection
  std::chrono::time_point<std::chrono::system_clock> current_time =
    std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_time = current_time - connection_time_;
  tello_state_.stamp = elapsed_time.count();

  // Parse the state
  tello_state_.pitch = state_["pitch"] * M_PI / 180.0;
  tello_state_.roll = state_["roll"] * M_PI / 180.0;
  tello_state_.yaw = state_["yaw"] * M_PI / 180.0;

  tello_state_.vgx = state_["vgx"] / 100.0;
  tello_state_.vgy = state_["vgy"] / 100.0;
  tello_state_.vgz = state_["vgz"] / 100.0;

  tello_state_.templ = state_["templ"];
  tello_state_.temph = state_["temph"];
  tello_state_.tof = state_["tof"];
  tello_state_.h = state_["h"] / 100.0;
  tello_state_.bat = static_cast<int>(state_["bat"]);
  tello_state_.baro = state_["baro"] / 100.0;
  tello_state_.time = state_["time"];

  // TODO(RPS98): Check units (m/s^2)¿?
  tello_state_.agx = state_["agx"] * 9.81 / 1000.0;
  tello_state_.agy = state_["agy"] * 9.81 / 1000.0;
  tello_state_.agz = state_["agz"] * 9.81 / 1000.0;
  return tello_state_;
}

const std::string & TelloStateReceiver::getTelloStateString()
{
  return tello_state_raw_;
}

static std::vector<std::string> split(const std::string & target, char c)
{
  std::string temp;
  std::stringstream stringstream{target};
  std::vector<std::string> result;

  while (std::getline(stringstream, temp, c)) {
    result.emplace_back(temp);
  }

  return result;
}

bool TelloStateReceiver::parseState(const std::string & data)
{
  if (data.empty()) {
    return false;
  }

  // Split the data by ';'
  std::vector<std::string> values = split(data, ';');

  // Iterate over each key-value pair
  for (const std::string & pair : values) {
    // Skip empty pairs
    if (pair.empty()) {
      continue;
    }

    // Tello v1.3 ends the state message with "\r\n" -> continue
    if (pair == "\r\n") {
      continue;
    }

    // Split the pair by ':'
    std::vector<std::string> keyValue = split(pair, ':');
    // Fails if pairs that do not contain exactly one ':'
    if (keyValue.size() != 2) {
      std::cout << "Invalid key-value pair: " << pair << std::endl;
      return false;
    }

    std::string key = keyValue[0];
    std::string value = keyValue[1];

    try {
      // Convert value to double and store in the state map
      state_[key] = std::stod(value);
    } catch (const std::invalid_argument & e) {
      // If conversion fails, return false
      std::cout << "Invalid argument: " << e.what() << std::endl;
      return false;
    } catch (const std::out_of_range & e) {
      // If conversion is out of range, return false
      std::cout << "Out of range: " << e.what() << std::endl;
      return false;
    }
  }

  // Successfully parsed all key-value pairs
  return true;
}

TelloCommandSender::TelloCommandSender(
  const std::string & tello_ip, const int port_command, const int port_command_client)
{
  // Create UDP socket to send commands
  udp_socket_command_ = std::make_unique<SocketUDP>(tello_ip, port_command_client, port_command);

  // Send command to enter SDK mode
  if (!entrySDKMode()) {
    throw std::runtime_error("Error: Entering SDK mode");
  }

  // Get SDK version
  std::string tello_version_response = "";
  if (!askSDKVersion(tello_version_response)) {
    throw std::runtime_error("Error: Getting SDK version");
  }
  tello_version_ = tello_version_response;
}

TelloCommandSender::~TelloCommandSender()
{
  udp_socket_command_->closeSocket();
}

const std::string & TelloCommandSender::getSDKVersion()
{
  return tello_version_;
}


bool TelloCommandSender::getTime(std::string & response)
{
  return sendReadCommands("time?", response, -1);
}

bool TelloCommandSender::entrySDKMode()
{
  return sendControlCommand("command");
}

bool TelloCommandSender::setPort(const int port_state, const int port_camera)
{
  return sendControlCommand(
    "port " + std::to_string(port_state) + " " +
    std::to_string(port_camera));
}

bool TelloCommandSender::takeoff()
{
  if (!sendControlCommand("takeoff")) {
    return false;
  }
  // Wait 0.5s for the drone odometry
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}

bool TelloCommandSender::land()
{
  if (!sendControlCommand("land")) {
    return false;
  }
  // Wait 5s for the drone to land
  // std::this_thread::sleep_for(std::chrono::seconds(5));
  return true;
}

bool TelloCommandSender::emergency()
{
  return sendControlCommand("emergency");
}

bool TelloCommandSender::positionMotionCommand(
  const double x, const double y, const double z,
  const double speed)
{
  int x_cm = static_cast<int>(x * 100);
  int y_cm = static_cast<int>(y * 100);
  int z_cm = static_cast<int>(z * 100);
  int speed_cms = static_cast<int>(speed * 100);

  x_cm = std::clamp(x_cm, -500, 500);
  y_cm = std::clamp(y_cm, -500, 500);
  z_cm = std::clamp(z_cm, -500, 500);
  speed_cms = std::clamp(speed_cms, 10, 100);

  std::string msg = "go " + std::to_string(x_cm) + " " + std::to_string(y_cm) + " " +
    std::to_string(z_cm) + " " + std::to_string(speed_cms);
  return sendControlCommand(msg, 0);
}

bool TelloCommandSender::speedMotionCommand(
  const double x, const double y, const double z,
  const double yaw)
{
  int x_cm = static_cast<int>(x * 100);
  int y_cm = static_cast<int>(y * 100);
  int z_cm = static_cast<int>(z * 100);
  // TODO(RPS98): Convert from max yaw speed of tello to rad/s
  int yaw_deg = static_cast<int>(yaw * 100);

  x_cm = std::clamp(x_cm, -100, 100);
  y_cm = std::clamp(-y_cm, -100, 100);
  z_cm = std::clamp(z_cm, -100, 100);
  yaw_deg = std::clamp(-yaw_deg, -100, 100);

  std::string msg = "rc " + std::to_string(y_cm) + " " + std::to_string(x_cm) + " " +
    std::to_string(z_cm) + " " + std::to_string(yaw_deg);

  auto command_sent = sendControlCommand(msg, 0);
  return command_sent;
}

bool TelloCommandSender::yawMotion(double yaw)
{
  int yaw_deg = static_cast<int>(yaw * 180.0 / M_PI);
  // Wrap yaw between 0 and 360
  yaw_deg = yaw_deg % 360;

  std::string msg = "";
  if (yaw_deg < 0) {
    msg = "ccw " + std::to_string(abs(yaw_deg) / 10);
  } else {
    msg = "cw " + std::to_string(abs(yaw_deg) / 10);
  }
  return sendControlCommand(msg, 0);
}

bool TelloCommandSender::sendCameraCommand(
  const bool enable, const int timeout_milis)
{
  if (enable) {
    return sendControlCommand("streamon", timeout_milis);
  }
  return sendControlCommand("streamoff", timeout_milis);
}

bool TelloCommandSender::askSDKVersion(std::string & response)
{
  return sendReadCommands("sdk?", response, -1);
}

bool TelloCommandSender::sendControlCommand(
  const std::string & command, const int timeout_milis)
{
  std::string response = "";
  bool sent = udp_socket_command_->send(command, response, timeout_milis);
  if (timeout_milis != 0) {
    bool is_ok_response = response.find("ok") != std::string::npos;
    return sent && response.find("ok") != std::string::npos;
  }
  return sent;
}

bool TelloCommandSender::sendReadCommands(
  const std::string & command, std::string & response,
  const int timeout_milis)
{
  return udp_socket_command_->send(command, response, timeout_milis);
}

TelloCameraManager::TelloCameraManager(
  std::shared_ptr<TelloCommandSender> tello_command_sender,
  const std::string & stream_url)
: tello_command_sender_(tello_command_sender)
{
  if (!setVideoStream(true, stream_url)) {
    std::cerr << "Failed to initialize video stream" << std::endl;
  }
}

TelloCameraManager::~TelloCameraManager()
{
  if (camera_enabled_) {
    video_stream_capture_.release();
  }
}

bool TelloCameraManager::setVideoStream(
  const bool enable, const std::string & stream_url)
{
  if (!enable) {
    // If the camera is enabled, release the video stream
    if (camera_enabled_) {
      video_stream_capture_.release();
    }
    camera_enabled_ = false;
    return tello_command_sender_->sendCameraCommand(false);
  }
  bool response = tello_command_sender_->sendCameraCommand(true);

  if (response) {
    video_stream_capture_.open(stream_url, cv::CAP_FFMPEG);
    if (!video_stream_capture_.isOpened()) {
      std::cerr << "Failed to open video stream: " << stream_url << std::endl;
      camera_enabled_ = false;
      return false;
    }
    camera_enabled_ = true;
    video_stream_frame_ = cv::Mat();
  } else {
    std::cerr << "Failed to send camera command" << std::endl;
  }
  return response;
}

const cv::Mat & TelloCameraManager::getFrame()
{
  if (camera_enabled_) {
    video_stream_capture_ >> video_stream_frame_;
    if (video_stream_frame_.empty()) {
      std::cerr << "Failed to capture frame" << std::endl;
    }
  }
  return video_stream_frame_;
}
}  // namespace tello
