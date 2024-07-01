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
 * @file socket_udp.cpp
 *
 * Implements socket communication.
 *
 * @authors Rafael Pérez Seguí
 */

#include "tello/socket_udp.hpp"
#include "stdexcept"
#include "cstring"
#include "iostream"

namespace tello
{

SocketUDP::SocketUDP(
  const std::string & server_address, uint16_t client_port, uint16_t server_port)
{
  // Create UDP socket
  socket_file_descriptor_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_file_descriptor_ < 0) {
    throw std::runtime_error("Failed to create socket");
  }

  // Client socket address structure
  memset(&client_address_, 0, sizeof(client_address_));  // Clear address structure
  client_address_.sin_family = AF_INET;    // Set address family
  client_address_.sin_addr.s_addr = htonl(INADDR_ANY);    // Set client address
  client_address_.sin_port = htons(client_port);    // Set client port (server port)
  // Bind the socket to the client address and port
  bindSocket(socket_file_descriptor_, client_address_);

  // Set server address structure
  memset(&server_address_, 0, sizeof(server_address_));    // Clear address structure
  server_address_.sin_family = AF_INET;    // Set address family
  server_address_.sin_port = htons(server_port);    // Set server port
  // Convert server address from text to binary form
  if (inet_pton(AF_INET, server_address.c_str(), &server_address_.sin_addr) <= 0) {
    closeSocket();
    throw std::runtime_error("Invalid address/ Address not supported");
  }

  // Set socket to non-blocking mode
  int flags = fcntl(socket_file_descriptor_, F_GETFL, 0);
  fcntl(socket_file_descriptor_, F_SETFL, flags | O_NONBLOCK);

  // Set internal variables
  socket_address_len_ = sizeof(server_address_);    // Set socket address length

  // Set socket file in read mode
  FD_ZERO(&read_file_descriptor_);
  FD_SET(socket_file_descriptor_, &read_file_descriptor_);
}

SocketUDP::~SocketUDP()
{
  close(socket_file_descriptor_);
}

void SocketUDP::closeSocket()
{
  close(socket_file_descriptor_);
}

bool SocketUDP::send(
  const std::string & message, std::string & response, int timeout_milis)
{
  // Send message to server
  if (sendto(
      socket_file_descriptor_, message.c_str(),
      message.size(), 0, (struct sockaddr *)&server_address_, sizeof(server_address_)) < 0)
  {
    std::cout << "Failed to send message" << std::endl;
    return false;  // Return false if sending failed
  }

  // Wait for response if enabled
  bool received = receive_(response, timeout_milis, server_address_);
  if (timeout_milis != 0) {
    return received;  // Return true if response received
  }

  // Return true indicating successful sending
  return true;
}

bool SocketUDP::receive(
  std::string & response, const int timeout_milis)
{
  return receive_(response, timeout_milis, server_address_);
}

void SocketUDP::bindSocket(
  const int socket_file_descriptor,
  const struct sockaddr_in & server_address)
{
  if (bind(
      socket_file_descriptor, (struct sockaddr *)&server_address,
      sizeof(server_address)) < 0)
  {
    throw std::runtime_error("Failed to bind socket");
  }
}

bool SocketUDP::receive_(
  std::string & response,
  const int timeout_milis,
  const struct sockaddr_in & socket_address)
{
  received_bytes_ = 0;

  // If timeout is negative, set timeout to SOCKET_TIMEOUT
  const int timeout_int = timeout_milis < 0 ? SOCKET_TIMEOUT : timeout_milis;

  // Set timeout structure
  struct timeval timeout;
  timeout.tv_sec = timeout_int / 1000;
  timeout.tv_usec = (timeout_int % 1000) * 1000;

  // Reset the read file descriptor set before each call to select
  fd_set temp_fds = read_file_descriptor_;
  FD_SET(socket_file_descriptor_, &temp_fds);

  // Wait until data is available to read
  int result = select(
    socket_file_descriptor_ + 1, &temp_fds, nullptr, nullptr,
    &timeout);

  // Receive data if available
  if (result > 0 && FD_ISSET(socket_file_descriptor_, &temp_fds)) {
    // Read all data in the buffer and return last message
    socklen_t addr_len = sizeof(socket_address);
    std::string latest_response = "";

    // Clean buffer reading all available data
    while (true) {
      received_bytes_ = recvfrom(
        socket_file_descriptor_, buffer_, sizeof(buffer_) - 1, 0,
        (struct sockaddr *)&socket_address, &addr_len);

      if (received_bytes_ <= 0) {
        break;
      }

      buffer_[received_bytes_] = '\0';      // Null-terminate the received message
      latest_response = std::string(buffer_);      // Save the response
    }

    if (latest_response.size() > 0) {
      response = latest_response;
      return true;
    } else {
      std::cout << "Failed to receive message" << std::endl;
    }
  }

  // Return false if no response received or on timeout
  return false;
}

}  // namespace tello
