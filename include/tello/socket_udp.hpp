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
 * @file socket_udp.hpp
 *
 * Implements socket communication.
 *
 * @authors Rafael Pérez Seguí
 */

#ifndef TELLO__SOCKET_UDP_HPP_
#define TELLO__SOCKET_UDP_HPP_

#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string>
#include <optional>
#include <chrono>

#define SOCKET_TIMEOUT 10000  // Socket timeout in milliseconds

namespace tello
{

/**
 * Manage a UDP connection with a server, receiving and sending messages.
*/
class SocketUDP
{
public:
  /**
   * Constructor to create a UDP connection with a server.
   *
   * @param server_address The server address.
   * @param server_port The server port.
   * @param command_address Default address for sending commands.
   * @param command_port Default port for sending commands.
  */
  SocketUDP(
    const std::string & server_address, uint16_t client_port, uint16_t server_port);

  /**
   * Destructor to close the socket file descriptor.
  */
  ~SocketUDP();

  /**
   * Close the socket file descriptor.
  */
  void closeSocket();

  /**
   * Send a message to the server and optionally wait for a response.
   *
   * @param message The message to send.
   * @param response The response received reference.
   * @param timeout_milis Timeout in milliseconds.
   *                      If zero, dont wait for a response.
   *                      If negative, wait indefinitely.
   *                      Default: 0
   * @return True if the message was sent successfully, false otherwise.
   *         If waiting for a response, true if a response was received, false otherwise.
  */
  bool send(
    const std::string & message, std::string & response, int timeout_milis = 0);

  /**
   * Receive a message from the server.
   *
   * @param response The response received reference.
   * @param timeout_milis Timeout in milliseconds to wait for a response.
   *                      If zero, dont wait for a response.
   *                      If negative, wait indefinitely.
   * @return True if a response was received, false otherwise.
  */
  bool receive(
    std::string & response, const int timeout_milis = 0);

private:
  // Server socket
  int socket_file_descriptor_;  // Socket file descriptor
  struct sockaddr_in client_address_;  // Client socket address structure
  struct sockaddr_in server_address_;  // Server socket address structure
  struct sockaddr_in command_address_;  // Command socket address structure

  // Receive buffer
  char buffer_[1024];  // Buffer to store received data
  ssize_t received_bytes_;  // Number of received bytes

  // Internal variables
  socklen_t socket_address_len_;  // Length of the socket address structure
  fd_set read_file_descriptor_;    // File descriptor set for reading

private:
  /**
   * Bind the socket to the server address and port.
  */
  void bindSocket(const int socket_file_descriptor, const struct sockaddr_in & server_address);

  /**
   * Receive a message from the server.
   *
   * @param response The response received reference.
   * @param timeout_milis Timeout in milliseconds to wait for a response.
   *                      If zero, dont wait for a response.
   *                      If negative, wait indefinitely.
   * @param socket_address The socket address structure to store the sender address.
   * @return True if a response was received, false otherwise.
  */
  bool receive_(
    std::string & response, const int timeout_milis, const struct sockaddr_in & socket_address);
};  // class SocketUDP

}  // namespace tello

#endif  // TELLO__SOCKET_UDP_HPP_
