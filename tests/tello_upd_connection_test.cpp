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
 * @file tello_upd_connection_test.cpp
 *
 * Tello connection test
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#include "tello/socket_udp.hpp"

int main()
{
  tello::SocketUDP tello_command_socket("192.168.10.1", 8889, 8889);
  // Send command and wait for response
  std::string response;
  if (tello_command_socket.send("command", response, 5000)) {
    std::cout << "Received response: " << response << std::endl;
  } else {
    std::cout << "Failed to send command" << std::endl;
  }

  tello::SocketUDP tello_state_socket("0.0.0.0", 8890, 8890);
  int cont = 10;
  // Receive state
  std::string state;
  while (cont > 0) {
    if (tello_state_socket.receive(state, -1)) {
      std::cout << "Received state: " << state << std::endl;
      cont -= 1;
    } else {
      std::cout << "Couldnt received state" << std::endl;
    }
  }

  return 0;
}
