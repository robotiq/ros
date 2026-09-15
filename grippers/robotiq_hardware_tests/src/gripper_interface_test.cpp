// Copyright (c) 2023 PickNik, Inc.
// Copyright (c) 2026 Robotiq, Inc.
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
//    * Neither the name of the {copyright_holder} nor the names of its
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

//! Manual bench exercise for a real gripper: connect, activate, then drive the
//! fingers through a few open/close cycles at two speeds. Not a unit test —
//! it needs hardware on a serial port, so nothing in CI runs it.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/command.hpp>
#include <Robotiq/gripper/connection_config.hpp>
#include <Robotiq/gripper/status.hpp>
#include <Robotiq/gripper/to_string.hpp>
#include <Robotiq/gripper/wait.hpp>

#include "command_line_utility.hpp"

namespace {
constexpr const char* kComPort = "/dev/ttyUSB0";
constexpr int kBaudRate = 115200;
constexpr double kTimeout = 1.0;
constexpr int kSlaveAddress = 0x09;

// Generous: a full open-to-close sweep at the slowest speed still lands well
// inside it, and overshooting only costs a failed run its error message.
constexpr auto kMotionTimeout = std::chrono::seconds{10};

//! Command a position and block until the gripper stops moving — either it
//! arrived, or it closed on something.
bool moveTo(Robotiq::Gripper& gripper, uint8_t position, uint8_t speed, uint8_t force)
{
   Robotiq::GripperCommand command = Robotiq::GripperCommand::defaults();
   command.action.set(Robotiq::ActionRequestBit::GoTo);
   command.positionRequest = position;
   command.speed = speed;
   command.force = force;
   gripper.setCommand(command);

   // The gripper needs a cycle or two to acknowledge the request before
   // gOBJ leaves Moving; wait for the echo first so this does not read the
   // previous move's "stopped" and return immediately.
   const auto deadline = std::chrono::steady_clock::now() + kMotionTimeout;
   if(!Robotiq::waitUntil([&] { return gripper.getStatus().positionRequestEcho == position; }, deadline))
   {
      std::cout << "  the gripper never echoed the position request" << std::endl;
      return false;
   }
   if(!Robotiq::waitUntil(
         [&] { return gripper.getStatus().gripperStatus.objectDetection() != Robotiq::ObjectDetection::Moving; },
         deadline))
   {
      std::cout << "  the gripper is still moving after " << kMotionTimeout.count() << " s" << std::endl;
      return false;
   }

   const Robotiq::GripperStatus status = gripper.getStatus();
   std::cout << "  stopped at " << static_cast<int>(status.position) << " counts"
             << (status.gripperStatus.objectDetection() == Robotiq::ObjectDetection::AtRequestedPosition
                    ? ""
                    : " (object detected)")
             << std::endl;
   return true;
}
} // namespace

int main(int argc, char* argv[])
{
   CommandLineUtility cli;

   std::string port = kComPort;
   cli.registerHandler("--port", [&port](const char* value) { port = value; }, false);

   int baudrate = kBaudRate;
   cli.registerHandler("--baudrate", [&baudrate](const char* value) { baudrate = std::stoi(value); }, false);

   double timeout = kTimeout;
   cli.registerHandler("--timeout", [&timeout](const char* value) { timeout = std::stod(value); }, false);

   int slave_address = kSlaveAddress;
   cli.registerHandler(
      "--slave-address",
      [&slave_address](const char* value) { slave_address = std::stoi(value, nullptr, 0); },
      false);

   cli.registerHandler("-h", [&]() {
      std::cout << "Usage: ./full_test [OPTIONS]\n"
                << "Options:\n"
                << "  --port VALUE                      Set the com port (default " << kComPort << ")\n"
                << "  --baudrate VALUE                  Set the baudrate (default " << kBaudRate << "bps)\n"
                << "  --timeout VALUE                   Set the read/write timeout (default " << kTimeout << "s)\n"
                << "  --slave-address VALUE             Set the slave address (default " << kSlaveAddress << ")\n"
                << "  -h                                Show this help message\n";
      exit(0);
   });

   if(!cli.parse(argc, argv))
   {
      return 1;
   }

   Robotiq::ConnectionConfig config;
   config.serial.port = port;
   config.serial.baudrate = static_cast<uint32_t>(baudrate);
   config.serial.timeout =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout));
   config.modbusSlaveAddress = static_cast<uint8_t>(slave_address);

   std::cout << "Using the following parameters: " << std::endl;
   std::cout << " - port: " << config.serial.port << std::endl;
   std::cout << " - baudrate: " << config.serial.baudrate << "bps" << std::endl;
   std::cout << " - read/write timeout: " << config.serial.timeout.count() << "ms" << std::endl;
   std::cout << " - slave address: " << static_cast<int>(config.modbusSlaveAddress) << std::endl;

   try
   {
      // Construction opens the link, reads the gripper and starts the
      // exchange cycle; it throws if the gripper does not answer.
      Robotiq::Gripper gripper{config};
      std::cout << "The gripper is connected." << std::endl;

      std::cout << "Activating the gripper..." << std::endl;
      const Robotiq::ActivationResult activation = Robotiq::activate(gripper);
      std::cout << "  " << Robotiq::toString(activation) << std::endl;
      if(activation != Robotiq::ActivationResult::Activated && activation != Robotiq::ActivationResult::AlreadyActive)
      {
         // A latched fault is deliberately not cleared here: the recovery
         // reset releases any grip and sweeps the fingers.
         std::cout << "Cannot continue without an activated gripper." << std::endl;
         return 1;
      }

      constexpr uint8_t kFullSpeed = 0xFF;
      constexpr uint8_t kSlowSpeed = 0x0F;
      constexpr uint8_t kFullForce = 0xFF;

      const struct
      {
         const char* description;
         uint8_t position;
         uint8_t speed;
      } steps[] = {
         {"Closing the gripper...", 0xFF, kFullSpeed},
         {"Opening the gripper...", 0x00, kFullSpeed},
         {"Half closing the gripper...", 0x80, kFullSpeed},
         {"Opening the gripper...", 0x00, kFullSpeed},
         {"Closing the gripper slowly...", 0xFF, kSlowSpeed},
         {"Opening the gripper...", 0x00, kFullSpeed},
      };

      for(const auto& step : steps)
      {
         std::cout << step.description << std::endl;
         if(!moveTo(gripper, step.position, step.speed, kFullForce))
         {
            return 1;
         }
      }
   }
   catch(const std::exception& e)
   {
      std::cout << "Failed to communicate with the gripper: " << e.what() << std::endl;
      return 1;
   }

   return 0;
}
