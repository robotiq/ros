// Copyright (c) 2022 PickNik, Inc.
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
//    * Neither the name of the copyright holder nor the names of its
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

#include <robotiq_driver/hardware_parameters.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>

namespace robotiq_driver {
namespace {
constexpr const char* kComPortParam = "COM_port";
constexpr const char* kBaudrateParam = "baudrate";
constexpr const char* kTimeoutParam = "timeout";
constexpr const char* kSlaveAddressParam = "slave_address";
constexpr const char* kConnectionFrequencyParam = "connection_frequency";
constexpr const char* kClosedPositionParam = "gripper_closed_position";
constexpr const char* kMaxSpeedParam = "gripper_max_speed";
constexpr const char* kMinSpeedParam = "gripper_min_speed";
constexpr const char* kMaxForceParam = "gripper_max_force";
constexpr const char* kSpeedMultiplierParam = "gripper_speed_multiplier";
constexpr const char* kForceMultiplierParam = "gripper_force_multiplier";
constexpr const char* kActivationTimeoutParam = "activation_timeout";
constexpr const char* kUseDummyParam = "use_dummy";

//! Whether \p text reads as "no". Anything else — including a bare "true" or
//! any typo — selects the fake gripper, so the spellings people actually write
//! have to be recognised here.
bool isFalsey(std::string text)
{
   std::transform(text.begin(), text.end(), text.begin(), [](unsigned char letter) {
      return static_cast<char>(std::tolower(letter));
   });
   return text.empty() || text == "0" || text == "false" || text == "no" || text == "off";
}

//! Look a parameter up and convert it with \p parse. An absent parameter
//! keeps \p fallback silently; a present but unparsable one keeps it loudly.
template <typename T, typename Parse>
T parameterOr(const hardware_interface::HardwareInfo& info,
              const rclcpp::Logger& logger,
              const char* name,
              T fallback,
              Parse parse)
{
   const auto entry = info.hardware_parameters.find(name);
   if(entry == info.hardware_parameters.end())
   {
      return fallback;
   }
   try
   {
      return parse(entry->second);
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(logger, "Ignoring malformed '%s' parameter '%s': %s", name, entry->second.c_str(), e.what());
      return fallback;
   }
}

//! Parse \p text in \p base, rejecting the trailing characters std::stoull
//! silently ignores: "115200bps" would otherwise read as 115200 and "1.152e5"
//! as 1.
uint64_t asWholeNumber(const std::string& text, int base = 10)
{
   std::size_t consumed = 0;
   const uint64_t value = std::stoull(text, &consumed, base);
   if(consumed != text.size())
   {
      throw std::invalid_argument("expected a whole number");
   }
   return value;
}

//! The range libserialport accepts. Out of it the port still opens and every
//! exchange then times out, which reads as a dead gripper rather than a typo.
uint32_t asBaudrate(const std::string& text)
{
   const uint64_t value = asWholeNumber(text);
   if(value == 0 || value > 1000000)
   {
      throw std::out_of_range("baudrate must be between 1 and 1000000");
   }
   return static_cast<uint32_t>(value);
}

double asDouble(const std::string& text)
{
   return std::stod(text);
}

//! For a scale the register arithmetic divides by.
double asPositiveDouble(const std::string& text)
{
   const double value = std::stod(text);
   if(!std::isfinite(value) || value <= 0.0)
   {
      throw std::invalid_argument("expected a finite value above zero");
   }
   return value;
}
} // namespace

GripperParameters parseParameters(const hardware_interface::HardwareInfo& info, const rclcpp::Logger& logger)
{
   GripperParameters parameters;

   parameters.connection.serial.port =
      parameterOr<std::string>(info, logger, kComPortParam, kComPortDefault, [](const std::string& text) {
         return text;
      });
   parameters.connection.serial.baudrate =
      parameterOr<uint32_t>(info, logger, kBaudrateParam, kBaudrateDefault, asBaudrate);
   // The URDF spells the timeout in seconds; SerialConfig wants milliseconds.
   parameters.connection.serial.timeout =
      parameterOr<std::chrono::milliseconds>(info, logger, kTimeoutParam, kTimeoutDefault, [](const std::string& text) {
         return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(std::stod(text)));
      });
   // Base 0: "0x9" as the manual prints it, and a bare "9" as the decimal it
   // looks like. Rejecting a value that does not fit the byte keeps a typo from
   // quietly addressing a different device.
   parameters.connection.modbusSlaveAddress =
      parameterOr<uint8_t>(info, logger, kSlaveAddressParam, kSlaveAddressDefault, [](const std::string& text) {
         const auto address = asWholeNumber(text, 0);
         if(address > 0xFF)
         {
            throw std::out_of_range("slave_address does not fit in a byte");
         }
         return static_cast<uint8_t>(address);
      });
   parameters.connection.connectionFrequency =
      parameterOr<double>(info, logger, kConnectionFrequencyParam, kConnectionFrequencyDefault, asDouble);

   // No default: the joint mapping is meaningless without the gripper's
   // closed angle, and guessing one would silently mis-scale every command.
   // Zero and non-finite are the two it cannot divide by; negative is fine, and
   // is how a joint that closes in the negative direction is described.
   parameters.closed_position = std::stod(info.hardware_parameters.at(kClosedPositionParam));
   if(!std::isfinite(parameters.closed_position) || parameters.closed_position == 0.0)
   {
      throw std::invalid_argument("gripper_closed_position must be a non-zero, finite joint value");
   }

   parameters.max_speed = parameterOr<double>(info, logger, kMaxSpeedParam, parameters.max_speed, asPositiveDouble);
   parameters.min_speed = parameterOr<double>(info, logger, kMinSpeedParam, parameters.min_speed, asPositiveDouble);
   parameters.max_force = parameterOr<double>(info, logger, kMaxForceParam, parameters.max_force, asPositiveDouble);
   parameters.speed_multiplier =
      parameterOr<double>(info, logger, kSpeedMultiplierParam, parameters.speed_multiplier, asDouble);
   parameters.force_multiplier =
      parameterOr<double>(info, logger, kForceMultiplierParam, parameters.force_multiplier, asDouble);
   parameters.activation_timeout = parameterOr<std::chrono::milliseconds>(
      info,
      logger,
      kActivationTimeoutParam,
      parameters.activation_timeout,
      [](const std::string& text) {
         return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(std::stod(text)));
      });

   const auto use_dummy = info.hardware_parameters.find(kUseDummyParam);
   parameters.use_dummy = use_dummy != info.hardware_parameters.end() && !isFalsey(use_dummy->second);

   return parameters;
}
} // namespace robotiq_driver
