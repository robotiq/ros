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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <map>
#include <stdexcept>
#include <string>

#include <robotiq_driver/hardware_parameters.hpp>

#include <rclcpp/logger.hpp>

namespace robotiq_driver::test {
namespace {
const rclcpp::Logger& logger()
{
   static const rclcpp::Logger instance = rclcpp::get_logger("test_hardware_parameters");
   return instance;
}

//! A HardwareInfo carrying only the mandatory closed position, plus whatever
//! the test adds on top.
hardware_interface::HardwareInfo info_with(std::map<std::string, std::string> parameters = {})
{
   hardware_interface::HardwareInfo info;
   info.hardware_parameters.emplace("gripper_closed_position", "0.7929");
   for(auto& [name, value] : parameters)
   {
      info.hardware_parameters.insert_or_assign(name, value);
   }
   return info;
}
} // namespace

TEST(HardwareParameters, DefaultsMatchTheSdkConnectionDefaults)
{
   const GripperParameters parameters = parseParameters(info_with(), logger());

   // Spelled out rather than compared against the kDefault constants: this
   // test pins the values themselves, and against the constants it would pass
   // whatever they drifted to.
   EXPECT_EQ("/dev/ttyUSB0", parameters.connection.serial.port);
   EXPECT_EQ(115200u, parameters.connection.serial.baudrate);
   EXPECT_EQ(std::chrono::milliseconds{500}, parameters.connection.serial.timeout);
   EXPECT_EQ(0x09, parameters.connection.modbusSlaveAddress);
   EXPECT_DOUBLE_EQ(100.0, parameters.connection.connectionFrequency);
   EXPECT_EQ(std::chrono::milliseconds{15000}, parameters.activation_timeout);
}

TEST(HardwareParameters, ReadsTheConnectionParameters)
{
   const GripperParameters parameters =
      parseParameters(info_with({{"COM_port", "/dev/ttyUSB7"}, {"baudrate", "57600"}, {"connection_frequency", "50"}}),
                      logger());

   EXPECT_EQ("/dev/ttyUSB7", parameters.connection.serial.port);
   EXPECT_EQ(57600u, parameters.connection.serial.baudrate);
   EXPECT_DOUBLE_EQ(50.0, parameters.connection.connectionFrequency);
}

TEST(HardwareParameters, ABaudrateTheSerialPortCannotUseKeepsTheDefault)
{
   // Now that a launch argument feeds this, the ways someone writes the rate
   // wrong matter: std::stoul alone reads "115200bps" as 115200 and "1.152e5"
   // as 1, and libserialport takes 0 or a wrapped negative without complaint,
   // leaving a port that opens and then answers nothing.
   for(const char* unusable : {"115200bps", "1.152e5", "115 200", "0", "-1", "2000000", ""})
   {
      EXPECT_EQ(kBaudrateDefault,
                parseParameters(info_with({{"baudrate", unusable}}), logger()).connection.serial.baudrate)
         << "baudrate '" << unusable << "'";
   }
}

TEST(HardwareParameters, ASlaveAddressWithTrailingCharactersKeepsTheDefault)
{
   EXPECT_EQ(kSlaveAddressDefault,
             parseParameters(info_with({{"slave_address", "0x9 (default)"}}), logger()).connection.modbusSlaveAddress);
}

TEST(HardwareParameters, AClosedPositionItCannotDivideByIsFatal)
{
   // on_init turns this into CallbackReturn::ERROR. PickNik accepted it and
   // then produced an undefined register from the division.
   EXPECT_THROW(parseParameters(info_with({{"gripper_closed_position", "0"}}), logger()), std::invalid_argument);
   EXPECT_THROW(parseParameters(info_with({{"gripper_closed_position", "nan"}}), logger()), std::invalid_argument);
   EXPECT_THROW(parseParameters(info_with({{"gripper_closed_position", "inf"}}), logger()), std::invalid_argument);
}

TEST(HardwareParameters, ANegativeClosedPositionIsSupported)
{
   // A joint whose closed direction is negative: the mapping inverts and works,
   // as it did under PickNik.
   EXPECT_DOUBLE_EQ(-0.7929,
                    parseParameters(info_with({{"gripper_closed_position", "-0.7929"}}), logger()).closed_position);
}

TEST(HardwareParameters, SlaveAddressTakesTheManualsHexOrPlainDecimal)
{
   // The manual prints the address in hex and a description may pass it through
   // verbatim; a bare number is the decimal it looks like. PickNik read
   // everything as hex, so "10" meant 16.
   EXPECT_EQ(0x12, parseParameters(info_with({{"slave_address", "0x12"}}), logger()).connection.modbusSlaveAddress);
   EXPECT_EQ(9, parseParameters(info_with({{"slave_address", "9"}}), logger()).connection.modbusSlaveAddress);
   EXPECT_EQ(10, parseParameters(info_with({{"slave_address", "10"}}), logger()).connection.modbusSlaveAddress);
}

TEST(HardwareParameters, ASlaveAddressBeyondAByteKeepsTheDefault)
{
   EXPECT_EQ(kSlaveAddressDefault,
             parseParameters(info_with({{"slave_address", "0x1FF"}}), logger()).connection.modbusSlaveAddress);
}

TEST(HardwareParameters, TimeoutsAreSecondsInTheUrdfAndMillisecondsInTheConfig)
{
   const GripperParameters parameters =
      parseParameters(info_with({{"timeout", "0.25"}, {"activation_timeout", "30"}}), logger());

   EXPECT_EQ(std::chrono::milliseconds{250}, parameters.connection.serial.timeout);
   EXPECT_EQ(std::chrono::milliseconds{30000}, parameters.activation_timeout);
}

TEST(HardwareParameters, ReadsTheScalingParameters)
{
   const GripperParameters parameters = parseParameters(info_with({{"gripper_max_speed", "0.2"},
                                                                   {"gripper_min_speed", "0.03"},
                                                                   {"gripper_max_force", "185"},
                                                                   {"gripper_speed_multiplier", "0.75"},
                                                                   {"gripper_force_multiplier", "0.5"}}),
                                                        logger());

   EXPECT_DOUBLE_EQ(0.7929, parameters.closed_position);
   EXPECT_DOUBLE_EQ(0.2, parameters.max_speed);
   EXPECT_DOUBLE_EQ(0.03, parameters.min_speed);
   EXPECT_DOUBLE_EQ(185.0, parameters.max_force);
   EXPECT_DOUBLE_EQ(0.75, parameters.speed_multiplier);
   EXPECT_DOUBLE_EQ(0.5, parameters.force_multiplier);
}

TEST(HardwareParameters, AMalformedParameterFallsBackToItsDefault)
{
   // One unparsable string in the description must not take the gripper
   // down at startup — it is reported and the default stands.
   const GripperParameters parameters =
      parseParameters(info_with({{"baudrate", "fast"}, {"gripper_max_force", ""}}), logger());

   EXPECT_EQ(kBaudrateDefault, parameters.connection.serial.baudrate);
   EXPECT_DOUBLE_EQ(kMaxForceDefault, parameters.max_force);
}

TEST(HardwareParameters, AnUnusableScaleFallsBackToItsDefault)
{
   // Both are divisors of the register arithmetic, so a bad value is caught
   // here rather than at each use.
   for(const char* unusable : {"0", "-1", "-235", "nan", "-inf"})
   {
      const GripperParameters parameters = parseParameters(
         info_with({{"gripper_max_force", unusable}, {"gripper_max_speed", unusable}, {"gripper_min_speed", unusable}}),
         logger());

      EXPECT_DOUBLE_EQ(kMaxForceDefault, parameters.max_force) << "gripper_max_force '" << unusable << "'";
      EXPECT_DOUBLE_EQ(kMaxSpeedDefault, parameters.max_speed) << "gripper_max_speed '" << unusable << "'";
      EXPECT_DOUBLE_EQ(kMinSpeedDefault, parameters.min_speed) << "gripper_min_speed '" << unusable << "'";
   }
}

TEST(HardwareParameters, AMissingClosedPositionIsFatal)
{
   // Unlike every other parameter this one has no defensible default: a
   // guessed value mis-scales every position the gripper is ever given.
   hardware_interface::HardwareInfo info;
   EXPECT_THROW(GripperParameters parameters = parseParameters(info, logger()), std::out_of_range);
}

TEST(HardwareParameters, UseDummyDefaultsOff)
{
   EXPECT_FALSE(parseParameters(info_with(), logger()).use_dummy);
   EXPECT_FALSE(parseParameters(info_with({{"use_dummy", "false"}}), logger()).use_dummy);
}

TEST(HardwareParameters, WhenUseDummyIsFalseLike_thenTheRealGripperIsUsed)
{
   for(const char* spelling : {"false", "False", "FALSE", "0", "no", "off", ""})
   {
      EXPECT_FALSE(parseParameters(info_with({{"use_dummy", spelling}}), logger()).use_dummy)
         << "'" << spelling << "' should not select the fake gripper";
   }
}

TEST(HardwareParameters, WhenUseDummyIsNotFalseLike_thenTheDummyIsUsed)
{
   EXPECT_TRUE(parseParameters(info_with({{"use_dummy", "true"}}), logger()).use_dummy);
   EXPECT_TRUE(parseParameters(info_with({{"use_dummy", "True"}}), logger()).use_dummy);
   EXPECT_TRUE(parseParameters(info_with({{"use_dummy", "1"}}), logger()).use_dummy);
   EXPECT_TRUE(parseParameters(info_with({{"use_dummy", "yes"}}), logger()).use_dummy);
}
} // namespace robotiq_driver::test
