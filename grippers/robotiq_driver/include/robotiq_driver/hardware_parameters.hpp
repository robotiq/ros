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

//! \brief The URDF's <hardware> params, parsed into the SDK's
//!        Robotiq::ConnectionConfig plus the joint-scaling constants the
//!        wrapper needs on top of it.
//! Parsing is a free function over HardwareInfo so it can be tested against a
//! hand-built parameter map, with no controller manager in the way.

#pragma once

#include <chrono>
#include <cstdint>

#include <Robotiq/gripper/connection_config.hpp>

#include <hardware_interface/hardware_info.hpp>

#include <rclcpp/logger.hpp>

namespace robotiq_driver {

//! Defaults for the connection parameters. They match Robotiq::ConnectionConfig
//! today, but are pinned here on purpose: a description that omits one of
//! these parameters has to keep getting this value even if the SDK changes
//! its own.
inline constexpr const char* kComPortDefault = "/dev/ttyUSB0";
inline constexpr uint32_t kBaudrateDefault = 115200;
inline constexpr auto kTimeoutDefault = std::chrono::milliseconds{500};
inline constexpr uint8_t kSlaveAddressDefault = 0x09;
inline constexpr double kConnectionFrequencyDefault = 100.0;

//! Full scale of the shipped 2F-85, and so the default for
//! gripper_max_speed / gripper_max_force.
inline constexpr double kMaxSpeedDefault = 0.150; // m/s
//! Slowest speed of the shipped 2F-85, what rSP 0 moves at, and so the
//! default for gripper_min_speed. Robotiq::profiles::k2F85.minSpeed.
inline constexpr double kMinSpeedDefault = 0.020; // m/s
inline constexpr double kMaxForceDefault = 235.0; // N

struct GripperParameters
{
   //! Serial link + Modbus addressing, handed straight to Robotiq::Gripper.
   Robotiq::ConnectionConfig connection;

   //! Joint angle (rad) at a fully closed gripper — the scale of the
   //! register-count-to-joint mapping. Model-specific, so it has no default
   //! here: the URDF must supply it.
   double closed_position = 0.0;

   //! Full-scale speed and force, used to turn the SI values written to the
   //! set_gripper_max_velocity / set_gripper_max_effort command interfaces
   //! into rSP / rFR register fractions.
   double max_speed = kMaxSpeedDefault;
   //! Speed at rSP 0. A speed command maps linearly from here (register 0)
   //! to max_speed (register 255), the SDK's speedToRegister rule.
   double min_speed = kMinSpeedDefault;
   double max_force = kMaxForceDefault;

   //! Initial fractions of the above published on those command interfaces.
   double speed_multiplier = 1.0;
   double force_multiplier = 1.0;

   //! Budget for the blocking activation and fault-recovery procedures; the
   //! calibration sweep dominates it.
   std::chrono::milliseconds activation_timeout{15000};

   //! Drive a simulated gripper instead of a real one, for testing without
   //! hardware. The serial settings above are then ignored.
   bool use_dummy = false;
};

//! Parse \p info's hardware parameters. Missing parameters keep the defaults
//! above; malformed ones are reported through \p logger and also keep the
//! default, so one bad string cannot take the gripper down at startup.
//! \throw std::out_of_range when gripper_closed_position is absent,
//!        std::invalid_argument when it does not parse — the one parameter
//!        with no sane default.
[[nodiscard]] GripperParameters parseParameters(const hardware_interface::HardwareInfo& info,
                                                const rclcpp::Logger& logger);
} // namespace robotiq_driver
