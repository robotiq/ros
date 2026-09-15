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

//! \brief Conversions between ros2_control's SI joint quantities and the
//!        gripper's 0..255 register counts.
//! Pulled out of the hardware interface so the arithmetic — the part that
//! silently mis-commands a gripper when it is wrong — is unit-testable
//! without a gripper, a serial port, or a controller manager.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <Robotiq/gripper/fault_status.hpp>

namespace robotiq_driver {

// The usable travel band of the 2F fingers in register counts. The gripper
// reports and accepts 0..255, but the extremes are outside the mechanical
// range, so the joint mapping is anchored on these two.
inline constexpr uint8_t kGripperMinPos = 3;
inline constexpr uint8_t kGripperMaxPos = 230;
inline constexpr uint8_t kGripperRange = kGripperMaxPos - kGripperMinPos;

//! Register count (gPO: 0 open .. 255 closed) -> joint position, linear over
//! the usable travel band. \p closed_position is the joint value at a fully
//! closed gripper and sets the unit: the mapping carries whatever the URDF
//! uses. The shipped 2F descriptions model the knuckle as a revolute joint,
//! so there it is radians.
[[nodiscard]] inline double jointPositionFromRegister(uint8_t position, double closed_position)
{
   return closed_position * (position - kGripperMinPos) / kGripperRange;
}

//! Joint position -> register count (rPR), in the same unit as
//! \p closed_position. Counts outside the byte are clamped. Nothing when the
//! arithmetic cannot produce a count — a NaN command, or a
//! \p closed_position of zero: there is no defensible register value, and
//! commanding a made-up one moves a gripper that may be holding something.
//!
//! Truncates rather than rounds, so a position read off the gripper and
//! commanded straight back can land one count below where it came from.
[[nodiscard]] inline std::optional<uint8_t> registerFromJointPosition(double joint_position, double closed_position)
{
   const double counts = (joint_position / closed_position) * kGripperRange + kGripperMinPos;
   if(!std::isfinite(counts))
   {
      return std::nullopt;
   }
   return static_cast<uint8_t>(std::clamp(counts, 0.0, 255.0));
}

//! An SI speed or force command -> its 0x00..0xFF register (rSP / rFR), as the
//! fraction of \p maximum it represents. Truncates, as above. Nothing unless
//! both sides are in range: \p value from zero up, \p maximum above it. The
//! range excludes NaN and infinity, which no comparison admits.
[[nodiscard]] inline std::optional<uint8_t> registerFromFractionOf(double value, double maximum)
{
   constexpr double kLargest = std::numeric_limits<double>::max();
   const bool in_range = value >= 0.0 && value <= kLargest && maximum > 0.0 && maximum <= kLargest;
   if(!in_range)
   {
      return std::nullopt;
   }
   return static_cast<uint8_t>(std::min(value / maximum, 1.0) * 0xFF);
}

inline constexpr double kAmperesPerCurrentCount = 0.010;

[[nodiscard]] inline constexpr double motorCurrentFromRegister(uint8_t register_value)
{
   return kAmperesPerCurrentCount * register_value;
}

// gFLT alone: the fault byte's high nibble is kFLT, a fault of the controller
// rather than of the gripper, and nothing exports it.
[[nodiscard]] inline double gripperFaultFromRegister(Robotiq::FaultStatus fault)
{
   return static_cast<double>(fault.gripperFault());
}

// The SDK ranks a code it does not recognize as Major, so an undocumented
// fault is never reported as harmless.
[[nodiscard]] inline double gripperFaultSeverityFromRegister(Robotiq::FaultStatus fault)
{
   return static_cast<double>(Robotiq::severity(fault.gripperFault()));
}
} // namespace robotiq_driver
