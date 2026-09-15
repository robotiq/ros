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

#include <limits>
#include <optional>
#include <type_traits>

#include <robotiq_driver/gripper_scaling.hpp>

namespace robotiq_driver::test {
namespace {
// The 2F-85's fully-closed joint angle, as the shipped description sets it.
constexpr double kClosedPosition = 0.7929;
} // namespace

TEST(GripperScaling, TravelEndpointsMapToTheJointLimits)
{
   EXPECT_DOUBLE_EQ(0.0, jointPositionFromRegister(kGripperMinPos, kClosedPosition));
   EXPECT_DOUBLE_EQ(kClosedPosition, jointPositionFromRegister(kGripperMaxPos, kClosedPosition));
}

TEST(GripperScaling, JointLimitsMapBackToTheTravelEndpoints)
{
   EXPECT_EQ(kGripperMinPos, registerFromJointPosition(0.0, kClosedPosition));
   EXPECT_EQ(kGripperMaxPos, registerFromJointPosition(kClosedPosition, kClosedPosition));
}

TEST(GripperScaling, RegisterRoundTripLosesAtMostOneCount)
{
   // counts -> radians -> counts does not always land back on the same count:
   // the reverse mapping truncates, so a position read off the gripper and
   // commanded straight back can come out one low. That is the behaviour the
   // driver has always had, pinned here rather than fixed, so that changing it
   // is a visible decision rather than a side effect of some later edit.
   for(uint8_t counts = kGripperMinPos; counts <= kGripperMaxPos; ++counts)
   {
      const std::optional<uint8_t> round_tripped =
         registerFromJointPosition(jointPositionFromRegister(counts, kClosedPosition), kClosedPosition);
      ASSERT_TRUE(round_tripped.has_value());
      EXPECT_THAT(*round_tripped, testing::AnyOf(counts, counts - 1))
         << "round trip drifted more than a count at " << static_cast<int>(counts) << " counts";
   }
}

TEST(GripperScaling, PositionsOutsideTheJointRangeClampIntoTheByte)
{
   EXPECT_EQ(0, registerFromJointPosition(-1.0, kClosedPosition));
   EXPECT_EQ(255, registerFromJointPosition(10.0 * kClosedPosition, kClosedPosition));
}

TEST(GripperScaling, PositionsBelowTheTravelBandStayReachable)
{
   // The band starts at 3 counts, but 0..2 are still commandable: clamping is
   // to the byte, not to the band.
   EXPECT_LT(registerFromJointPosition(-0.005, kClosedPosition), kGripperMinPos);
}

TEST(GripperScaling, AnUnusableClosedPositionCommandsNothing)
{
   // 0/0 and x/0 have no register to land on. Answering "nothing" keeps the
   // caller from moving the fingers on a made-up count.
   EXPECT_FALSE(registerFromJointPosition(0.0, 0.0).has_value());
   EXPECT_FALSE(registerFromJointPosition(0.5, 0.0).has_value());
   EXPECT_FALSE(registerFromJointPosition(std::numeric_limits<double>::quiet_NaN(), kClosedPosition).has_value());
   EXPECT_FALSE(registerFromJointPosition(0.5, std::numeric_limits<double>::quiet_NaN()).has_value());
}

TEST(GripperScaling, AClosedPositionThatClosesNegativeStillMaps)
{
   // A joint whose closed direction is negative is a supported description:
   // the ratio inverts and the mapping still lands in the travel band.
   EXPECT_EQ(kGripperMinPos, registerFromJointPosition(0.0, -kClosedPosition));
   EXPECT_EQ(kGripperMaxPos, registerFromJointPosition(-kClosedPosition, -kClosedPosition));
}

TEST(GripperScaling, AFractionOutsideTheRangeCommandsNothing)
{
   // As above: no fraction, so no register, rather than a guessed speed or
   // force. A negative speed and a maximum that is not positive are both broken
   // descriptions, not directions the way a negative closed position is.
   EXPECT_FALSE(registerFromFractionOf(0.1, 0.0).has_value());
   EXPECT_FALSE(registerFromFractionOf(0.1, -0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(-0.1, 0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(std::numeric_limits<double>::quiet_NaN(), 0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(0.1, std::numeric_limits<double>::quiet_NaN()).has_value());
   EXPECT_FALSE(registerFromFractionOf(std::numeric_limits<double>::infinity(), 0.15).has_value());
   EXPECT_FALSE(registerFromFractionOf(0.1, std::numeric_limits<double>::infinity()).has_value());
}

TEST(GripperScaling, FractionOfSpansTheWholeByte)
{
   EXPECT_EQ(0, registerFromFractionOf(0.0, 0.15));
   EXPECT_EQ(255, registerFromFractionOf(0.15, 0.15));
   // 0.5 * 0xFF is 127.5, truncated to 127 — see the round-trip test above.
   EXPECT_EQ(127, registerFromFractionOf(0.075, 0.15));
}

TEST(GripperScaling, FractionOfSaturatesRatherThanWrapping)
{
   // A request beyond full scale must pin at 0xFF; truncating the product
   // would wrap it to a near-zero speed or force.
   EXPECT_EQ(255, registerFromFractionOf(10.0, 0.15));
}

TEST(GripperScaling, whenTheMaximumIsNearZero_theRegisterIsTheMaxAllowedValue)
{
   EXPECT_EQ(255, registerFromFractionOf(0.15, std::numeric_limits<double>::denorm_min()));
}

TEST(GripperScaling, MotorCurrentIsTenMilliampsPerCount)
{
   EXPECT_DOUBLE_EQ(0.0, motorCurrentFromRegister(0));
   EXPECT_DOUBLE_EQ(0.01, motorCurrentFromRegister(1));
   EXPECT_DOUBLE_EQ(1.27, motorCurrentFromRegister(127));
   EXPECT_DOUBLE_EQ(2.55, motorCurrentFromRegister(255));
}

TEST(GripperScaling, MotorCurrentTakesNoScaleFromTheDescription)
{
   // Scaling gCU by a description parameter would make one motor current read
   // as two numbers on two robots.
   static_assert(std::is_invocable_r_v<double, decltype(motorCurrentFromRegister), uint8_t>);
   EXPECT_DOUBLE_EQ(1.0, motorCurrentFromRegister(100));
}

TEST(GripperScaling, TheFaultInterfacesReadTheGripperFaultAndItsOwnSeverity)
{
   const Robotiq::FaultStatus fault = Robotiq::FaultStatus::fromRaw(0x5C);
   //            \__ gripper fault: 0xC Internal; controller fault: 0x5 NoDeviceDetected

   const auto expectedGripperFault = Robotiq::GripperFault::InternalFault;
   EXPECT_EQ(static_cast<double>(expectedGripperFault), gripperFaultFromRegister(fault));
   EXPECT_EQ(static_cast<double>(Robotiq::severity(expectedGripperFault)), gripperFaultSeverityFromRegister(fault));
}

} // namespace robotiq_driver::test
