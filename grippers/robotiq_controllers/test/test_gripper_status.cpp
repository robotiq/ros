// Copyright (c) 2026 Robotiq
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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <robotiq_controllers/gripper_status.hpp>

namespace robotiq_controllers::gripper_status::test {
namespace {
using Status = robotiq_msgs::msg::GripperStatus;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const rclcpp::Time kTime{7, 0};

Readings all(double object_status, double motor_current, double gripper_fault, double gripper_fault_severity)
{
   return {object_status, motor_current, gripper_fault, gripper_fault_severity};
}

TEST(BindInterfaces, binds_nothing_without_object_status)
{
   EXPECT_FALSE(bindInterfaces({{"joint", "position"}, {"joint", "velocity"}}).has_value());
   EXPECT_FALSE(bindInterfaces({}).has_value());
}

TEST(BindInterfaces, binds_each_field_by_name)
{
   const std::optional<Indices> indices = bindInterfaces({{"joint", "position"},
                                                          {"joint", "gripper_fault_severity"},
                                                          {"joint", "object_status"},
                                                          {"joint", "motor_current"},
                                                          {"joint", "gripper_fault"}});
   ASSERT_TRUE(indices.has_value());
   EXPECT_EQ(1u, indices->at(GRIPPER_FAULT_SEVERITY));
   EXPECT_EQ(2u, indices->at(OBJECT_STATUS));
   EXPECT_EQ(3u, indices->at(MOTOR_CURRENT));
   EXPECT_EQ(4u, indices->at(GRIPPER_FAULT));
}

TEST(BindInterfaces, leaves_unexported_fields_unbound)
{
   const std::optional<Indices> indices = bindInterfaces({{"joint", "object_status"}, {"joint", "motor_current"}});
   ASSERT_TRUE(indices.has_value());
   EXPECT_TRUE(indices->at(OBJECT_STATUS).has_value());
   EXPECT_FALSE(indices->at(GRIPPER_FAULT).has_value());
   EXPECT_FALSE(indices->at(GRIPPER_FAULT_SEVERITY).has_value());
}

TEST(BindInterfaces, takes_every_field_from_the_joint_that_exports_object_status)
{
   const std::optional<Indices> indices = bindInterfaces({{"arm_joint", "motor_current"},
                                                          {"arm_joint", "gripper_fault"},
                                                          {"gripper_joint", "object_status"},
                                                          {"gripper_joint", "motor_current"}});
   ASSERT_TRUE(indices.has_value());
   EXPECT_EQ(2u, indices->at(OBJECT_STATUS));
   EXPECT_EQ(3u, indices->at(MOTOR_CURRENT));
   EXPECT_FALSE(indices->at(GRIPPER_FAULT).has_value());
}

TEST(Decode, reports_every_field)
{
   const std::optional<Status> status = decode(all(2.0, 0.42, 14.0, 3.0), kTime);
   ASSERT_TRUE(status.has_value());
   EXPECT_EQ(Status::DETECTED_WHILE_CLOSING, status->object_detection);
   EXPECT_DOUBLE_EQ(0.42, status->motor_current);
   EXPECT_EQ(Status::GRIPPER_FAULT_OVERCURRENT, status->gripper_fault);
   EXPECT_EQ(Status::GRIPPER_FAULT_SEVERITY_MAJOR, status->gripper_fault_severity);
   EXPECT_EQ(7, status->header.stamp.sec);
   EXPECT_EQ(0u, status->header.stamp.nanosec);
}

TEST(Decode, says_nothing_until_object_status_has_a_value)
{
   EXPECT_FALSE(decode(all(kNaN, kNaN, kNaN, kNaN), kTime).has_value());
   EXPECT_FALSE(decode({std::nullopt, 0.1, 0.0, 0.0}, kTime).has_value());
   EXPECT_TRUE(decode(all(0.0, 0.1, 0.0, 0.0), kTime).has_value());
}

TEST(Decode, reports_no_fault_for_the_fields_a_joint_does_not_export)
{
   const std::optional<Status> status = decode({1.0, std::nullopt, std::nullopt, std::nullopt}, kTime);
   ASSERT_TRUE(status.has_value());
   EXPECT_EQ(Status::DETECTED_WHILE_OPENING, status->object_detection);
   EXPECT_TRUE(std::isnan(status->motor_current));
   EXPECT_EQ(Status::GRIPPER_FAULT_NONE, status->gripper_fault);
   EXPECT_EQ(Status::GRIPPER_FAULT_SEVERITY_NONE, status->gripper_fault_severity);
}

TEST(Decode, rounds_a_code_that_arrived_as_a_double)
{
   const std::optional<Status> status = decode(all(2.9999999, 0.1, 5.0000001, 1.0), kTime);
   ASSERT_TRUE(status.has_value());
   EXPECT_EQ(Status::AT_REQUESTED_POSITION, status->object_detection);
   EXPECT_EQ(Status::GRIPPER_FAULT_ACTION_DELAYED, status->gripper_fault);
}

TEST(Decode, says_nothing_when_an_exported_field_is_not_a_code)
{
   for(const double bad : {-1.0, 256.0, std::numeric_limits<double>::infinity()})
   {
      EXPECT_FALSE(decode(all(bad, 0.1, 0.0, 0.0), kTime).has_value()) << bad;
   }
   for(const double bad : {-1.0, 256.0, kNaN})
   {
      EXPECT_FALSE(decode(all(0.0, 0.1, bad, 0.0), kTime).has_value()) << bad;
      EXPECT_FALSE(decode(all(0.0, 0.1, 0.0, bad), kTime).has_value()) << bad;
   }
}
} // namespace
} // namespace robotiq_controllers::gripper_status::test
