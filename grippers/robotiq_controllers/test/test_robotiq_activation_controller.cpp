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

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <robotiq_controllers/robotiq_activation_controller.hpp>

#include "controller_test_compat.hpp"

namespace robotiq_controllers::test {
namespace {
constexpr const char* kControllerName = "test_robotiq_activation_controller";
constexpr unsigned int kUpdateRate = 100;

//! The command interfaces the controller claims, once initialized with \p parameters.
std::vector<std::string> claimedInterfaces(const std::vector<rclcpp::Parameter>& parameters = {})
{
   RobotiqActivationController controller;
   EXPECT_EQ(controller_interface::return_type::OK,
             test_compat::init(controller, kControllerName, kUpdateRate, parameters));
   return controller.command_interface_configuration().names;
}

TEST(RobotiqActivationControllerTest, ClaimsTheUnprefixedGpioByDefault)
{
   const std::vector<std::string> expected{"reactivate_gripper/reactivate_gripper_cmd",
                                           "reactivate_gripper/reactivate_gripper_response"};
   EXPECT_EQ(expected, claimedInterfaces());
}

TEST(RobotiqActivationControllerTest, ClaimsThePrefixedGpioItIsGiven)
{
   const std::vector<std::string> expected{"left_reactivate_gripper/reactivate_gripper_cmd",
                                           "left_reactivate_gripper/reactivate_gripper_response"};
   EXPECT_EQ(expected, claimedInterfaces({rclcpp::Parameter("reactivate_gpio", "left_reactivate_gripper")}));
}

TEST(RobotiqActivationControllerTest, RejectsAnEmptyGpioName)
{
   RobotiqActivationController controller;
   EXPECT_EQ(controller_interface::return_type::ERROR,
             test_compat::init(controller, kControllerName, kUpdateRate, {rclcpp::Parameter("reactivate_gpio", "")}));
}
} // namespace
} // namespace robotiq_controllers::test

// Not gtest_main: the tests create nodes, so rclcpp must be up before the first
// and down after the last.
int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   rclcpp::init(argc, argv);
   const int result = RUN_ALL_TESTS();
   rclcpp::shutdown();
   return result;
}
