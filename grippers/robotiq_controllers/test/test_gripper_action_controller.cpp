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

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robotiq_controllers/gripper_action_controller.hpp>
#include <robotiq_msgs/msg/gripper_status.hpp>

#include "controller_test_compat.hpp"

namespace robotiq_controllers::test {
namespace {
constexpr const char* kControllerName = "test_gripper_action_controller";
constexpr const char* kJoint = "robotiq_85_left_knuckle_joint";
constexpr const char* kOtherJoint = "shoulder_pan_joint";
constexpr unsigned int kUpdateRate = 100;
constexpr double kGoalTolerance = 0.02;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

using Status = robotiq_msgs::msg::GripperStatus;
using Action = control_msgs::action::ParallelGripperCommand;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;
using Result = ClientGoalHandle::WrappedResult;

class GripperActionControllerTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      client_node_ = std::make_shared<rclcpp::Node>("gripper_action_client");
      executor_.add_node(client_node_);
      client_ = rclcpp_action::create_client<Action>(client_node_, std::string{"/"} + kControllerName + "/gripper_cmd");
   }

   // A zero stall timeout makes the stock check call a stall on the first
   // cycle, so a goal that survives it was decided by nothing else.
   void init(bool use_object_status, bool allow_stalling = true, double stall_timeout = 0.0)
   {
      controller_ = std::make_unique<GripperActionController>();
      ASSERT_EQ(controller_interface::return_type::OK,
                test_compat::init(*controller_,
                                  kControllerName,
                                  kUpdateRate,
                                  {rclcpp::Parameter("joint", kJoint),
                                   rclcpp::Parameter("use_object_status", use_object_status),
                                   rclcpp::Parameter("allow_stalling", allow_stalling),
                                   rclcpp::Parameter("goal_tolerance", kGoalTolerance),
                                   rclcpp::Parameter("stall_timeout", stall_timeout)}));
      executor_.add_node(controller_->get_node()->get_node_base_interface());
   }

   // Loans the gripper joint's position and velocity plus its object_status when
   // asked, behind another joint's when asked, as an ALL claim would deliver them.
   void assign(bool with_object_status, bool other_joint_first = false)
   {
      std::vector<hardware_interface::LoanedStateInterface> states;
      std::vector<hardware_interface::LoanedCommandInterface> commands;
      owned_states_.clear();
      owned_commands_.clear();

      auto add = [&](const std::string& joint, const std::string& name, double* value) {
         owned_states_.push_back(test_compat::makeStateInterface(joint, name, value));
         states.push_back(test_compat::loan(owned_states_.back()));
      };

      if(other_joint_first)
      {
         add(kOtherJoint, "position", &other_position_);
         add(kOtherJoint, "velocity", &other_velocity_);
      }
      add(kJoint, "position", &position_);
      add(kJoint, "velocity", &velocity_);
      if(with_object_status)
      {
         add(kJoint, "object_status", &object_status_);
      }
      owned_commands_.push_back(test_compat::makeCommandInterface(kJoint, "position", &position_command_));
      commands.push_back(test_compat::loanCommand(owned_commands_.back()));

      controller_->assign_interfaces(std::move(commands), std::move(states));
   }

   void configure() { ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_->configure().id()); }

   void activate()
   {
      ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_->get_node()->activate().id());
   }

   void deactivate()
   {
      ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_->get_node()->deactivate().id());
      controller_->release_interfaces();
   }

   void bringUp(bool use_object_status, bool with_object_status, bool allow_stalling = true, double stall_timeout = 0.0)
   {
      init(use_object_status, allow_stalling, stall_timeout);
      assign(with_object_status);
      configure();
      activate();
   }

   // Bounds a failing wait; a passing one ends as soon as `done` holds.
   void spinUntil(const std::function<bool()>& done)
   {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
      while(!done() && std::chrono::steady_clock::now() < deadline)
      {
         executor_.spin_once(std::chrono::milliseconds{10});
      }
   }

   void update()
   {
      ASSERT_EQ(controller_interface::return_type::OK,
                controller_->update(client_node_->now(), rclcpp::Duration::from_seconds(1.0 / kUpdateRate)));
   }

   void sendGoal(double position)
   {
      ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds{5}));
      Action::Goal goal;
      goal.command.name = {kJoint};
      goal.command.position = {position};
      auto goal_future = client_->async_send_goal(goal);
      spinUntil([&] { return goal_future.wait_for(std::chrono::seconds{0}) == std::future_status::ready; });
      ASSERT_EQ(std::future_status::ready, goal_future.wait_for(std::chrono::seconds{0}));
      goal_handle_ = goal_future.get();
      ASSERT_TRUE(goal_handle_) << "the goal was rejected";
      result_future_ = client_->async_get_result(goal_handle_);
      // The acceptance cycle, whose reading predates the command.
      update();
   }

   bool resultArrived() { return result_future_.wait_for(std::chrono::seconds{0}) == std::future_status::ready; }

   // Cycles the controller until the client holds the result.
   std::optional<Result> awaitResult()
   {
      spinUntil([&] {
         update();
         return resultArrived();
      });
      if(!resultArrived())
      {
         return std::nullopt;
      }
      return result_future_.get();
   }

   // A goal the controller must leave open: a few cycles, spaced so that the
   // action monitor timer would have relayed a verdict.
   void expectStillActive()
   {
      for(int cycle = 0; cycle < 10; ++cycle)
      {
         update();
         executor_.spin_some();
         std::this_thread::sleep_for(std::chrono::milliseconds{10});
      }
      EXPECT_FALSE(resultArrived()) << "the goal was decided";
   }

   std::unique_ptr<GripperActionController> controller_;
   std::vector<std::shared_ptr<hardware_interface::StateInterface>> owned_states_;
   std::vector<std::shared_ptr<hardware_interface::CommandInterface>> owned_commands_;

   rclcpp::Node::SharedPtr client_node_;
   rclcpp_action::Client<Action>::SharedPtr client_;
   rclcpp::executors::SingleThreadedExecutor executor_;
   ClientGoalHandle::SharedPtr goal_handle_;
   std::shared_future<Result> result_future_;

   double position_ = 0.1;
   double velocity_ = 0.0;
   double object_status_ = kNaN;
   double other_position_ = 1.5;
   double other_velocity_ = 0.0;
   double position_command_ = kNaN;
};

TEST_F(GripperActionControllerTest, claims_every_state_interface_only_with_object_status_on)
{
   init(true);
   configure();
   EXPECT_EQ(controller_interface::interface_configuration_type::ALL,
             controller_->state_interface_configuration().type);

   executor_.remove_node(controller_->get_node()->get_node_base_interface());
   init(false);
   configure();
   const controller_interface::InterfaceConfiguration stock = controller_->state_interface_configuration();
   EXPECT_EQ(controller_interface::interface_configuration_type::INDIVIDUAL, stock.type);
   EXPECT_EQ((std::vector<std::string>{std::string{kJoint} + "/position", std::string{kJoint} + "/velocity"}),
             stock.names);
}

TEST_F(GripperActionControllerTest, ignores_the_object_status_when_off)
{
   bringUp(false, true, true, 3600.0);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;
   expectStillActive();

   position_ = 0.5;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, waits_for_the_status_to_change_from_its_value_at_acceptance)
{
   bringUp(true, true);
   object_status_ = Status::AT_REQUESTED_POSITION;
   sendGoal(0.5);
   expectStillActive();
   object_status_ = Status::MOVING;
   expectStillActive();

   object_status_ = Status::DETECTED_WHILE_CLOSING;
   position_ = 0.3;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->stalled);
   EXPECT_FALSE(result->result->reached_goal);
   EXPECT_DOUBLE_EQ(0.3, result->result->state.position[0]);
}

TEST_F(GripperActionControllerTest, aborts_a_stall_without_allow_stalling)
{
   bringUp(true, true, false);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, reports_reached_at_the_requested_position)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::AT_REQUESTED_POSITION;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, reports_a_stall_while_opening)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.0);
   object_status_ = Status::DETECTED_WHILE_OPENING;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, decides_on_a_direct_change_between_settled_states)
{
   bringUp(true, true);
   object_status_ = Status::AT_REQUESTED_POSITION;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, leaves_a_goal_accepted_without_a_reading_to_the_stock_path)
{
   bringUp(true, true);
   object_status_ = kNaN;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;
   expectStillActive();

   position_ = 0.5;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
}

TEST_F(GripperActionControllerTest, holds_while_the_reading_is_missing)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = kNaN;
   expectStillActive();

   object_status_ = Status::AT_REQUESTED_POSITION;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
}

TEST_F(GripperActionControllerTest, takes_a_new_baseline_for_the_next_goal)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;
   ASSERT_TRUE(awaitResult().has_value());

   // The gripper still reports the grasp when the next goal arrives.
   sendGoal(0.0);
   expectStillActive();
   object_status_ = Status::MOVING;
   expectStillActive();

   object_status_ = Status::AT_REQUESTED_POSITION;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, decides_on_the_starting_value_once_motion_was_seen)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;
   ASSERT_TRUE(awaitResult().has_value());

   // Tightening on the held object: the gripper moves, then detects it again.
   sendGoal(0.6);
   expectStillActive();
   object_status_ = Status::MOVING;
   expectStillActive();

   object_status_ = Status::DETECTED_WHILE_CLOSING;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, takes_a_new_baseline_for_a_preempting_goal)
{
   bringUp(true, true);
   object_status_ = Status::AT_REQUESTED_POSITION;
   sendGoal(0.5);
   object_status_ = Status::MOVING;
   expectStillActive();

   // The stock controller relays no result for the goal it preempts, so only
   // the new one is observable.
   sendGoal(0.6);
   object_status_ = Status::DETECTED_WHILE_CLOSING;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, falls_back_to_the_stock_path_where_the_joint_exports_no_object_status)
{
   bringUp(true, false, true, 3600.0);
   sendGoal(0.5);
   expectStillActive();

   position_ = 0.5;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
}

TEST_F(GripperActionControllerTest, keeps_the_stock_stall_timeout_where_the_joint_exports_no_object_status)
{
   bringUp(true, false);
   sendGoal(0.5);

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, leaves_the_stall_to_the_gripper)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   velocity_ = 0.0;
   expectStillActive();

   object_status_ = Status::DETECTED_WHILE_CLOSING;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, restores_the_stall_timeout_when_reactivated_without_object_status)
{
   bringUp(true, true);
   deactivate();
   assign(false);
   activate();
   sendGoal(0.5);

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, binds_the_gripper_joint_behind_another_joints_interfaces)
{
   init(true);
   assign(true, true);
   configure();
   activate();
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::AT_REQUESTED_POSITION;
   position_ = 0.5;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_DOUBLE_EQ(position_, result->result->state.position[0]);
}

TEST_F(GripperActionControllerTest, decides_again_after_a_deactivation)
{
   bringUp(true, true);
   object_status_ = Status::MOVING;
   sendGoal(0.5);
   object_status_ = Status::DETECTED_WHILE_CLOSING;
   ASSERT_TRUE(awaitResult().has_value());

   deactivate();
   assign(true);
   activate();
   sendGoal(0.0);
   expectStillActive();
   object_status_ = Status::AT_REQUESTED_POSITION;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
}
} // namespace
} // namespace robotiq_controllers::test

int main(int argc, char** argv)
{
   ::testing::InitGoogleTest(&argc, argv);
   rclcpp::init(argc, argv);
   const int result = RUN_ALL_TESTS();
   rclcpp::shutdown();
   return result;
}
