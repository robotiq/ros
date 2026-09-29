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
#include <vector>

#include <control_msgs/action/parallel_gripper_command.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robotiq_controllers/gripper_action_controller.hpp>

#include "controller_test_compat.hpp"

namespace robotiq_controllers::test {
namespace {
constexpr const char* kControllerName = "test_gripper_action_controller";
constexpr const char* kJoint = "robotiq_85_left_knuckle_joint";
constexpr unsigned int kUpdateRate = 100;
constexpr double kGoalTolerance = 0.02;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// object_status_timeout is left at its default; the update times are relative
// to what the timeout is measured from.
constexpr double kObjectStatusTimeout = 10.0;
constexpr double kWithinTimeout = kObjectStatusTimeout / 2;
constexpr double kPastTimeout = kObjectStatusTimeout + 1.0;
// A stock stall timeout no test outlasts.
constexpr double kStockStallNever = 3600.0;

constexpr double kMoving = static_cast<double>(Robotiq::ObjectDetection::Moving);
constexpr double kDetectedWhileClosing = static_cast<double>(Robotiq::ObjectDetection::DetectedWhileClosing);
constexpr double kAtRequestedPosition = static_cast<double>(Robotiq::ObjectDetection::AtRequestedPosition);

using ParallelGripperCommand = control_msgs::action::ParallelGripperCommand;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<ParallelGripperCommand>;
using Result = ClientGoalHandle::WrappedResult;

// The zero stall timeout makes the stock check call a stall on the first
// cycle, so a goal that survives it was decided by nothing else.
struct Config
{
   bool use_object_status = false;
   bool export_joint_object_status = false;
   bool allow_stalling = false;
   double stall_timeout = 0.0;

   // The driver's configuration.
   static Config driver() { return Config{}.useObjectStatus().exportJointObjectStatus().allowStalling(); }

   Config& useObjectStatus()
   {
      use_object_status = true;
      return *this;
   }
   Config& exportJointObjectStatus()
   {
      export_joint_object_status = true;
      return *this;
   }
   Config& allowStalling()
   {
      allow_stalling = true;
      return *this;
   }
   Config& stallTimeout(double seconds)
   {
      stall_timeout = seconds;
      return *this;
   }
};

// The controller clears the goal it holds the moment it decides it, so a test
// can ask synchronously, without waiting for the result to reach a client.
class TestableController : public GripperActionController
{
public:
   bool holdsGoal()
   {
      RealtimeGoalHandlePtr goal;
      rt_active_goal_.get([&](const RealtimeGoalHandlePtr& active) { goal = active; });
      return goal != nullptr;
   }
};

class GripperActionControllerTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      client_node_ = std::make_shared<rclcpp::Node>("gripper_action_client");
      executor_.add_node(client_node_);
      client_ =
         rclcpp_action::create_client<ParallelGripperCommand>(client_node_,
                                                              std::string{"/"} + kControllerName + "/gripper_cmd");
   }

   void init(const Config& config)
   {
      controller_ = std::make_unique<TestableController>();
      ASSERT_EQ(controller_interface::return_type::OK,
                test_compat::init(*controller_,
                                  kControllerName,
                                  kUpdateRate,
                                  {rclcpp::Parameter("joint", kJoint),
                                   rclcpp::Parameter("use_object_status", config.use_object_status),
                                   rclcpp::Parameter("allow_stalling", config.allow_stalling),
                                   rclcpp::Parameter("goal_tolerance", kGoalTolerance),
                                   rclcpp::Parameter("stall_timeout", config.stall_timeout),
                                   // Relays a verdict to the client without delaying awaitResult.
                                   rclcpp::Parameter("action_monitor_rate", 1000.0)}));
      executor_.add_node(controller_->get_node()->get_node_base_interface());
   }

   void assign(const Config& config)
   {
      std::vector<hardware_interface::LoanedStateInterface> states;
      std::vector<hardware_interface::LoanedCommandInterface> commands;
      owned_states_.clear();
      owned_commands_.clear();

      auto add = [&](const std::string& joint, const std::string& name, double* value) {
         owned_states_.push_back(test_compat::makeStateInterface(joint, name, value));
         states.push_back(test_compat::loan(owned_states_.back()));
      };

      add(kJoint, "position", &position_);
      add(kJoint, "velocity", &velocity_);
      if(config.export_joint_object_status)
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

   void bringUp(const Config& config = Config::driver())
   {
      init(config);
      assign(config);
      configure();
      activate();
   }

   void spinUntil(const std::function<bool()>& done)
   {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
      while(!done() && std::chrono::steady_clock::now() < deadline)
      {
         executor_.spin_once(std::chrono::milliseconds{10});
      }
   }

   void update(double seconds_from_now = 0.0)
   {
      ASSERT_EQ(controller_interface::return_type::OK,
                controller_->update(client_node_->now() + rclcpp::Duration::from_seconds(seconds_from_now),
                                    rclcpp::Duration::from_seconds(1.0 / kUpdateRate)));
   }

   void sendGoal(double position)
   {
      ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds{5}));
      ParallelGripperCommand::Goal goal;
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

   // A goal the controller must leave open: one cycle takes the reading in,
   // the next is the first that could act on it.
   void expectStillActive()
   {
      update();
      update();
      EXPECT_TRUE(controller_->holdsGoal()) << "the goal was decided";
   }

   std::unique_ptr<TestableController> controller_;
   std::vector<std::shared_ptr<hardware_interface::StateInterface>> owned_states_;
   std::vector<std::shared_ptr<hardware_interface::CommandInterface>> owned_commands_;

   rclcpp::Node::SharedPtr client_node_;
   rclcpp_action::Client<ParallelGripperCommand>::SharedPtr client_;
   rclcpp::executors::SingleThreadedExecutor executor_;
   ClientGoalHandle::SharedPtr goal_handle_;
   std::shared_future<Result> result_future_;

   double position_ = 0.1;
   double velocity_ = 0.0;
   double object_status_ = kNaN;
   double position_command_ = kNaN;
};

TEST_F(GripperActionControllerTest, claims_the_object_status_only_with_the_flag_on)
{
   init(Config{}.useObjectStatus());
   configure();
   const controller_interface::InterfaceConfiguration with_flag = controller_->state_interface_configuration();
   EXPECT_EQ(controller_interface::interface_configuration_type::INDIVIDUAL, with_flag.type);
   EXPECT_EQ((std::vector<std::string>{std::string{kJoint} + "/position",
                                       std::string{kJoint} + "/velocity",
                                       std::string{kJoint} + "/object_status"}),
             with_flag.names);

   executor_.remove_node(controller_->get_node()->get_node_base_interface());
   init(Config{});
   configure();
   const controller_interface::InterfaceConfiguration stock = controller_->state_interface_configuration();
   EXPECT_EQ(controller_interface::interface_configuration_type::INDIVIDUAL, stock.type);
   EXPECT_EQ((std::vector<std::string>{std::string{kJoint} + "/position", std::string{kJoint} + "/velocity"}),
             stock.names);
}

TEST_F(GripperActionControllerTest, ignores_the_object_status_when_off)
{
   bringUp(Config{}.exportJointObjectStatus().allowStalling().stallTimeout(kStockStallNever));
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kDetectedWhileClosing;
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
   bringUp();
   object_status_ = kAtRequestedPosition;
   sendGoal(0.5);
   expectStillActive();
   object_status_ = kMoving;
   expectStillActive();

   object_status_ = kDetectedWhileClosing;
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
   bringUp(Config{}.useObjectStatus().exportJointObjectStatus());
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kDetectedWhileClosing;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, reports_reached_at_the_requested_position)
{
   bringUp();
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kAtRequestedPosition;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, decides_on_a_direct_change_between_settled_states)
{
   bringUp();
   object_status_ = kAtRequestedPosition;
   sendGoal(0.5);
   object_status_ = kDetectedWhileClosing;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, takes_the_first_reading_after_acceptance_as_the_baseline)
{
   bringUp();
   object_status_ = kNaN;
   sendGoal(0.5);
   expectStillActive();
   object_status_ = kAtRequestedPosition;
   expectStillActive();

   object_status_ = kDetectedWhileClosing;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, reports_the_gripper_verdict_over_the_goal_tolerance)
{
   bringUp(Config{}.useObjectStatus().exportJointObjectStatus());
   object_status_ = kMoving;
   sendGoal(0.5);
   position_ = 0.5;
   object_status_ = kDetectedWhileClosing;

   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_TRUE(result->result->stalled);
   EXPECT_FALSE(result->result->reached_goal);
}

TEST_F(GripperActionControllerTest, aborts_a_goal_the_gripper_never_decides)
{
   bringUp();
   object_status_ = kDetectedWhileClosing;
   sendGoal(0.5);
   expectStillActive();

   update(kPastTimeout);
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_FALSE(result->result->stalled);
   EXPECT_FALSE(result->result->reached_goal);

   // The next goal is timed from its own acceptance.
   sendGoal(0.6);
   update(kWithinTimeout);
   expectStillActive();
}

TEST_F(GripperActionControllerTest, times_a_goal_from_the_last_motion_seen)
{
   bringUp();
   object_status_ = kAtRequestedPosition;
   sendGoal(0.5);
   object_status_ = kMoving;
   update(kWithinTimeout);
   // Past the acceptance's deadline, within the motion's.
   update(kPastTimeout);
   expectStillActive();

   update(kWithinTimeout + kPastTimeout);
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
}

TEST_F(GripperActionControllerTest, holds_while_the_reading_is_missing)
{
   bringUp();
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kNaN;
   expectStillActive();

   object_status_ = kAtRequestedPosition;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
}

TEST_F(GripperActionControllerTest, takes_a_new_baseline_for_the_next_goal)
{
   bringUp();
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(awaitResult().has_value());

   sendGoal(0.0);
   expectStillActive();
   object_status_ = kMoving;
   expectStillActive();

   object_status_ = kAtRequestedPosition;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, decides_on_the_starting_value_once_motion_was_seen)
{
   bringUp();
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(awaitResult().has_value());

   sendGoal(0.6);
   expectStillActive();
   object_status_ = kMoving;
   expectStillActive();

   object_status_ = kDetectedWhileClosing;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->stalled);
}

TEST_F(GripperActionControllerTest, refuses_a_non_positive_object_status_timeout)
{
   for(const double timeout : {0.0, -1.0})
   {
      init(Config{}.useObjectStatus());
      controller_->get_node()->set_parameter(rclcpp::Parameter("object_status_timeout", timeout));
      EXPECT_NE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, controller_->configure().id()) << timeout;
      executor_.remove_node(controller_->get_node()->get_node_base_interface());
   }
}

TEST_F(GripperActionControllerTest, refuses_to_activate_without_the_object_status)
{
   init(Config{}.useObjectStatus());
   assign(Config{});
   configure();
   EXPECT_NE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, controller_->get_node()->activate().id());
}

TEST_F(GripperActionControllerTest, decides_again_after_a_deactivation)
{
   bringUp();
   object_status_ = kMoving;
   sendGoal(0.5);
   object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(awaitResult().has_value());

   deactivate();
   assign(Config::driver());
   activate();
   sendGoal(0.0);
   expectStillActive();
   object_status_ = kAtRequestedPosition;
   const std::optional<Result> result = awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
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
