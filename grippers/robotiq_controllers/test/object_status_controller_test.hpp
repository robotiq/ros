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

//! The tests of ObjectStatusController, a typed suite each distro's controller
//! test instantiates on its own Traits: the controller, its action, the goal
//! for a position and the position in a result.
//! Humble EOL: simplify — one instantiation remains, so this can fold back
//! into test_gripper_action_controller.cpp as plain TEST_Fs.

#pragma once

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <robotiq_controllers/ros2_control_compat.hpp>
#include <robotiq_driver/gripper_scaling.hpp>

#include "controller_test_compat.hpp"
#include "gripper_urdf.hpp"

namespace robotiq_controllers::test {

constexpr const char* kJoint = "robotiq_85_left_knuckle_joint";
constexpr unsigned int kUpdateRate = 100;
constexpr double kGoalTolerance = 0.02;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// object_status_timeout is left at its default; the update times are relative
// to what the timeout is measured from.
constexpr double kObjectStatusTimeout = 10.0;
constexpr double kWithinTimeout = kObjectStatusTimeout / 2;
constexpr double kPastTimeout = kObjectStatusTimeout + 1.0;
constexpr double kClosedPosition = 0.8;
// One register count of the driver's conversion, in joint units.
constexpr double kOneCount = kClosedPosition / robotiq_driver::kGripperRange;
// A stock stall timeout no test outlasts.
constexpr double kStockStallNever = 3600.0;

constexpr double kMoving = static_cast<double>(Robotiq::ObjectDetection::Moving);
constexpr double kDetectedWhileClosing = static_cast<double>(Robotiq::ObjectDetection::DetectedWhileClosing);
constexpr double kAtRequestedPosition = static_cast<double>(Robotiq::ObjectDetection::AtRequestedPosition);

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
template <class Controller>
class Testable : public Controller
{
public:
   bool holdsGoal()
   {
      const auto goal = compat::tryGet<typename Controller::RealtimeGoalHandlePtr>(this->rt_active_goal_);
      return goal.has_value() && *goal != nullptr;
   }
};

template <class Traits>
class ObjectStatusControllerTest : public ::testing::Test
{
protected:
   using Action = typename Traits::Action;
   using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Action>;
   using Result = typename ClientGoalHandle::WrappedResult;

   void SetUp() override
   {
      client_node_ = std::make_shared<rclcpp::Node>("gripper_action_client");
      executor_.add_node(client_node_);
      client_ = rclcpp_action::create_client<Action>(client_node_, std::string{"/"} + Traits::kName + "/gripper_cmd");
   }

   void init(const Config& config)
   {
      controller_ = std::make_unique<Testable<typename Traits::Controller>>();
      ASSERT_EQ(controller_interface::return_type::OK,
                test_compat::init(*controller_,
                                  Traits::kName,
                                  kUpdateRate,
                                  {rclcpp::Parameter("joint", kJoint),
                                   rclcpp::Parameter("use_object_status", config.use_object_status),
                                   rclcpp::Parameter("allow_stalling", config.allow_stalling),
                                   rclcpp::Parameter("goal_tolerance", kGoalTolerance),
                                   rclcpp::Parameter("stall_timeout", config.stall_timeout),
                                   // Relays a verdict to the client without delaying awaitResult.
                                   rclcpp::Parameter("action_monitor_rate", 1000.0),
                                   // Humble EOL: delete; the robot description below supplies it.
                                   rclcpp::Parameter("gripper_closed_position", kClosedPosition)},
                                  gripperUrdf(kJoint, kClosedPosition)));
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
      auto goal_future = client_->async_send_goal(Traits::goal(kJoint, position));
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

   std::unique_ptr<Testable<typename Traits::Controller>> controller_;
   std::vector<std::shared_ptr<hardware_interface::StateInterface>> owned_states_;
   std::vector<std::shared_ptr<hardware_interface::CommandInterface>> owned_commands_;

   rclcpp::Node::SharedPtr client_node_;
   typename rclcpp_action::Client<Action>::SharedPtr client_;
   rclcpp::executors::SingleThreadedExecutor executor_;
   typename ClientGoalHandle::SharedPtr goal_handle_;
   std::shared_future<Result> result_future_;

   double position_ = 0.1;
   double velocity_ = 0.0;
   double object_status_ = kNaN;
   double position_command_ = kNaN;
};

TYPED_TEST_SUITE_P(ObjectStatusControllerTest);

// Names an instantiation after its action rather than gtest's default, its index.
struct NamedByAction
{
   template <class Traits>
   static std::string GetName([[maybe_unused]] int index)
   {
      return Traits::kAction;
   }
};

TYPED_TEST_P(ObjectStatusControllerTest, claims_the_object_status_only_with_the_flag_on)
{
   this->init(Config{}.useObjectStatus());
   this->configure();
   const controller_interface::InterfaceConfiguration with_flag = this->controller_->state_interface_configuration();
   EXPECT_EQ(controller_interface::interface_configuration_type::INDIVIDUAL, with_flag.type);
   EXPECT_EQ((std::vector<std::string>{std::string{kJoint} + "/position",
                                       std::string{kJoint} + "/velocity",
                                       std::string{kJoint} + "/object_status"}),
             with_flag.names);

   this->executor_.remove_node(this->controller_->get_node()->get_node_base_interface());
   this->init(Config{});
   this->configure();
   const controller_interface::InterfaceConfiguration stock = this->controller_->state_interface_configuration();
   EXPECT_EQ(controller_interface::interface_configuration_type::INDIVIDUAL, stock.type);
   EXPECT_EQ((std::vector<std::string>{std::string{kJoint} + "/position", std::string{kJoint} + "/velocity"}),
             stock.names);
}

TYPED_TEST_P(ObjectStatusControllerTest, ignores_the_object_status_when_off)
{
   this->bringUp(Config{}.exportJointObjectStatus().allowStalling().stallTimeout(kStockStallNever));
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;
   this->expectStillActive();

   this->position_ = 0.5;
   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TYPED_TEST_P(ObjectStatusControllerTest, waits_for_the_status_to_change_from_its_value_at_acceptance)
{
   this->bringUp();
   this->object_status_ = kAtRequestedPosition;
   this->sendGoal(0.5);
   this->expectStillActive();
   this->object_status_ = kMoving;
   this->expectStillActive();

   this->object_status_ = kDetectedWhileClosing;
   this->position_ = 0.3;
   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->stalled);
   EXPECT_FALSE(result->result->reached_goal);
   EXPECT_DOUBLE_EQ(0.3, TypeParam::position(*result->result));
}

TYPED_TEST_P(ObjectStatusControllerTest, aborts_a_stall_without_allow_stalling)
{
   this->bringUp(Config{}.useObjectStatus().exportJointObjectStatus());
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;

   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_TRUE(result->result->stalled);
}

TYPED_TEST_P(ObjectStatusControllerTest, reports_reached_at_the_requested_position)
{
   this->bringUp();
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kAtRequestedPosition;

   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TYPED_TEST_P(ObjectStatusControllerTest, reports_the_gripper_verdict_over_the_goal_tolerance)
{
   this->bringUp(Config{}.useObjectStatus().exportJointObjectStatus());
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->position_ = 0.5;
   this->object_status_ = kDetectedWhileClosing;

   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_TRUE(result->result->stalled);
   EXPECT_FALSE(result->result->reached_goal);
}

TYPED_TEST_P(ObjectStatusControllerTest, aborts_a_goal_the_gripper_never_decides)
{
   this->bringUp();
   this->object_status_ = kDetectedWhileClosing;
   this->sendGoal(0.5);
   this->expectStillActive();

   this->update(kPastTimeout);
   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::ABORTED, result->code);
   EXPECT_FALSE(result->result->stalled);
   EXPECT_FALSE(result->result->reached_goal);

   // The next goal is timed from its own acceptance.
   this->sendGoal(0.6);
   this->update(kWithinTimeout);
   this->expectStillActive();
}

TYPED_TEST_P(ObjectStatusControllerTest, takes_a_new_baseline_for_the_next_goal)
{
   this->bringUp();
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(this->awaitResult().has_value());

   this->sendGoal(0.0);
   this->expectStillActive();
   this->object_status_ = kMoving;
   this->expectStillActive();

   this->object_status_ = kAtRequestedPosition;
   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
   EXPECT_FALSE(result->result->stalled);
}

TYPED_TEST_P(ObjectStatusControllerTest, repeats_the_outcome_for_a_goal_on_the_same_count)
{
   this->bringUp();
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(this->awaitResult().has_value());

   this->sendGoal(0.5 + kOneCount / 100);
   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_EQ(rclcpp_action::ResultCode::SUCCEEDED, result->code);
   EXPECT_TRUE(result->result->stalled);
}

TYPED_TEST_P(ObjectStatusControllerTest, waits_for_a_change_for_a_goal_on_another_count)
{
   this->bringUp();
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(this->awaitResult().has_value());

   this->sendGoal(0.5 + kOneCount);
   this->expectStillActive();
}

TYPED_TEST_P(ObjectStatusControllerTest, repeats_no_outcome_across_a_deactivation)
{
   this->bringUp();
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(this->awaitResult().has_value());

   this->deactivate();
   this->assign(Config::driver());
   this->activate();
   this->sendGoal(0.5);
   this->expectStillActive();
}

TYPED_TEST_P(ObjectStatusControllerTest, refuses_a_non_positive_object_status_timeout)
{
   for(const double timeout : {0.0, -1.0})
   {
      this->init(Config{}.useObjectStatus());
      this->controller_->get_node()->set_parameter(rclcpp::Parameter("object_status_timeout", timeout));
      EXPECT_NE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, this->controller_->configure().id()) << timeout;
      this->executor_.remove_node(this->controller_->get_node()->get_node_base_interface());
   }
}

TYPED_TEST_P(ObjectStatusControllerTest, refuses_to_activate_without_the_object_status)
{
   this->init(Config{}.useObjectStatus());
   this->assign(Config{});
   this->configure();
   EXPECT_NE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, this->controller_->get_node()->activate().id());
}

TYPED_TEST_P(ObjectStatusControllerTest, decides_again_after_a_deactivation)
{
   this->bringUp();
   this->object_status_ = kMoving;
   this->sendGoal(0.5);
   this->object_status_ = kDetectedWhileClosing;
   ASSERT_TRUE(this->awaitResult().has_value());

   this->deactivate();
   this->assign(Config::driver());
   this->activate();
   this->sendGoal(0.0);
   this->expectStillActive();
   this->object_status_ = kAtRequestedPosition;
   const auto result = this->awaitResult();
   ASSERT_TRUE(result.has_value());
   EXPECT_TRUE(result->result->reached_goal);
}

REGISTER_TYPED_TEST_SUITE_P(ObjectStatusControllerTest,
                            claims_the_object_status_only_with_the_flag_on,
                            ignores_the_object_status_when_off,
                            waits_for_the_status_to_change_from_its_value_at_acceptance,
                            aborts_a_stall_without_allow_stalling,
                            reports_reached_at_the_requested_position,
                            reports_the_gripper_verdict_over_the_goal_tolerance,
                            aborts_a_goal_the_gripper_never_decides,
                            takes_a_new_baseline_for_the_next_goal,
                            repeats_the_outcome_for_a_goal_on_the_same_count,
                            waits_for_a_change_for_a_goal_on_another_count,
                            repeats_no_outcome_across_a_deactivation,
                            refuses_a_non_positive_object_status_timeout,
                            refuses_to_activate_without_the_object_status,
                            decides_again_after_a_deactivation);
} // namespace robotiq_controllers::test
