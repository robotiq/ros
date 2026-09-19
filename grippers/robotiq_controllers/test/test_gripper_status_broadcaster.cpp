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

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <robotiq_controllers/gripper_status_broadcaster.hpp>

#include "controller_test_compat.hpp"

namespace robotiq_controllers::test {
namespace {
constexpr const char* kControllerName = "test_gripper_status_broadcaster";
constexpr const char* kJoint = "robotiq_85_left_knuckle_joint";
constexpr unsigned int kUpdateRate = 100;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// Stand-ins for both LoanedStateInterface shapes, so each distro tests the branch
// it does not compile. Humble EOL: keep NewApiLoaned only.
class OldApiLoaned
{
public:
   explicit OldApiLoaned(hardware_interface::StateInterface& state_interface)
      : name_(state_interface.get_interface_name())
   {
   }

   const std::string& name() const { return name_; }

private:
   std::string name_;
};

class NewApiLoaned
{
public:
   explicit NewApiLoaned(std::shared_ptr<const hardware_interface::StateInterface> state_interface)
      : name_(state_interface->get_interface_name())
   {
   }

   const std::string& name() const { return name_; }

private:
   std::string name_;
};

TEST(GripperStatusBroadcasterLoanTest, loans_through_either_interface_shape)
{
   double value = 0.0;
   const std::shared_ptr<hardware_interface::StateInterface> state_interface =
      test_compat::makeStateInterface(kJoint, "object_status", &value);

   EXPECT_EQ("object_status", test_compat::loanAs<OldApiLoaned>(state_interface).name());
   EXPECT_EQ("object_status", test_compat::loanAs<NewApiLoaned>(state_interface).name());
}

class GripperStatusBroadcasterTest : public ::testing::Test
{
protected:
   using Status = robotiq_msgs::msg::GripperStatus;

   void SetUp() override
   {
      broadcaster_ = std::make_unique<GripperStatusBroadcaster>();
      ASSERT_EQ(controller_interface::return_type::OK, test_compat::init(*broadcaster_, kControllerName, kUpdateRate));

      subscriber_ = std::make_shared<rclcpp::Node>("status_subscriber");
      subscription_ =
         subscriber_->create_subscription<Status>(std::string{"/"} + kControllerName + "/status",
                                                  rclcpp::SystemDefaultsQoS(),
                                                  [this](Status::SharedPtr message) { received_ = *message; });
      executor_.add_node(subscriber_);
   }

   // Loans position and velocity, which the ALL claim always brings in, plus the
   // gripper's four status interfaces when asked.
   void assign(bool with_status)
   {
      std::vector<hardware_interface::LoanedStateInterface> loans;
      owned_.clear();

      auto add = [&](const std::string& name, double* value) {
         owned_.push_back(test_compat::makeStateInterface(kJoint, name, value));
         loans.push_back(test_compat::loan(owned_.back()));
      };

      add("position", &position_);
      add("velocity", &velocity_);
      if(with_status)
      {
         add("object_status", &object_status_);
         add("motor_current", &motor_current_);
         add("gripper_fault", &gripper_fault_);
         add("gripper_fault_severity", &gripper_fault_severity_);
      }

      broadcaster_->assign_interfaces({}, std::move(loans));
   }

   void configure() { ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, broadcaster_->configure().id()); }

   void activate()
   {
      ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, broadcaster_->get_node()->activate().id());
   }

   void deactivate()
   {
      ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, broadcaster_->get_node()->deactivate().id());
      broadcaster_->release_interfaces();
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

   void waitForDiscovery()
   {
      spinUntil([this] { return subscription_->get_publisher_count() > 0; });
      ASSERT_GT(subscription_->get_publisher_count(), 0u) << "the subscriber never discovered the broadcaster";
   }

   // update() re-offers the message while the publisher thread still holds the
   // buffer; earlier cycles' messages may still be in flight, hence `accept`.
   std::optional<Status> receive(const std::function<bool(const Status&)>& accept = [](const Status&) { return true; })
   {
      received_.reset();
      const rclcpp::Duration period = rclcpp::Duration::from_seconds(1.0 / kUpdateRate);
      spinUntil([&] {
         EXPECT_EQ(controller_interface::return_type::OK, broadcaster_->update(subscriber_->now(), period));
         return received_ && accept(received_.value());
      });
      return received_;
   }

   std::unique_ptr<GripperStatusBroadcaster> broadcaster_;
   std::vector<std::shared_ptr<hardware_interface::StateInterface>> owned_;

   rclcpp::Node::SharedPtr subscriber_;
   rclcpp::Subscription<Status>::SharedPtr subscription_;
   rclcpp::executors::SingleThreadedExecutor executor_;
   std::optional<Status> received_;

   double position_ = kNaN;
   double velocity_ = kNaN;
   double object_status_ = kNaN;
   double motor_current_ = kNaN;
   double gripper_fault_ = kNaN;
   double gripper_fault_severity_ = kNaN;
};

TEST_F(GripperStatusBroadcasterTest, publishes_what_it_reads)
{
   assign(true);
   configure();
   activate();
   waitForDiscovery();

   object_status_ = 3.0;
   motor_current_ = 0.1;
   gripper_fault_ = 0.0;
   gripper_fault_severity_ = 0.0;

   const std::optional<Status> status = receive();
   ASSERT_TRUE(status.has_value()) << "nothing arrived on the status topic";
   EXPECT_EQ(Status::AT_REQUESTED_POSITION, status->object_detection);
   EXPECT_DOUBLE_EQ(motor_current_, status->motor_current);
}

TEST_F(GripperStatusBroadcasterTest, stays_active_where_the_gripper_exports_nothing)
{
   assign(false);
   configure();
   activate();

   EXPECT_EQ(controller_interface::return_type::OK,
             broadcaster_->update(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
}

TEST_F(GripperStatusBroadcasterTest, publishes_again_after_a_deactivation)
{
   assign(true);
   configure();
   activate();
   waitForDiscovery();
   object_status_ = 0.0;
   gripper_fault_ = 0.0;
   gripper_fault_severity_ = 0.0;
   ASSERT_TRUE(receive().has_value());

   deactivate();
   assign(true);
   activate();
   object_status_ = 3.0;

   const std::optional<Status> status =
      receive([](const Status& status) { return status.object_detection == Status::AT_REQUESTED_POSITION; });
   ASSERT_TRUE(status.has_value());
   EXPECT_EQ(Status::AT_REQUESTED_POSITION, status->object_detection);
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
