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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <robotiq_controllers/object_status_goal.hpp>

#include "controller_test_compat.hpp"
#include "gripper_urdf.hpp"

namespace robotiq_controllers::object_status_goal::test {
namespace {
using Robotiq::ObjectDetection;

constexpr double kTimeout = 10.0;
constexpr uint8_t kPositionRequest = 128;
const rclcpp::Time kAccepted{100, 0, RCL_ROS_TIME};

rclcpp::Time later(double seconds)
{
   return kAccepted + rclcpp::Duration::from_seconds(seconds);
}

Verdict accepted(const std::optional<ObjectDetection>& detection)
{
   Verdict verdict;
   verdict.reset(kAccepted, detection, kTimeout, kPositionRequest);
   return verdict;
}

// A verdict whose last goal, requesting kPositionRequest, stalled on an object while closing.
Verdict afterAStall()
{
   Verdict verdict = accepted(ObjectDetection::Moving);
   EXPECT_TRUE(verdict.decide(later(1), ObjectDetection::DetectedWhileClosing).has_value());
   return verdict;
}

TEST(Verdict, waits_for_the_reading_to_change_from_its_value_at_acceptance)
{
   Verdict verdict = accepted(ObjectDetection::AtRequestedPosition);
   EXPECT_FALSE(verdict.decide(later(1), ObjectDetection::AtRequestedPosition).has_value());
   EXPECT_FALSE(verdict.decide(later(2), ObjectDetection::Moving).has_value());

   const std::optional<Outcome> outcome = verdict.decide(later(3), ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->stalled);
   EXPECT_FALSE(outcome->reached_goal);
}

TEST(Verdict, reaches_at_the_requested_position)
{
   Verdict verdict = accepted(ObjectDetection::Moving);
   const std::optional<Outcome> outcome = verdict.decide(later(1), ObjectDetection::AtRequestedPosition);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->reached_goal);
   EXPECT_FALSE(outcome->stalled);
}

TEST(Verdict, stalls_on_an_object_while_opening)
{
   Verdict verdict = accepted(ObjectDetection::Moving);
   const std::optional<Outcome> outcome = verdict.decide(later(1), ObjectDetection::DetectedWhileOpening);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->stalled);
}

TEST(Verdict, decides_on_a_direct_change_between_settled_states)
{
   Verdict verdict = accepted(ObjectDetection::AtRequestedPosition);
   const std::optional<Outcome> outcome = verdict.decide(later(1), ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->stalled);
}

TEST(Verdict, takes_the_first_reading_after_acceptance_as_the_baseline)
{
   Verdict verdict = accepted(std::nullopt);
   EXPECT_FALSE(verdict.decide(later(1), std::nullopt).has_value());
   // The previous goal's verdict, still reported: a baseline, not a decision.
   EXPECT_FALSE(verdict.decide(later(2), ObjectDetection::AtRequestedPosition).has_value());
   EXPECT_FALSE(verdict.decide(later(3), ObjectDetection::AtRequestedPosition).has_value());

   const std::optional<Outcome> outcome = verdict.decide(later(4), ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->stalled);
}

TEST(Verdict, holds_while_the_reading_is_missing)
{
   Verdict verdict = accepted(ObjectDetection::Moving);
   EXPECT_FALSE(verdict.decide(later(1), std::nullopt).has_value());

   const std::optional<Outcome> outcome = verdict.decide(later(2), ObjectDetection::AtRequestedPosition);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->reached_goal);
}

TEST(Verdict, decides_on_the_starting_value_once_motion_was_seen)
{
   // Tightening on the held object: the gripper moves, then detects it again.
   Verdict verdict = accepted(ObjectDetection::DetectedWhileClosing);
   EXPECT_FALSE(verdict.decide(later(1), ObjectDetection::DetectedWhileClosing).has_value());
   EXPECT_FALSE(verdict.decide(later(2), ObjectDetection::Moving).has_value());

   const std::optional<Outcome> outcome = verdict.decide(later(3), ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->stalled);
}

TEST(Verdict, gives_up_on_a_goal_undecided_at_the_timeout)
{
   Verdict verdict = accepted(ObjectDetection::DetectedWhileClosing);
   EXPECT_FALSE(verdict.decide(later(kTimeout / 2), ObjectDetection::DetectedWhileClosing).has_value());

   const std::optional<Outcome> outcome = verdict.decide(later(kTimeout), ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_FALSE(outcome->reached_goal);
   EXPECT_FALSE(outcome->stalled);
}

TEST(Verdict, times_the_goal_from_the_last_motion_seen)
{
   Verdict verdict = accepted(ObjectDetection::AtRequestedPosition);
   EXPECT_FALSE(verdict.decide(later(kTimeout / 2), ObjectDetection::Moving).has_value());
   // Past the acceptance's deadline, within the motion's.
   EXPECT_FALSE(verdict.decide(later(kTimeout), ObjectDetection::Moving).has_value());

   const std::optional<Outcome> outcome = verdict.decide(later(kTimeout * 1.5), ObjectDetection::Moving);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_FALSE(outcome->reached_goal);
   EXPECT_FALSE(outcome->stalled);
}

TEST(Verdict, lets_a_change_at_the_timeout_win)
{
   Verdict verdict = accepted(ObjectDetection::Moving);
   const std::optional<Outcome> outcome = verdict.decide(later(kTimeout), ObjectDetection::AtRequestedPosition);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->reached_goal);
}

TEST(Verdict, times_the_next_goal_from_its_own_acceptance)
{
   Verdict verdict = accepted(ObjectDetection::AtRequestedPosition);
   ASSERT_TRUE(verdict.decide(later(kTimeout), ObjectDetection::AtRequestedPosition).has_value());

   verdict.reset(later(kTimeout), ObjectDetection::AtRequestedPosition, kTimeout, kPositionRequest);
   EXPECT_FALSE(verdict.decide(later(kTimeout * 1.5), ObjectDetection::AtRequestedPosition).has_value());
   EXPECT_TRUE(verdict.decide(later(kTimeout * 2), ObjectDetection::AtRequestedPosition).has_value());
}

TEST(Verdict, repeats_the_outcome_for_the_same_position_request_at_the_same_reading)
{
   Verdict verdict = afterAStall();
   verdict.reset(later(2), ObjectDetection::DetectedWhileClosing, kTimeout, kPositionRequest);
   const std::optional<Outcome> outcome = verdict.decide(later(2), ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(outcome.has_value());
   EXPECT_TRUE(outcome->stalled);
   EXPECT_FALSE(outcome->reached_goal);

   verdict.reset(later(3), ObjectDetection::DetectedWhileClosing, kTimeout, kPositionRequest);
   EXPECT_TRUE(verdict.decide(later(3), ObjectDetection::DetectedWhileClosing).has_value());
}

TEST(Verdict, waits_for_a_change_for_another_position_request)
{
   Verdict verdict = afterAStall();
   verdict.reset(later(2), ObjectDetection::DetectedWhileClosing, kTimeout, uint8_t{kPositionRequest + 1});
   EXPECT_FALSE(verdict.decide(later(2), ObjectDetection::DetectedWhileClosing).has_value());
}

TEST(Verdict, waits_for_a_change_when_object_detection_changes)
{
   Verdict verdict = afterAStall();
   verdict.reset(later(2), ObjectDetection::Moving, kTimeout, kPositionRequest);
   EXPECT_FALSE(verdict.decide(later(2), ObjectDetection::Moving).has_value());
}

TEST(Verdict, waits_for_a_change_after_an_undecided_goal)
{
   Verdict verdict = afterAStall();
   verdict.reset(later(2), ObjectDetection::DetectedWhileClosing, kTimeout, uint8_t{0});
   verdict.reset(later(3), ObjectDetection::DetectedWhileClosing, kTimeout, kPositionRequest);
   EXPECT_FALSE(verdict.decide(later(3), ObjectDetection::DetectedWhileClosing).has_value());
}

TEST(Verdict, repeats_nothing_when_the_reading_is_unavailable)
{
   Verdict verdict = afterAStall();
   verdict.reset(later(2), std::nullopt, kTimeout, kPositionRequest); // new goal with null ObjectDetection
   EXPECT_FALSE(verdict.decide(later(2), std::nullopt).has_value()); // still waiting for the goal to be satisfied

   const std::optional<Outcome> outcome = verdict.decide(later(2 + kTimeout), std::nullopt); // timeout -> goal failure
   ASSERT_TRUE(outcome.has_value());
   EXPECT_FALSE(outcome->reached_goal);
   EXPECT_FALSE(outcome->stalled);
}

TEST(Verdict, repeats_no_timeout)
{
   Verdict verdict = accepted(ObjectDetection::DetectedWhileClosing);
   ASSERT_TRUE(verdict.decide(later(kTimeout), ObjectDetection::DetectedWhileClosing).has_value());
   verdict.reset(later(kTimeout), ObjectDetection::DetectedWhileClosing, kTimeout, kPositionRequest);
   EXPECT_FALSE(verdict.decide(later(kTimeout), ObjectDetection::DetectedWhileClosing).has_value());
}

TEST(Verdict, repeats_nothing_for_a_goal_without_a_position_request)
{
   Verdict verdict = afterAStall();
   verdict.reset(later(2), ObjectDetection::DetectedWhileClosing, kTimeout, std::nullopt);
   EXPECT_FALSE(verdict.decide(later(2), ObjectDetection::DetectedWhileClosing).has_value());
}

TEST(ClosedPositionFromUrdf, reads_the_driver_parameter_of_the_joint_s_hardware)
{
   const std::optional<double> closed_position =
      closedPositionFromUrdf(robotiq_controllers::test::gripperUrdf("finger_joint", 0.695), "finger_joint");
   ASSERT_TRUE(closed_position.has_value());
   EXPECT_DOUBLE_EQ(0.695, *closed_position);
}

TEST(ClosedPositionFromUrdf, finds_nothing_for_another_joint)
{
   EXPECT_FALSE(closedPositionFromUrdf(robotiq_controllers::test::gripperUrdf("finger_joint", 0.695), "other_joint"));
}

TEST(ClosedPositionFromUrdf, finds_nothing_without_the_parameter)
{
   EXPECT_FALSE(closedPositionFromUrdf(robotiq_controllers::test::gripperUrdf("finger_joint", ""), "finger_joint"));
}

TEST(ClosedPositionFromUrdf, finds_nothing_in_a_value_the_driver_rejects)
{
   for(const char* value : {"0", "nan", "inf", "wide open"})
   {
      EXPECT_FALSE(
         closedPositionFromUrdf(robotiq_controllers::test::gripperUrdf(
                                   "finger_joint",
                                   std::string(R"(<param name="gripper_closed_position">)") + value + "</param>"),
                                "finger_joint"))
         << value;
   }
}

TEST(ClosedPositionFromUrdf, finds_nothing_in_a_malformed_description)
{
   EXPECT_FALSE(closedPositionFromUrdf("not a urdf", "finger_joint"));
   EXPECT_FALSE(closedPositionFromUrdf("", "finger_joint"));
}

TEST(ProfileFromUrdf, ReadsTheDriverParameterOfTheJointsHardware)
{
   const std::optional<Robotiq::DeviceProfile> profile = profileFromUrdf(
      robotiq_controllers::test::gripperUrdf("finger_joint", R"(<param name="gripper_profile">hand_e</param>)"),
      "finger_joint");
   ASSERT_TRUE(profile.has_value());
   EXPECT_EQ(Robotiq::profiles::kHandE.closedPosition, profile->closedPosition);
}

TEST(ProfileFromUrdf, DefaultsToThe2F85LikeTheDriver)
{
   const std::optional<Robotiq::DeviceProfile> profile =
      profileFromUrdf(robotiq_controllers::test::gripperUrdf("finger_joint", 0.695), "finger_joint");
   ASSERT_TRUE(profile.has_value());
   EXPECT_EQ(Robotiq::profiles::k2F85.closedPosition, profile->closedPosition);
}

TEST(ProfileFromUrdf, FindsNothingInANameTheDriverRejects)
{
   EXPECT_FALSE(profileFromUrdf(
      robotiq_controllers::test::gripperUrdf("finger_joint", R"(<param name="gripper_profile">hand-e</param>)"),
      "finger_joint"));
}

TEST(InterfaceName, joins_the_joint_and_the_field)
{
   EXPECT_EQ("finger_joint/object_status", interfaceName("finger_joint"));
}

class FindInterface : public ::testing::Test
{
protected:
   void add(const std::string& joint, const std::string& name)
   {
      owned_.push_back(test_compat::makeStateInterface(joint, name, &value_));
      loaned_.push_back(test_compat::loan(owned_.back()));
   }

   std::vector<std::shared_ptr<hardware_interface::StateInterface>> owned_;
   std::vector<hardware_interface::LoanedStateInterface> loaned_;
   double value_ = 0.0;
   rclcpp::Logger logger_ = rclcpp::get_logger("test_object_status_goal");
};

TEST_F(FindInterface, finds_the_joint_s_object_status)
{
   add("joint", "position");
   add("other", "object_status");
   add("joint", "object_status");
   const auto found = findInterface(loaned_, "joint", logger_);
   ASSERT_TRUE(found.has_value());
   EXPECT_EQ(&loaned_[2], &found->get());
}

TEST_F(FindInterface, finds_nothing_for_another_joint_s_object_status)
{
   add("joint", "position");
   add("other", "object_status");
   EXPECT_FALSE(findInterface(loaned_, "joint", logger_).has_value());
}

TEST_F(FindInterface, finds_nothing_without_it)
{
   add("joint", "position");
   add("joint", "velocity");
   EXPECT_FALSE(findInterface(loaned_, "joint", logger_).has_value());
}
} // namespace
} // namespace robotiq_controllers::object_status_goal::test
