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

#include "robotiq_controllers/gripper_action_controller.hpp"

#include <algorithm>
#include <exception>
#include <limits>
#include <utility>
#include <vector>

#include "robotiq_controllers/gripper_status.hpp"
#include "robotiq_controllers/ros2_control_compat.hpp"
#include "robotiq_msgs/msg/gripper_status.hpp"

namespace robotiq_controllers {
namespace {
constexpr const char* kUseObjectStatusParameter = "use_object_status";
constexpr const char* kObjectStatusInterface = gripper_status::kInterfaceNames.at(gripper_status::OBJECT_STATUS);
using Status = robotiq_msgs::msg::GripperStatus;
} // namespace

controller_interface::InterfaceConfiguration GripperActionController::state_interface_configuration() const
{
   if(!use_object_status_)
   {
      return Base::state_interface_configuration();
   }
   // Whether the joint exports object_status is only known once the interfaces
   // are loaned, and an individual claim of a missing one fails activation.
   return {controller_interface::interface_configuration_type::ALL, {}};
}

controller_interface::return_type GripperActionController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
   if(object_status_)
   {
      decideFromObjectStatus();
   }
   // A goal decided above is no longer active, so the stock check finds nothing to do.
   return Base::update(time, period);
}

void GripperActionController::decideFromObjectStatus()
{
   RealtimeGoalHandlePtr goal;
   if(!rt_active_goal_.try_get([&](const RealtimeGoalHandlePtr& active) { goal = active; }))
   {
      return;
   }
   if(goal != tracked_goal_)
   {
      tracked_goal_ = goal;
      // gOBJ still holds the previous goal's verdict until the gripper acts on
      // the new target, so only a change from this reading counts. Without a
      // reading here the goal is left to the goal tolerance.
      baseline_ = gripper_status::asCode(compat::getValue(object_status_->get()));
   }
   if(!goal || !baseline_)
   {
      return;
   }

   const std::optional<uint8_t> detection = gripper_status::asCode(compat::getValue(object_status_->get()));
   if(!detection || detection == baseline_)
   {
      return;
   }
   if(detection == Status::MOVING)
   {
      // Motion seen: whatever the gripper settles on next is this goal's verdict,
      // even the value it started from.
      baseline_ = detection;
      return;
   }
   const std::optional<double> position = compat::getValue(joint_position_state_interface_->get());
   if(!position)
   {
      return;
   }

   pre_alloc_result_->state.position[0] = position.value();
   pre_alloc_result_->state.effort[0] = computed_command_;
   if(detection == Status::AT_REQUESTED_POSITION)
   {
      pre_alloc_result_->reached_goal = true;
      pre_alloc_result_->stalled = false;
      goal->setSucceeded(pre_alloc_result_);
   }
   else
   {
      pre_alloc_result_->reached_goal = false;
      pre_alloc_result_->stalled = true;
      if(params_.allow_stalling)
      {
         goal->setSucceeded(pre_alloc_result_);
      }
      else
      {
         goal->setAborted(pre_alloc_result_);
      }
   }
   rt_active_goal_.try_set([](RealtimeGoalHandlePtr& active) { active = RealtimeGoalHandlePtr(); });
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperActionController::on_init()
{
   if(Base::on_init() != CallbackReturn::SUCCESS)
   {
      return CallbackReturn::ERROR;
   }
   try
   {
      use_object_status_ = auto_declare<bool>(kUseObjectStatusParameter, false);
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare %s: %s", kUseObjectStatusParameter, e.what());
      return CallbackReturn::ERROR;
   }
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperActionController::on_configure(
   const rclcpp_lifecycle::State& previous_state)
{
   const CallbackReturn result = Base::on_configure(previous_state);
   if(result != CallbackReturn::SUCCESS)
   {
      return result;
   }
   use_object_status_ = get_node()->get_parameter(kUseObjectStatusParameter).as_bool();
   stall_timeout_ = params_.stall_timeout;
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperActionController::on_activate(
   const rclcpp_lifecycle::State& previous_state)
{
   if(use_object_status_)
   {
      moveJointInterfacesFirst();
   }
   params_.stall_timeout = stall_timeout_;
   const CallbackReturn result = Base::on_activate(previous_state);
   if(result != CallbackReturn::SUCCESS || !use_object_status_)
   {
      return result;
   }

   const auto object_status =
      std::find_if(state_interfaces_.begin(),
                   state_interfaces_.end(),
                   [this](const hardware_interface::LoanedStateInterface& i) {
                      return i.get_prefix_name() == params_.joint && i.get_interface_name() == kObjectStatusInterface;
                   });
   if(object_status == state_interfaces_.end())
   {
      RCLCPP_WARN(get_node()->get_logger(),
                  "%s is set but joint '%s' exports no %s, so goals are decided from the velocity as without it. "
                  "Mock and topic-based hardware do not report it; use the driver against a gripper or its simulation.",
                  kUseObjectStatusParameter,
                  params_.joint.c_str(),
                  kObjectStatusInterface);
      return CallbackReturn::SUCCESS;
   }
   object_status_ = *object_status;
   // Only the gripper may call a stall now: the stock timer would end a goal
   // on a velocity that no longer means anything here.
   params_.stall_timeout = std::numeric_limits<double>::infinity();
   RCLCPP_INFO(get_node()->get_logger(),
               "Deciding goals from the object detection of joint '%s'.",
               params_.joint.c_str());
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperActionController::on_deactivate(
   const rclcpp_lifecycle::State& previous_state)
{
   object_status_.reset();
   tracked_goal_.reset();
   baseline_.reset();
   return Base::on_deactivate(previous_state);
}

// Base::on_activate binds the first position and velocity interfaces it finds,
// whichever joint they belong to, and the ALL claim loans every joint's.
void GripperActionController::moveJointInterfacesFirst()
{
   std::vector<hardware_interface::LoanedStateInterface> ordered;
   std::vector<hardware_interface::LoanedStateInterface> others;
   ordered.reserve(state_interfaces_.size());
   for(hardware_interface::LoanedStateInterface& interface : state_interfaces_)
   {
      (interface.get_prefix_name() == params_.joint ? ordered : others).push_back(std::move(interface));
   }
   for(hardware_interface::LoanedStateInterface& interface : others)
   {
      ordered.push_back(std::move(interface));
   }
   state_interfaces_ = std::move(ordered);
}
} // namespace robotiq_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::GripperActionController, controller_interface::ControllerInterface)
