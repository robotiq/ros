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
#include <string>

#include "robotiq_controllers/gripper_status.hpp"
#include "robotiq_controllers/ros2_control_compat.hpp"

namespace robotiq_controllers {
namespace {
constexpr const char* kUseObjectStatusParameter = "use_object_status";
constexpr const char* kObjectStatusTimeoutParameter = "object_status_timeout";
constexpr double kDefaultObjectStatusTimeout = 10.0;
constexpr const char* kObjectStatusInterface = gripper_status::kInterfaceNames.at(gripper_status::OBJECT_STATUS);
} // namespace

controller_interface::InterfaceConfiguration GripperActionController::state_interface_configuration() const
{
   controller_interface::InterfaceConfiguration configuration = Base::state_interface_configuration();
   if(use_object_status_)
   {
      configuration.names.push_back(params_.joint + "/" + kObjectStatusInterface);
   }
   return configuration;
}

controller_interface::return_type GripperActionController::update(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
   if(object_status_)
   {
      // Reasserted every cycle rather than once at activation: only the gripper
      // may call a stall, and a base that refreshed params_ would bring the
      // velocity one back.
      params_.stall_timeout = std::numeric_limits<double>::infinity();
      decideFromObjectStatus(time);
   }
   // A goal decided above is no longer active, so the stock check finds nothing to do.
   return Base::update(time, period);
}

void GripperActionController::decideFromObjectStatus(const rclcpp::Time& time)
{
   RealtimeGoalHandlePtr goal;
   if(!rt_active_goal_.try_get([&](const RealtimeGoalHandlePtr& active) { goal = active; }) || !goal)
   {
      return;
   }

   const std::optional<Robotiq::ObjectDetection> detection =
      gripper_status::toObjectDetection(compat::getValue(object_status_->get()));
   if(goal != tracked_goal_)
   {
      tracked_goal_ = goal;
      timed_from_ = time;
      // gOBJ still holds the previous goal's verdict until the gripper acts on
      // the new target, so only a change from this reading counts.
      baseline_ = detection;
      return;
   }
   if(!baseline_)
   {
      // No reading at acceptance: the first one stands in for it.
      baseline_ = detection;
   }
   else if(detection && detection != baseline_)
   {
      if(detection == Robotiq::ObjectDetection::Moving)
      {
         // Motion seen: whatever the gripper settles on next is this goal's
         // verdict, even the value it started from, as when it tightens on the
         // object it already held.
         baseline_ = detection;
         timed_from_ = time;
      }
      else
      {
         const bool reached = detection == Robotiq::ObjectDetection::AtRequestedPosition;
         finish(goal, reached, !reached);
         return;
      }
   }
   if((time - timed_from_).seconds() >= object_status_timeout_)
   {
      finish(goal, false, false);
   }
}

void GripperActionController::finish(const RealtimeGoalHandlePtr& goal, bool reached_goal, bool stalled)
{
   const std::optional<double> position = compat::getValue(joint_position_state_interface_->get());
   if(!position)
   {
      return;
   }
   pre_alloc_result_->state.position[0] = position.value();
   pre_alloc_result_->state.effort[0] = computed_command_;
   pre_alloc_result_->reached_goal = reached_goal;
   pre_alloc_result_->stalled = stalled;
   if(reached_goal || (stalled && params_.allow_stalling))
   {
      goal->setSucceeded(pre_alloc_result_);
   }
   else
   {
      goal->setAborted(pre_alloc_result_);
   }
   rt_active_goal_.set([](RealtimeGoalHandlePtr& active) { active = RealtimeGoalHandlePtr(); });
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
      object_status_timeout_ = auto_declare<double>(kObjectStatusTimeoutParameter, kDefaultObjectStatusTimeout);
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare the object status parameters: %s", e.what());
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
   object_status_timeout_ = get_node()->get_parameter(kObjectStatusTimeoutParameter).as_double();
   if(!(object_status_timeout_ > 0.0))
   {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%s must be positive, got %g.",
                   kObjectStatusTimeoutParameter,
                   object_status_timeout_);
      return CallbackReturn::ERROR;
   }
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperActionController::on_activate(
   const rclcpp_lifecycle::State& previous_state)
{
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
      RCLCPP_ERROR(get_node()->get_logger(),
                   "%s is set but joint '%s' exports no %s. Mock and topic-based hardware do not report it; "
                   "use the driver against a gripper or its simulation, or unset the parameter.",
                   kUseObjectStatusParameter,
                   params_.joint.c_str(),
                   kObjectStatusInterface);
      return CallbackReturn::ERROR;
   }
   object_status_ = *object_status;
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
} // namespace robotiq_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::GripperActionController, controller_interface::ControllerInterface)
