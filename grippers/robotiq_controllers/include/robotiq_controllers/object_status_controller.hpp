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

//! A stock gripper action controller deciding stall and goal states from the
//! gripper's own object detection instead of the velocity stall timeout:
//! parallel_gripper_controller's from Jazzy on, gripper_controllers' on Humble.
//! Humble EOL: simplify — one base remains, so this can become a plain class.

#pragma once

#include <exception>
#include <functional>
#include <limits>
#include <optional>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "robotiq_controllers/gripper_status.hpp"
#include "robotiq_controllers/object_status_goal.hpp"
#include "robotiq_controllers/ros2_control_compat.hpp"
#include "robotiq_driver/gripper_scaling.hpp"

namespace robotiq_controllers {

template <class Base>
class ObjectStatusController : public Base
{
public:
   controller_interface::InterfaceConfiguration state_interface_configuration() const override
   {
      controller_interface::InterfaceConfiguration configuration = Base::state_interface_configuration();
      if(use_object_status_)
      {
         configuration.names.push_back(object_status_goal::interfaceName(this->params_.joint));
      }
      return configuration;
   }

   controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override
   {
      if(object_status_)
      {
         // Reasserted every cycle rather than once at activation: only the gripper
         // may call a stall, and a base that refreshed params_ would bring the
         // velocity one back.
         this->params_.stall_timeout = std::numeric_limits<double>::infinity();
         decideFromObjectStatus(time);
      }
      // A goal decided above is no longer active, so the stock check finds nothing to do.
      return Base::update(time, period);
   }

   controller_interface::CallbackReturn on_init() override
   {
      if(Base::on_init() != controller_interface::CallbackReturn::SUCCESS)
      {
         return controller_interface::CallbackReturn::ERROR;
      }
      try
      {
         use_object_status_ = this->template auto_declare<bool>(object_status_goal::kUseParameter, false);
         object_status_timeout_ = this->template auto_declare<double>(object_status_goal::kTimeoutParameter,
                                                                      object_status_goal::kDefaultTimeout);
         if constexpr(!compat::detail::HasRobotDescription<Base>::value)
         {
            // Humble EOL: delete; the URDF supplies it from Jazzy on.
            this->template auto_declare<double>(robotiq_driver::kClosedPositionParam,
                                                std::numeric_limits<double>::quiet_NaN());
         }
      }
      catch(const std::exception& e)
      {
         RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to declare the object status parameters: %s", e.what());
         return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
   }

   controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override
   {
      const controller_interface::CallbackReturn result = Base::on_configure(previous_state);
      if(result != controller_interface::CallbackReturn::SUCCESS)
      {
         return result;
      }
      use_object_status_ = this->get_node()->get_parameter(object_status_goal::kUseParameter).as_bool();
      object_status_timeout_ = this->get_node()->get_parameter(object_status_goal::kTimeoutParameter).as_double();
      if(!(object_status_timeout_ > 0.0))
      {
         RCLCPP_ERROR(this->get_node()->get_logger(),
                      "%s must be positive, got %g.",
                      object_status_goal::kTimeoutParameter,
                      object_status_timeout_);
         return controller_interface::CallbackReturn::ERROR;
      }
      closed_position_ = closedPosition();
      if(use_object_status_ && !closed_position_)
      {
         RCLCPP_WARN(this->get_node()->get_logger(),
                     "No %s for joint '%s': a goal repeating the last one waits for %s to change.",
                     robotiq_driver::kClosedPositionParam,
                     this->params_.joint.c_str(),
                     object_status_goal::kTimeoutParameter);
      }
      return controller_interface::CallbackReturn::SUCCESS;
   }

   controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
   {
      const controller_interface::CallbackReturn result = Base::on_activate(previous_state);
      if(result != controller_interface::CallbackReturn::SUCCESS || !use_object_status_)
      {
         return result;
      }

      object_status_ = object_status_goal::findInterface(this->state_interfaces_,
                                                         this->params_.joint,
                                                         this->get_node()->get_logger());
      if(!object_status_)
      {
         return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
   }

   controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
   {
      object_status_.reset();
      tracked_goal_.reset();
      verdict_ = object_status_goal::Verdict();
      return Base::on_deactivate(previous_state);
   }

protected:
   using RealtimeGoalHandlePtr = typename Base::RealtimeGoalHandlePtr;

private:
   void decideFromObjectStatus(const rclcpp::Time& time)
   {
      const std::optional<RealtimeGoalHandlePtr> goal = compat::tryGet<RealtimeGoalHandlePtr>(this->rt_active_goal_);
      if(!goal || !*goal)
      {
         return;
      }

      const std::optional<Robotiq::ObjectDetection> detection =
         gripper_status::toObjectDetection(compat::getValue(object_status_->get()));
      if(*goal != tracked_goal_)
      {
         tracked_goal_ = *goal;
         verdict_.reset(time, detection, object_status_timeout_, positionRequest(**goal));
         return;
      }
      if(const std::optional<object_status_goal::Outcome> outcome = verdict_.decide(time, detection))
      {
         finish(*goal, outcome.value());
      }
   }

   // The register count the driver sends for the goal's position.
   template <class RealtimeGoalHandle>
   std::optional<uint8_t> positionRequest(const RealtimeGoalHandle& goal) const
   {
      const std::optional<double> position = compat::goalPosition(*goal.gh_->get_goal());
      if(!position || !closed_position_)
      {
         return std::nullopt;
      }
      return robotiq_driver::registerFromJointPosition(*position, *closed_position_);
   }

   std::optional<double> closedPosition()
   {
      if constexpr(compat::detail::HasRobotDescription<Base>::value)
      {
         return object_status_goal::closedPositionFromUrdf(this->get_robot_description(), this->params_.joint);
      }
      else
      {
         // Humble EOL: delete this branch.
         const double closed_position =
            this->get_node()->get_parameter(robotiq_driver::kClosedPositionParam).as_double();
         return robotiq_driver::isValidClosedPosition(closed_position) ? std::optional(closed_position) : std::nullopt;
      }
   }

   void finish(const RealtimeGoalHandlePtr& goal, const object_status_goal::Outcome& outcome)
   {
      const std::optional<double> position = compat::getValue(this->joint_position_state_interface_->get());
      if(!position)
      {
         return;
      }
      compat::setResult(*this->pre_alloc_result_, position.value(), this->computed_command_);
      this->pre_alloc_result_->reached_goal = outcome.reached_goal;
      this->pre_alloc_result_->stalled = outcome.stalled;
      if(outcome.reached_goal || (outcome.stalled && this->params_.allow_stalling))
      {
         goal->setSucceeded(this->pre_alloc_result_);
      }
      else
      {
         goal->setAborted(this->pre_alloc_result_);
      }
      compat::set(this->rt_active_goal_, RealtimeGoalHandlePtr());
   }

   bool use_object_status_ = false;
   double object_status_timeout_ = 0.0;
   std::optional<double> closed_position_;
   std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> object_status_;
   RealtimeGoalHandlePtr tracked_goal_;
   object_status_goal::Verdict verdict_;
};
} // namespace robotiq_controllers
