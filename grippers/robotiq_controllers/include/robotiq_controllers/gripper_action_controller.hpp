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

//! The stock ParallelGripperCommand controller, deciding a goal from the
//! gripper's own object detection instead of the velocity stall timeout.

#pragma once

#include <cstdint>
#include <functional>
#include <optional>

#include "hardware_interface/loaned_state_interface.hpp"
#include "parallel_gripper_controller/parallel_gripper_action_controller.hpp"

namespace robotiq_controllers {

// With use_object_status set, a goal is stalled when the joint's object_status
// interface reports an object and reached when it reports the requested
// position or the stock goal tolerance is met; the stock velocity timeout only
// runs where the joint exports no object_status. Off, the controller is the
// stock one.
class GripperActionController : public parallel_gripper_action_controller::GripperActionController
{
public:
   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
   controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   CallbackReturn on_init() override;
   CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
   CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
   CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
   using Base = parallel_gripper_action_controller::GripperActionController;

   void moveJointInterfacesFirst();
   void decideFromObjectStatus();

   bool use_object_status_ = false;
   double stall_timeout_ = 0.0;
   std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> object_status_;
   RealtimeGoalHandlePtr tracked_goal_;
   std::optional<uint8_t> baseline_;
};
} // namespace robotiq_controllers
