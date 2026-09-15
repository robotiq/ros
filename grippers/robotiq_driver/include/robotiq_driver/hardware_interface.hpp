// Copyright (c) 2022 PickNik, Inc.
// Copyright (c) 2026 Robotiq, Inc.
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
//    * Neither the name of the {copyright_holder} nor the names of its
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

//! \brief ros2_control system interface for Robotiq 2F grippers, wrapping the
//!        Robotiq gripper SDK.
//! The SDK owns the serial link and runs its own exchange cycle, keeping a
//! process image that setCommand()/getStatus() reach without touching the bus.
//! read() and write() are therefore pure memory operations against that image
//! and never block the controller manager — this package runs no communication
//! thread of its own.

#pragma once

#include <future>
#include <limits>
#include <memory>
#include <vector>

#include <robotiq_driver/hardware_parameters.hpp>
#include <robotiq_driver/visibility_control.hpp>

#include <robotiq_driver/ros2_control_compat.hpp>

#include <Robotiq/gripper.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robotiq_driver {

inline constexpr const char* kObjectStatusInterface = "object_status";
inline constexpr const char* kMotorCurrentInterface = "motor_current";
inline constexpr const char* kGripperFaultInterface = "gripper_fault";
inline constexpr const char* kFaultSeverityInterface = "fault_severity";

//! ros2_control hardware interface for a Robotiq 2F gripper, driven through the
//! gripper SDK.
//! Member order carries an invariant: recovery_ is declared after gripper_ so
//! that destruction, which runs in reverse, joins the recovery before the
//! gripper it borrows goes away. Reordering compiles and then misbehaves at
//! shutdown.
class RobotiqGripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
   RCLCPP_SHARED_PTR_DEFINITIONS(RobotiqGripperHardwareInterface)

   ROBOTIQ_DRIVER_PUBLIC
   RobotiqGripperHardwareInterface();

   ROBOTIQ_DRIVER_PUBLIC
   ~RobotiqGripperHardwareInterface() override;

   /**
    * Read and validate the hardware parameters and the joint's interfaces.
    * Performs no I/O — the link is opened in on_configure.
    * @param params Structure with parameters for initializing this hardware component
    * (a HardwareComponentInterfaceParams from Jazzy on, a HardwareInfo on Humble —
    * see ros2_control_compat.hpp).
    * @returns CallbackReturn::SUCCESS if required data are provided and can be
    * parsed or CallbackReturn::ERROR if any error happens or data are missing.
    */
   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_init(const OnInitParams& params) override;

   /**
    * Open the serial link and start the SDK exchange cycle. Constructing the
    * gripper reads it once, so a gripper that is unpowered, unplugged or at
    * another slave address fails here rather than at the first read().
    * @param previous_state The previous state.
    * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
    */
   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

   /**
    * Stop the exchange cycle and close the link. The gripper keeps its
    * activation and its grip.
    */
   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

   /**
    * on_shutdown and on_error have the same behavior as on_cleanup
    */
   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

   /**
    * This method exposes position, velocity, motor current and object status of
    * joints for reading.
    */
   ROBOTIQ_DRIVER_PUBLIC
   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

   /**
    * This method exposes the joints targets for writing.
    */
   ROBOTIQ_DRIVER_PUBLIC
   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

   /**
    * Reset and activate the gripper, blocking until it reports completion.
    *
    * @param previous_state Unconfigured, Inactive, Active or Finalized.
    * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
    */
   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

   /**
    * @param previous_state Unconfigured, Inactive, Active or Finalized.
    * @returns CallbackReturn::SUCCESS or CallbackReturn::ERROR.
    */
   ROBOTIQ_DRIVER_PUBLIC
   CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

   /**
    * Read data from the hardware.
    */
   ROBOTIQ_DRIVER_PUBLIC
   hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

   /**
    * Write data to hardware.
    */
   ROBOTIQ_DRIVER_PUBLIC
   hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
   /**
    * @throw Robotiq::SerialIOException, Robotiq::DriverException as the
    *        Robotiq::Gripper constructor does.
    */
   virtual std::unique_ptr<Robotiq::Gripper> createGripper();

   GripperParameters parameters_;

   // Log sink handed to the SDK, forwarding its diagnostics to /rosout.
   std::shared_ptr<Robotiq::Logger> logger_;

   // Owns the serial link and the exchange thread; null until on_configure.
   std::unique_ptr<Robotiq::Gripper> gripper_;

   // recoverFromFault() blocks for the length of a calibration sweep, so the
   // reactivate_gripper GPIO runs it off the control loop.
   // Declared after gripper_ on purpose: destruction runs in reverse, so an
   // std::async future here is joined before the gripper it borrows is
   // destroyed.
   std::future<Robotiq::ActivationResult> recovery_;

   static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

   double gripper_position_ = 0.0;
   double gripper_velocity_ = 0.0;

   // gCU in amperes, and gOBJ verbatim. Doubles to match the other
   // interfaces: Jazzy can carry a uint8_t, but only through the
   // description-driven StateInterface constructor, and Humble cannot at all.
   //
   // NaN before the first reading, because every value in range is a real one.
   double gripper_motor_current_ = std::numeric_limits<double>::quiet_NaN();
   double gripper_object_status_ = std::numeric_limits<double>::quiet_NaN();

   // gFLT and its severity, both decoded through the SDK so that no consumer
   // re-derives either from the raw byte.
   double gripper_fault_ = std::numeric_limits<double>::quiet_NaN();
   double fault_severity_ = std::numeric_limits<double>::quiet_NaN();

   double gripper_position_command_ = 0.0;

   // The last command write() sent. A register the arithmetic cannot produce
   // this cycle keeps the value it had rather than a made-up one.
   Robotiq::GripperCommand command_ = Robotiq::GripperCommand::defaults();

   // Per instance, so one gripper's throttled diagnostics cannot silence
   // another's in a multi-gripper cell.
   rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};

   double reactivate_gripper_cmd_ = 0.0;
   double reactivate_gripper_response_ = 0.0;
   double gripper_force_ = 0.0;
   double gripper_speed_ = 0.0;
};

} // namespace robotiq_driver
