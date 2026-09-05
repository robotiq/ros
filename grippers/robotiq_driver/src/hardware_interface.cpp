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

#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <robotiq_driver/gripper_scaling.hpp>
#include <robotiq_driver/hardware_interface.hpp>
#include <robotiq_driver/rclcpp_logger.hpp>

#include <Robotiq/gripper/fake/gripper_factory.hpp>
#include <Robotiq/gripper/wait.hpp>

#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>

const auto kLogger = rclcpp::get_logger("RobotiqGripperHardwareInterface");

// Link-health and fault messages would otherwise repeat every control cycle.
constexpr auto kDiagnosticThrottleMs = 5000;

// How long on_deactivate waits for the gripper to acknowledge the reset. The
// exchange cycle sends it within a cycle or two; this only has to outlast a
// retry or a lost frame.
constexpr auto kDeactivationTimeout = std::chrono::seconds{2};

// Where on_activate leaves the fingers. Fully open matches what the gripper does
// on its own and what the driver did before the SDK; a parameter could choose it
// instead, which would save the travel when the caller wants them elsewhere.
constexpr uint8_t kPostActivationPosition = robotiq_driver::kGripperMinPos;

// Waiting out that move, following the SDK's move_gripper example: how long the
// gripper has to echo the request, how long to watch for the motion to start
// (advisory — a short move can finish first), and the cap on the move itself.
// The sequence below is the example's; it belongs in the SDK as a shared moveTo
// so both callers get it from one place.
constexpr auto kCommandEchoTimeout = std::chrono::seconds{1};
constexpr auto kMotionStartTimeout = std::chrono::milliseconds{200};
constexpr auto kMotionTimeout = std::chrono::seconds{5};

namespace robotiq_driver {
namespace {
const char* toString(Robotiq::ConnectionState state)
{
   switch(state)
   {
   case Robotiq::ConnectionState::Disconnected:
      return "Disconnected";
   case Robotiq::ConnectionState::Connecting:
      return "Connecting";
   case Robotiq::ConnectionState::Operational:
      return "Operational";
   case Robotiq::ConnectionState::Faulted:
      return "Faulted";
   }
   return "Unknown";
}

// export_state_interfaces() exports all four whatever the description says, so
// the names below are the whole contract: nothing here gates what appears.
bool declaresOnlySupportedStateInterfaces(const hardware_interface::ComponentInfo& joint)
{
   for(const hardware_interface::InterfaceInfo& state_interface : joint.state_interfaces)
   {
      if(!(state_interface.name == hardware_interface::HW_IF_POSITION
           || state_interface.name == hardware_interface::HW_IF_VELOCITY
           || state_interface.name == kMotorCurrentInterface || state_interface.name == kObjectStatusInterface))
      {
         RCLCPP_FATAL(kLogger,
                      "Joint '%s' has %s state interface. Expected %s, %s, %s or %s.",
                      joint.name.c_str(),
                      state_interface.name.c_str(),
                      hardware_interface::HW_IF_POSITION,
                      hardware_interface::HW_IF_VELOCITY,
                      kMotorCurrentInterface,
                      kObjectStatusInterface);
         return false;
      }
   }
   return true;
}

// Shared by the two messages that report whether the link came up. A failed
// connect is almost always one of these three being wrong, and the SDK logs
// only the port and baud rate, at debug level.
std::string describeLink(const Robotiq::ConnectionConfig& connection)
{
   std::ostringstream text;
   text << connection.serial.port << " at " << connection.serial.baudrate << " bps (slave address 0x" << std::hex
        << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<unsigned>(connection.modbusSlaveAddress)
        << ")";
   return text.str();
}

// A recovery future stays valid from launch until read() consumes its result,
// so `valid()` alone answers "is a recovery outstanding"; this answers the
// narrower "has it finished".
bool hasFinished(const std::future<Robotiq::ActivationResult>& recovery)
{
   return recovery.valid() && recovery.wait_for(std::chrono::seconds{0}) == std::future_status::ready;
}
} // namespace

RobotiqGripperHardwareInterface::RobotiqGripperHardwareInterface() = default;

RobotiqGripperHardwareInterface::~RobotiqGripperHardwareInterface() = default;

std::unique_ptr<Robotiq::Gripper> RobotiqGripperHardwareInterface::createGripper()
{
   if(parameters_.use_dummy)
   {
      RCLCPP_WARN(kLogger, "You are connected to a dummy driver, not a real gripper.");
      return Robotiq::makeFakeGripper(parameters_.connection, logger_);
   }
   return std::make_unique<Robotiq::Gripper>(parameters_.connection, logger_);
}

hardware_interface::CallbackReturn RobotiqGripperHardwareInterface::on_init(const OnInitParams& params)
{
   RCLCPP_DEBUG(kLogger, "on_init");

   if(hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
   {
      return CallbackReturn::ERROR;
   }

   // The SDK prints to stderr unless a sink is injected; send it to /rosout.
   logger_ = std::make_shared<RclcppLogger>(rclcpp::get_logger("RobotiqGripperSDK"));

   try
   {
      parameters_ = parseParameters(info_, kLogger);
   }
   catch(const std::exception& e)
   {
      RCLCPP_FATAL(kLogger, "Failed to read the hardware parameters: %s", e.what());
      return CallbackReturn::ERROR;
   }

   gripper_position_ = std::numeric_limits<double>::quiet_NaN();
   gripper_velocity_ = std::numeric_limits<double>::quiet_NaN();
   gripper_motor_current_ = std::numeric_limits<double>::quiet_NaN();
   gripper_object_status_ = std::numeric_limits<double>::quiet_NaN();
   gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
   reactivate_gripper_cmd_ = NO_NEW_CMD_;

   const hardware_interface::ComponentInfo& joint = info_.joints.at(0);

   // There is one command interface: position.
   if(joint.command_interfaces.size() != 1)
   {
      RCLCPP_FATAL(kLogger,
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
   }

   if(joint.command_interfaces.at(0).name != hardware_interface::HW_IF_POSITION)
   {
      RCLCPP_FATAL(kLogger,
                   "Joint '%s' has %s command interfaces found. '%s' expected.",
                   joint.name.c_str(),
                   joint.command_interfaces.at(0).name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
   }

   if(!declaresOnlySupportedStateInterfaces(joint))
   {
      return CallbackReturn::ERROR;
   }

   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotiqGripperHardwareInterface::on_configure(
   const rclcpp_lifecycle::State& previous_state)
{
   RCLCPP_DEBUG(kLogger, "on_configure");

   if(hardware_interface::SystemInterface::on_configure(previous_state) != CallbackReturn::SUCCESS)
   {
      return CallbackReturn::ERROR;
   }

   try
   {
      gripper_ = createGripper();
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(kLogger,
                   "Cannot connect to the Robotiq gripper on %s: %s. Once it is connected, retry with: ros2 control "
                   "set_hardware_component_state %s active",
                   describeLink(parameters_.connection).c_str(),
                   e.what(),
                   info_.name.c_str());
      return CallbackReturn::ERROR;
   }

   RCLCPP_INFO(kLogger,
               "Connected to the Robotiq gripper on %s, exchanging at %.1f Hz.",
               describeLink(parameters_.connection).c_str(),
               parameters_.connection.connectionFrequency);
   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotiqGripperHardwareInterface::on_cleanup(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   RCLCPP_DEBUG(kLogger, "on_cleanup");

   // An in-flight recovery borrows the gripper; let it finish first.
   if(recovery_.valid())
   {
      recovery_.wait();
      recovery_ = {};
   }
   gripper_.reset();

   return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotiqGripperHardwareInterface::on_shutdown(
   const rclcpp_lifecycle::State& previous_state)
{
   RCLCPP_DEBUG(kLogger, "on_shutdown");
   return on_cleanup(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RobotiqGripperHardwareInterface::on_error(
   const rclcpp_lifecycle::State& previous_state)
{
   RCLCPP_DEBUG(kLogger, "on_error");
   return on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> RobotiqGripperHardwareInterface::export_state_interfaces()
{
   RCLCPP_DEBUG(kLogger, "export_state_interfaces");

   std::vector<hardware_interface::StateInterface> state_interfaces;

   state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
   state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_));
   state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, kMotorCurrentInterface, &gripper_motor_current_));
   state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, kObjectStatusInterface, &gripper_object_status_));

   return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotiqGripperHardwareInterface::export_command_interfaces()
{
   RCLCPP_DEBUG(kLogger, "export_command_interfaces");

   std::vector<hardware_interface::CommandInterface> command_interfaces;

   command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name,
                                                                        hardware_interface::HW_IF_POSITION,
                                                                        &gripper_position_command_));

   command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_velocity", &gripper_speed_));
   // Seeded from the shipped full scale, not from parameters_.max_speed /
   // max_force: a description that lowers those gets a seed above its own
   // full scale, which write() then clamps to the top register value.
   gripper_speed_ = kMaxSpeedDefault * parameters_.speed_multiplier;

   command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, "set_gripper_max_effort", &gripper_force_));
   gripper_force_ = kMaxForceDefault * parameters_.force_multiplier;

   command_interfaces.emplace_back(
      hardware_interface::CommandInterface("reactivate_gripper", "reactivate_gripper_cmd", &reactivate_gripper_cmd_));
   command_interfaces.emplace_back(hardware_interface::CommandInterface("reactivate_gripper",
                                                                        "reactivate_gripper_response",
                                                                        &reactivate_gripper_response_));

   return command_interfaces;
}

hardware_interface::CallbackReturn RobotiqGripperHardwareInterface::on_activate(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   RCLCPP_DEBUG(kLogger, "on_activate");

   if(!gripper_)
   {
      RCLCPP_FATAL(kLogger, "on_activate reached without a configured gripper.");
      return CallbackReturn::ERROR;
   }

   // set some default values for joints
   if(std::isnan(gripper_position_))
   {
      gripper_position_ = 0;
      gripper_velocity_ = 0;
      gripper_position_command_ = 0;
   }

   // recoverFromFault(), not activate(): the manual's reset handshake, which
   // clears a latched fault and runs the calibration sweep. Activating
   // therefore releases any grip and sweeps the fingers.
   const Robotiq::ActivationResult result = Robotiq::recoverFromFault(*gripper_, parameters_.activation_timeout);
   if(result != Robotiq::ActivationResult::Activated)
   {
      // recoverFromFault() reports only Activated or Timeout.
      RCLCPP_FATAL(kLogger,
                   "Failed to communicate with the Robotiq gripper: activation did not complete within %d ms.",
                   static_cast<int>(parameters_.activation_timeout.count()));
      return CallbackReturn::ERROR;
   }

   // Activation completes with the fingers closed, and the gripper re-opens them
   // afterwards on its own. Command that move instead of waiting it out: the
   // courtesy move runs with rGTO clear, and gOBJ only describes motion while
   // rGTO is set, so it reads "moving" throughout and never settles. Commanding
   // sets rGTO and makes gOBJ mean what the SDK's move_gripper example relies on.
   //
   // Something has to wait for it either way. Publishing a position mid-move
   // hands a controller that seeds its hold target from the position state —
   // both gripper_controllers and parallel_gripper_controller do — a half-closed
   // pose to latch, which stops the fingers there.
   command_.positionRequest = kPostActivationPosition;
   command_.action.set(Robotiq::ActionRequestBit::GoTo);
   gripper_->setCommand(command_);

   if(!Robotiq::waitFor([&] { return gripper_->getStatus().positionRequestEcho == kPostActivationPosition; },
                        kCommandEchoTimeout))
   {
      RCLCPP_WARN(kLogger, "The gripper never echoed the post-activation position request.");
   }
   else if(!Robotiq::waitFor(
              [&] { return gripper_->getStatus().gripperStatus.objectDetection() == Robotiq::ObjectDetection::Moving; },
              kMotionStartTimeout))
   {
      RCLCPP_DEBUG(kLogger, "No motion seen after the post-activation command; the fingers may already be there.");
   }

   if(!Robotiq::waitFor(
         [&] { return gripper_->getStatus().gripperStatus.objectDetection() != Robotiq::ObjectDetection::Moving; },
         kMotionTimeout))
   {
      RCLCPP_WARN(kLogger,
                  "The gripper had not settled %d ms after the post-activation command; publishing its position "
                  "anyway.",
                  static_cast<int>(std::chrono::milliseconds{kMotionTimeout}.count()));
   }
   const Robotiq::GripperStatus status = gripper_->getStatus();

   // Seed both sides from that settled reading so the first exported state, and
   // any hold target derived from it, describe where the fingers actually are.
   gripper_position_ = jointPositionFromRegister(status.position, parameters_.closed_position);
   gripper_velocity_ = 0.0;
   gripper_motor_current_ = motorCurrentFromRegister(status.current);
   gripper_object_status_ = static_cast<double>(status.gripperStatus.objectDetection());
   gripper_position_command_ = gripper_position_;

   RCLCPP_INFO(kLogger, "Robotiq Gripper successfully activated!");
   return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotiqGripperHardwareInterface::on_deactivate(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   RCLCPP_DEBUG(kLogger, "on_deactivate");

   // An in-flight GPIO recovery drives rACT itself: resetting over it races
   // the handshake, which could re-assert rACT after the reset below and
   // leave the gripper activated. Let it finish first — read() still
   // reports its result.
   if(recovery_.valid())
   {
      recovery_.wait();
   }

   if(gripper_)
   {
      // Zeroing the whole command block clears rACT, which resets the
      // gripper, so it releases any grip.
      gripper_->setCommand(Robotiq::GripperCommand{});

      // The exchange is asynchronous: without waiting for the gripper to
      // report the reset, a cleanup following closely can stop the exchange
      // cycle before the command goes out, leaving the gripper activated.
      if(!Robotiq::waitFor([this] { return !gripper_->getStatus().gripperStatus.activated(); }, kDeactivationTimeout))
      {
         RCLCPP_ERROR(kLogger, "Failed to deactivate the Robotiq gripper: it did not clear its activation bit.");
         return CallbackReturn::ERROR;
      }
   }

   RCLCPP_INFO(kLogger, "Robotiq Gripper successfully deactivated!");
   return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                                      const rclcpp::Duration& /*period*/)
{
   if(!gripper_)
   {
      return hardware_interface::return_type::ERROR;
   }

   const Robotiq::GripperStatus status = gripper_->getStatus();
   gripper_position_ = jointPositionFromRegister(status.position, parameters_.closed_position);
   // The status block carries no velocity — the gripper reports position and
   // motor current only.
   gripper_velocity_ = 0.0;
   gripper_motor_current_ = motorCurrentFromRegister(status.current);
   gripper_object_status_ = static_cast<double>(status.gripperStatus.objectDetection());

   // A faulted link recovers by itself on the next successful exchange, so
   // this warns rather than errors; the position above is the last good
   // reading until it does.
   if(const Robotiq::ConnectionState connection = gripper_->connectionState();
      connection != Robotiq::ConnectionState::Operational)
   {
      RCLCPP_WARN_THROTTLE(kLogger,
                           diagnostic_clock_,
                           kDiagnosticThrottleMs,
                           "The Robotiq gripper on %s: link is %s; the reported position may be stale.",
                           parameters_.connection.serial.port.c_str(),
                           toString(connection));
   }

   if(status.faultStatus.gripperFault() != Robotiq::GripperFault::None)
   {
      RCLCPP_WARN_THROTTLE(kLogger,
                           diagnostic_clock_,
                           kDiagnosticThrottleMs,
                           "The Robotiq gripper on %s reports fault status 0x%02X.",
                           parameters_.connection.serial.port.c_str(),
                           status.faultStatus.raw());
   }

   if(!std::isnan(reactivate_gripper_cmd_))
   {
      reactivate_gripper_cmd_ = NO_NEW_CMD_;
      if(recovery_.valid())
      {
         RCLCPP_WARN(kLogger, "A gripper recovery is already running; ignoring the reactivation request.");
      }
      else
      {
         RCLCPP_INFO(kLogger,
                     "Recovering the Robotiq gripper: the reset releases any grip and sweeps the fingers through "
                     "their full range.");
         recovery_ = std::async(std::launch::async, [this] {
            return Robotiq::recoverFromFault(*gripper_, parameters_.activation_timeout);
         });
      }
   }

   if(hasFinished(recovery_))
   {
      const Robotiq::ActivationResult result = recovery_.get();
      recovery_ = {};
      if(result == Robotiq::ActivationResult::Activated)
      {
         reactivate_gripper_response_ = 1.0;
         RCLCPP_INFO(kLogger, "The Robotiq gripper recovered and is activated.");
      }
      else
      {
         // The response interface only ever reports success; a failure leaves
         // it as it was.
         RCLCPP_ERROR(kLogger, "The Robotiq gripper failed to recover.");
      }
   }

   return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
   if(!gripper_)
   {
      return hardware_interface::return_type::ERROR;
   }

   // The recovery handshake drives rACT itself; commanding over it would
   // abort the reset half-way through.
   if(recovery_.valid())
   {
      return hardware_interface::return_type::OK;
   }

   // Before a controller has written a setpoint there is nothing to command,
   // and a NaN would convert to a garbage register value.
   if(std::isnan(gripper_position_command_))
   {
      return hardware_interface::return_type::OK;
   }

   const std::optional<uint8_t> position =
      registerFromJointPosition(gripper_position_command_, parameters_.closed_position);
   const std::optional<uint8_t> speed = registerFromFractionOf(gripper_speed_, parameters_.max_speed);
   const std::optional<uint8_t> force = registerFromFractionOf(gripper_force_, parameters_.max_force);
   if(!position || !speed || !force)
   {
      RCLCPP_ERROR_THROTTLE(kLogger,
                            diagnostic_clock_,
                            kDiagnosticThrottleMs,
                            "Cannot map position %f, speed %f, force %f onto gripper registers with "
                            "gripper_closed_position %f, gripper_max_speed %f, gripper_max_force %f; "
                            "keeping the previous command.",
                            gripper_position_command_,
                            gripper_speed_,
                            gripper_force_,
                            parameters_.closed_position,
                            parameters_.max_speed,
                            parameters_.max_force);
   }

   command_.action.set(Robotiq::ActionRequestBit::GoTo);
   command_.positionRequest = position.value_or(command_.positionRequest);
   command_.speed = speed.value_or(command_.speed);
   command_.force = force.value_or(command_.force);
   gripper_->setCommand(command_);

   return hardware_interface::return_type::OK;
}

} // namespace robotiq_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_driver::RobotiqGripperHardwareInterface, hardware_interface::SystemInterface)
