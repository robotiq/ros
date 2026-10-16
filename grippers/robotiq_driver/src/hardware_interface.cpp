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

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <robotiq_driver/gripper_scaling.hpp>
#include <robotiq_driver/hardware_interface.hpp>
#include <robotiq_driver/rclcpp_logger.hpp>

#include <Robotiq/gripper/fake/gripper_factory.hpp>
#include <Robotiq/gripper/to_string.hpp>
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
// The driver exports all of these whatever the description says, so the names
// below are the whole contract: nothing here gates what appears.
constexpr std::array<const char*, 6> kStateInterfaces{hardware_interface::HW_IF_POSITION,
                                                      hardware_interface::HW_IF_VELOCITY,
                                                      kMotorCurrentInterface,
                                                      kObjectStatusInterface,
                                                      kGripperFaultInterface,
                                                      kGripperFaultSeverityInterface};

bool declaresOnlySupportedStateInterfaces(const hardware_interface::ComponentInfo& joint)
{
   for(const hardware_interface::InterfaceInfo& state_interface : joint.state_interfaces)
   {
      if(std::find(kStateInterfaces.begin(), kStateInterfaces.end(), state_interface.name) == kStateInterfaces.end())
      {
         std::ostringstream expected;
         for(std::size_t i = 0; i < kStateInterfaces.size(); ++i)
         {
            expected << (i == 0 ? "" : ", ") << kStateInterfaces[i];
         }
         RCLCPP_FATAL(kLogger,
                      "Joint '%s' has %s state interface. Expected one of %s.",
                      joint.name.c_str(),
                      state_interface.name.c_str(),
                      expected.str().c_str());
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
   gripper_fault_ = {};
   gripper_position_command_ = std::numeric_limits<double>::quiet_NaN();
   reactivate_gripper_cmd_ = NO_NEW_CMD_;

   // The one GPIO is the reactivation handshake, under whatever name the
   // description gives it: a prefixed one keeps two grippers in one description
   // apart. Without one the driver exports it as kReactivateGripperGpio.
   if(info_.gpios.size() > 1)
   {
      RCLCPP_FATAL(kLogger,
                   "Component '%s' declares %zu GPIOs. At most 1 expected: the reactivation handshake.",
                   info_.name.c_str(),
                   info_.gpios.size());
      return CallbackReturn::ERROR;
   }
   reactivate_gpio_ = info_.gpios.empty() ? kReactivateGripperGpio : info_.gpios.front().name;

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
                   "set_hardware_component_state '%s' active",
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

#ifdef ROBOTIQ_DRIVER_INTERFACES_OWN_VALUES
namespace {
hardware_interface::InterfaceDescription describe(const std::string& prefix, const char* name)
{
   hardware_interface::InterfaceInfo info;
   info.name = name;
   return {prefix, info};
}

std::string fullName(const std::string& prefix, const char* name)
{
   return prefix + "/" + name;
}
} // namespace

std::vector<hardware_interface::InterfaceDescription> RobotiqGripperHardwareInterface::
   export_unlisted_state_interface_descriptions()
{
   const std::string& joint = info_.joints[0].name;
   std::vector<hardware_interface::InterfaceDescription> unlisted;
   for(const char* name : kStateInterfaces)
   {
      if(joint_state_interfaces_.count(fullName(joint, name)) == 0)
      {
         unlisted.push_back(describe(joint, name));
      }
   }
   return unlisted;
}

std::vector<hardware_interface::InterfaceDescription> RobotiqGripperHardwareInterface::
   export_unlisted_command_interface_descriptions()
{
   // position is the joint's one declared command interface; on_init requires it.
   const std::string& joint = info_.joints[0].name;
   std::vector<hardware_interface::InterfaceDescription> unlisted{describe(joint, kMaxVelocityInterface),
                                                                  describe(joint, kMaxEffortInterface)};
   for(const char* name : {kReactivateCmdInterface, kReactivateResponseInterface})
   {
      if(gpio_command_interfaces_.count(fullName(reactivate_gpio_, name)) == 0)
      {
         unlisted.push_back(describe(reactivate_gpio_, name));
      }
   }
   return unlisted;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> RobotiqGripperHardwareInterface::
   on_export_state_interfaces()
{
   RCLCPP_DEBUG(kLogger, "on_export_state_interfaces");

   std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces =
      SystemInterface::on_export_state_interfaces();

   const std::string& joint = info_.joints[0].name;
   exported_states_.clear();
   for(const auto& [name, value] :
       {std::pair<const char*, const double*>{hardware_interface::HW_IF_POSITION, &gripper_position_},
        {hardware_interface::HW_IF_VELOCITY, &gripper_velocity_},
        {kMotorCurrentInterface, &gripper_motor_current_},
        {kObjectStatusInterface, &gripper_object_status_},
        {kGripperFaultInterface, &gripper_fault_.code},
        {kGripperFaultSeverityInterface, &gripper_fault_.severity}})
   {
      exported_states_.emplace_back(get_state_interface_handle(fullName(joint, name)), value);
   }

   // The members over the description's initial_value: every state reads NaN
   // until the gripper has answered.
   pushStates();
   return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> RobotiqGripperHardwareInterface::
   on_export_command_interfaces()
{
   RCLCPP_DEBUG(kLogger, "on_export_command_interfaces");

   std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces =
      SystemInterface::on_export_command_interfaces();

   // Seeded from the shipped full scale, not from parameters_.max_speed /
   // max_force: a description that lowers those gets a seed above its own
   // full scale, which write() then clamps to the top register value.
   gripper_speed_ = kMaxSpeedDefault * parameters_.speed_multiplier;
   gripper_force_ = kMaxForceDefault * parameters_.force_multiplier;

   const std::string& joint = info_.joints[0].name;
   exported_commands_.clear();
   for(const auto& [prefix, name, value] :
       {std::tuple<std::string, const char*, double*>{joint,
                                                      hardware_interface::HW_IF_POSITION,
                                                      &gripper_position_command_},
        {joint, kMaxVelocityInterface, &gripper_speed_},
        {joint, kMaxEffortInterface, &gripper_force_},
        {reactivate_gpio_, kReactivateCmdInterface, &reactivate_gripper_cmd_},
        {reactivate_gpio_, kReactivateResponseInterface, &reactivate_gripper_response_}})
   {
      exported_commands_.emplace_back(get_command_interface_handle(fullName(prefix, name)), value);
      pushCommand(*value);
   }
   return command_interfaces;
}

// Blocking on the lock, as the framework's named accessors are: a skipped copy
// would drop a one-shot command such as a reactivation request.
void RobotiqGripperHardwareInterface::pullCommands()
{
   pullCommand(gripper_position_command_);
   pullCommand(gripper_speed_);
   pullCommand(gripper_force_);
}

void RobotiqGripperHardwareInterface::pullCommand(double& command)
{
   std::ignore = get_command(exportedHandle(command), command, true);
}

void RobotiqGripperHardwareInterface::pushStates()
{
   for(const auto& [handle, value] : exported_states_)
   {
      std::ignore = set_state(handle, *value, true);
   }
}

void RobotiqGripperHardwareInterface::pushCommand(const double& command)
{
   std::ignore = set_command(exportedHandle(command), command, true);
}

const hardware_interface::CommandInterface::SharedPtr& RobotiqGripperHardwareInterface::exportedHandle(
   const double& command) const
{
   const auto exported = std::find_if(exported_commands_.begin(), exported_commands_.end(), [&](const auto& pair) {
      return pair.second == &command;
   });
   if(exported == exported_commands_.end())
   {
      throw std::logic_error("The command is not bound to an exported interface.");
   }
   return exported->first;
}
#else
// Humble EOL: delete this branch.
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
   state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, kGripperFaultInterface, &gripper_fault_.code));
   state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name,
                                                                    kGripperFaultSeverityInterface,
                                                                    &gripper_fault_.severity));

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
      hardware_interface::CommandInterface(info_.joints[0].name, kMaxVelocityInterface, &gripper_speed_));
   // Seeded from the shipped full scale, not from parameters_.max_speed /
   // max_force: a description that lowers those gets a seed above its own
   // full scale, which write() then clamps to the top register value.
   gripper_speed_ = kMaxSpeedDefault * parameters_.speed_multiplier;

   command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[0].name, kMaxEffortInterface, &gripper_force_));
   gripper_force_ = kMaxForceDefault * parameters_.force_multiplier;

   command_interfaces.emplace_back(
      hardware_interface::CommandInterface(reactivate_gpio_, kReactivateCmdInterface, &reactivate_gripper_cmd_));
   command_interfaces.emplace_back(hardware_interface::CommandInterface(reactivate_gpio_,
                                                                        kReactivateResponseInterface,
                                                                        &reactivate_gripper_response_));

   return command_interfaces;
}

// The methods below are blank because the Humble interfaces read and write the members directly.
void RobotiqGripperHardwareInterface::pullCommands() {}

void RobotiqGripperHardwareInterface::pullCommand(double& /*command*/) {}

void RobotiqGripperHardwareInterface::pushStates() {}

void RobotiqGripperHardwareInterface::pushCommand(const double& /*command*/) {}
#endif

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

   if(!Robotiq::waitForPositionEcho(*gripper_, kPostActivationPosition, kCommandEchoTimeout))
   {
      RCLCPP_WARN(kLogger, "The gripper never echoed the post-activation position request.");
   }
   else if(!Robotiq::waitForObjectDetection(*gripper_, Robotiq::ObjectDetection::Moving, kMotionStartTimeout))
   {
      RCLCPP_DEBUG(kLogger, "No motion seen after the post-activation command; the fingers may already be there.");
   }

   const std::optional<Robotiq::StampedExchange> settled = Robotiq::waitForMotionEnd(*gripper_, kMotionTimeout);
   if(!settled)
   {
      RCLCPP_WARN(kLogger,
                  "The gripper had not settled %d ms after the post-activation command; publishing its position "
                  "anyway.",
                  static_cast<int>(std::chrono::milliseconds{kMotionTimeout}.count()));
   }
   const Robotiq::GripperStatus status = settled ? settled->status : gripper_->getStatus();

   // Seed both sides from that settled reading so the first exported state, and
   // any hold target derived from it, describe where the fingers actually are.
   gripper_position_ = jointPositionFromRegister(status.position, parameters_.closed_position);
   gripper_velocity_ = 0.0;
   gripper_motor_current_ = motorCurrentFromRegister(status.current);
   gripper_object_status_ = static_cast<double>(status.gripperStatus.objectDetection());
   gripper_fault_ = {gripperFaultFromRegister(status.faultStatus),
                     gripperFaultSeverityFromRegister(status.faultStatus)};
   gripper_position_command_ = gripper_position_;
   pushStates();
   pushCommand(gripper_position_command_);

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
      if(!Robotiq::waitFor(
            *gripper_,
            [](const Robotiq::StampedExchange& exchange) { return !exchange.status.gripperStatus.activated(); },
            kDeactivationTimeout))
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

   // The only command read() acts on; write() pulls the others.
   pullCommand(reactivate_gripper_cmd_);

   const Robotiq::GripperStatus status = gripper_->getStatus();
   const Robotiq::ConnectionState connection = gripper_->connectionState();
   gripper_position_ = jointPositionFromRegister(status.position, parameters_.closed_position);
   // The status block carries no velocity — the gripper reports position and
   // motor current only.
   gripper_velocity_ = 0.0;
   gripper_motor_current_ = motorCurrentFromRegister(status.current);
   gripper_object_status_ = objectStatusValue(status.gripperStatus.objectDetection(), connection);
   gripper_fault_ = {gripperFaultFromRegister(status.faultStatus),
                     gripperFaultSeverityFromRegister(status.faultStatus)};

   // A faulted link recovers by itself on the next successful exchange, so
   // this warns rather than errors; the position above is the last good
   // reading until it does.
   if(connection != Robotiq::ConnectionState::Operational)
   {
      RCLCPP_WARN_THROTTLE(kLogger,
                           diagnostic_clock_,
                           kDiagnosticThrottleMs,
                           "%s",
                           ("The Robotiq gripper on " + parameters_.connection.serial.port + ": link is "
                            + std::string{Robotiq::toString(connection)} + "; the reported position may be stale.")
                              .c_str());
   }

   if(status.faultStatus.gripperFault() != Robotiq::GripperFault::None)
   {
      const Robotiq::GripperFault fault = status.faultStatus.gripperFault();
      RCLCPP_WARN_THROTTLE(kLogger,
                           diagnostic_clock_,
                           kDiagnosticThrottleMs,
                           "%s",
                           ("The Robotiq gripper on " + parameters_.connection.serial.port + " reports gripper fault "
                            + std::string{Robotiq::toString(fault)} + " (code "
                            + std::to_string(static_cast<unsigned>(fault)) + ").")
                              .c_str());
   }

   if(!std::isnan(reactivate_gripper_cmd_))
   {
      reactivate_gripper_cmd_ = NO_NEW_CMD_;
      pushCommand(reactivate_gripper_cmd_);
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
         pushCommand(reactivate_gripper_response_);
         RCLCPP_INFO(kLogger, "The Robotiq gripper recovered and is activated.");
      }
      else
      {
         // The response interface only ever reports success; a failure leaves
         // it as it was.
         RCLCPP_ERROR(kLogger, "The Robotiq gripper failed to recover.");
      }
   }

   pushStates();
   return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotiqGripperHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
   if(!gripper_)
   {
      return hardware_interface::return_type::ERROR;
   }

   pullCommands();

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
