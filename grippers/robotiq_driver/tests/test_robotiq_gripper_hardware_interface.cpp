// Copyright (c) 2023 PickNik, Inc.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>
#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif

#include <robotiq_driver/rclcpp_logger.hpp>

#include <rclcpp/node.hpp>

#include <robotiq_driver/hardware_interface.hpp>

#include "ros2_control_compat.hpp"

namespace robotiq_driver::test {

namespace {
constexpr const char* kComponentName = "robotiq_driver_ros2_control";

//! \p extra_hardware_params is spliced into the <hardware> block, so a test
//! can select a backend without restating the whole description.
std::string minimalRobotUrdf(const std::string& extra_hardware_params = "")
{
   return std::string(R"(
        <?xml version="1.0" encoding="utf-8"?>
        <robot name="test_robot">
          <link name="robotiq_85_base_link"/>
          <link name="robotiq_85_left_knuckle_link"/>
          <joint name="robotiq_85_left_knuckle_joint" type="revolute">
            <parent link="robotiq_85_base_link" />
            <child link="robotiq_85_left_knuckle_link" />
            <axis xyz="0 -1 0" />
            <origin xyz="0.03060114 0.0 0.05490452" rpy="0 0 0" />
            <limit lower="0.0" upper="0.8" velocity="0.5" effort="50" />
          </joint>
          <ros2_control name="robotiq_driver_ros2_control" type="system">
            <hardware>
              <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
              <param name="gripper_speed_multiplier">1.0</param>
              <param name="gripper_force_multiplier">0.5</param>
              <param name="COM_port">/dev/ttyUSB0</param>
              <param name="gripper_closed_position">0.7929</param>
              )")
        + extra_hardware_params + R"(
            </hardware>
            <joint name="robotiq_85_left_knuckle_joint">
              <command_interface name="position" />
              <state_interface name="position">
                <param name="initial_value">0.7929</param>
              </state_interface>
              <state_interface name="velocity"/>
              <state_interface name="motor_current"/>
              <state_interface name="object_status"/>
              <state_interface name="gripper_fault"/>
              <state_interface name="gripper_fault_severity"/>
            </joint>
            <gpio name="reactivate_gripper">
              <command_interface name="reactivate_gripper_cmd" />
              <command_interface name="reactivate_gripper_response" />
            </gpio>
          </ros2_control>
        </robot>
        )";
}
} // namespace

/**
 * This test generates a minimal xacro robot configuration and loads the
 * hardware interface plugin.
 */
TEST(TestRobotiqGripperHardwareInterface, LoadsThePluginFromUrdf)
{
   const std::string urdf = minimalRobotUrdf();

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   // Initialize the resource manager
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   // Check interfaces
   EXPECT_EQ(1u, rm.system_components_size());
}

/**
 * The controller configuration shipped in robotiq_description
 * (robotiq_controllers.yaml) claims these command interfaces by name; if the
 * hardware stops exporting them under these exact names the gripper controller
 * fails to activate at runtime.
 */
TEST(TestRobotiqGripperHardwareInterface, ExportsExpectedCommandInterfaces)
{
   const std::string urdf = minimalRobotUrdf();

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   const auto keys = rm.command_interface_keys();
   EXPECT_THAT(keys,
               testing::IsSupersetOf({"robotiq_85_left_knuckle_joint/position",
                                      "robotiq_85_left_knuckle_joint/set_gripper_max_velocity",
                                      "robotiq_85_left_knuckle_joint/set_gripper_max_effort",
                                      "reactivate_gripper/reactivate_gripper_cmd",
                                      "reactivate_gripper/reactivate_gripper_response"}));
}

/**
 * Neither motor_current nor object_status is a ros2_control standard interface,
 * so nothing but this test pins their spelling.
 */
TEST(TestRobotiqGripperHardwareInterface, ExportsExpectedStateInterfaces)
{
   const std::string urdf = minimalRobotUrdf();

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   std::vector<std::string> joint_keys;
   for(const std::string& key : rm.state_interface_keys())
   {
      if(key.rfind("robotiq_85_left_knuckle_joint/", 0) == 0)
      {
         joint_keys.push_back(key);
      }
   }

   EXPECT_THAT(joint_keys,
               testing::UnorderedElementsAre("robotiq_85_left_knuckle_joint/position",
                                             "robotiq_85_left_knuckle_joint/velocity",
                                             "robotiq_85_left_knuckle_joint/motor_current",
                                             "robotiq_85_left_knuckle_joint/object_status",
                                             "robotiq_85_left_knuckle_joint/gripper_fault",
                                             "robotiq_85_left_knuckle_joint/gripper_fault_severity"));
}

TEST(TestRobotiqGripperHardwareInterface, ExportsEveryStateInterfaceWhateverTheDescriptionDeclares)
{
   std::string urdf = minimalRobotUrdf();
   const std::string declared = R"(              <state_interface name="motor_current"/>
              <state_interface name="object_status"/>
              <state_interface name="gripper_fault"/>
              <state_interface name="gripper_fault_severity"/>
)";
   ASSERT_NE(std::string::npos, urdf.find(declared));
   urdf.erase(urdf.find(declared), declared.size());

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   EXPECT_THAT(rm.state_interface_keys(),
               testing::IsSupersetOf({"robotiq_85_left_knuckle_joint/motor_current",
                                      "robotiq_85_left_knuckle_joint/object_status",
                                      "robotiq_85_left_knuckle_joint/gripper_fault",
                                      "robotiq_85_left_knuckle_joint/gripper_fault_severity"}));
}

/**
 * use_dummy brings the component all the way up with no gripper and no serial
 * port, and the joint position then follows the commanded position. It is the
 * only hardware-free path that exercises the real plugin — and therefore the
 * gripper and activation controllers, which bind to interfaces the
 * ros2_control mock does not export.
 */
TEST(TestRobotiqGripperHardwareInterface, UseDummyActivatesAndFollowsCommands)
{
   const std::string urdf = minimalRobotUrdf(R"(<param name="use_dummy">true</param>)");

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   rclcpp_lifecycle::State active{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                  hardware_interface::lifecycle_state_names::ACTIVE};
   ASSERT_EQ(hardware_interface::return_type::OK, rm.set_component_state(kComponentName, active))
      << "the dummy backend failed to configure and activate";

   const rclcpp::Time time{0};
   const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);

   auto position = rm.claim_state_interface("robotiq_85_left_knuckle_joint/position");
   auto command = rm.claim_command_interface("robotiq_85_left_knuckle_joint/position");

   // Activation seeds both sides from where the fingers actually are, so a
   // controller that adopts the position state as its hold target on activation
   // holds the gripper still instead of dragging it somewhere else.
   ASSERT_TRUE(compat::readWriteOk(rm.read(time, period)));
   const auto seeded_position = compat::getValue(position);
   ASSERT_TRUE(seeded_position.has_value());
   EXPECT_DOUBLE_EQ(compat::getValue(command).value_or(-1.0), seeded_position.value())
      << "the position command must start at the measured position, not at a fixed default";

   // Half closed, in joint radians against the 0.7929 closed position.
   constexpr double kTarget = 0.4;
   ASSERT_TRUE(compat::setValue(command, kTarget));
   ASSERT_TRUE(compat::readWriteOk(rm.write(time, period)));

   // The simulated gripper teleports, but the SDK's exchange cycle still has
   // to carry the command and bring the status back.
   const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
   bool reached = false;
   while(!reached && std::chrono::steady_clock::now() < deadline)
   {
      ASSERT_TRUE(compat::readWriteOk(rm.read(time, period)));
      reached = std::abs(compat::getValue(position).value_or(0.0) - kTarget) < 0.01;
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
   }
   EXPECT_TRUE(reached) << "joint position never followed the command; last read "
                        << compat::getValue(position).value_or(0.0);
}

/**
 * A smoke check on the unfaulted path: both fault interfaces exist and read
 * None. It cannot say more, because the fake gripper has no fault injection —
 * the decode itself is pinned over the whole nibble in test_gripper_scaling.
 */
TEST(TestRobotiqGripperHardwareInterface, FaultInterfacesReadNoFaultOnAHealthyGripper)
{
   const std::string urdf = minimalRobotUrdf(R"(<param name="use_dummy">true</param>)");

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   rclcpp_lifecycle::State active{lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                  hardware_interface::lifecycle_state_names::ACTIVE};
   ASSERT_EQ(hardware_interface::return_type::OK, rm.set_component_state(kComponentName, active));

   auto gripper_fault = rm.claim_state_interface("robotiq_85_left_knuckle_joint/gripper_fault");
   auto severity = rm.claim_state_interface("robotiq_85_left_knuckle_joint/gripper_fault_severity");

   ASSERT_TRUE(compat::readWriteOk(rm.read(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01))));

   EXPECT_DOUBLE_EQ(static_cast<double>(Robotiq::GripperFault::None), compat::getValue(gripper_fault).value_or(-1.0));
   EXPECT_DOUBLE_EQ(static_cast<double>(Robotiq::FaultSeverity::None), compat::getValue(severity).value_or(-1.0));
}

namespace {
//! The component's own ros2_control block, as on_init receives it.
hardware_interface::HardwareInfo hardwareInfo(const std::string& urdf)
{
   for(const hardware_interface::HardwareInfo& info : hardware_interface::parse_control_resources_from_urdf(urdf))
   {
      if(info.name == kComponentName)
      {
         return info;
      }
   }
   return {};
}

//! A description declaring one extra state interface, by name.
std::string urdfDeclaring(const std::string& interface)
{
   std::string urdf = minimalRobotUrdf();
   const std::string anchor = R"(              <state_interface name="velocity"/>
)";
   const std::size_t at = urdf.find(anchor);
   urdf.insert(at + anchor.size(), "              <state_interface name=\"" + interface + "\"/>\n");
   return urdf;
}
} // namespace

/**
 * A description may declare a subset of the exported interfaces, but not a name
 * the driver does not write: the joint would then carry a state nothing ever
 * updates. on_init is driven directly, because the resource manager rejects
 * such a description for its own reasons and would hide this one.
 */
TEST(TestRobotiqGripperHardwareInterface, RejectsAStateInterfaceTheDriverDoesNotExport)
{
   RobotiqGripperHardwareInterface driver;

   EXPECT_EQ(hardware_interface::CallbackReturn::ERROR,
             driver.on_init(compat::onInitParams(hardwareInfo(urdfDeclaring("effort")))))
      << "on_init accepted a state interface the driver never writes";
}

/**
 * The reverse: a name the driver does export is accepted, so the rejection
 * above is about the name and not about declaring an interface at all.
 */
TEST(TestRobotiqGripperHardwareInterface, AcceptsEveryStateInterfaceItExports)
{
   RobotiqGripperHardwareInterface driver;

   EXPECT_EQ(hardware_interface::CallbackReturn::SUCCESS,
             driver.on_init(compat::onInitParams(hardwareInfo(urdfDeclaring("gripper_fault_severity")))));
}

/**
 * Every state interface reads NaN until the gripper has answered. The sentinel
 * is load-bearing for object_status and gripper_fault, where 0 is a real
 * reading: "moving" and "no fault".
 */
TEST(TestRobotiqGripperHardwareInterface, StateInterfacesReadNaNBeforeActivation)
{
   const std::string urdf = minimalRobotUrdf(R"(<param name="use_dummy">true</param>)");

   rclcpp::Node node{"test_robotiq_gripper_hardware_interface"};

#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
   hardware_interface::ResourceManager rm(urdf, node.get_node_clock_interface(), node.get_node_logging_interface());
#else
   hardware_interface::ResourceManager rm(urdf);
#endif

   rclcpp_lifecycle::State inactive{lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                    hardware_interface::lifecycle_state_names::INACTIVE};
   ASSERT_EQ(hardware_interface::return_type::OK, rm.set_component_state(kComponentName, inactive));

   for(const char* interface : {"motor_current", "object_status", "gripper_fault", "gripper_fault_severity"})
   {
      auto handle = rm.claim_state_interface(std::string{"robotiq_85_left_knuckle_joint/"} + interface);
      EXPECT_TRUE(std::isnan(compat::getValue(handle).value_or(0.0))) << interface << " had a value to report";
   }
}

namespace {
//! White-box handle on the plugin: drives the lifecycle callbacks directly
//! over the SDK's fake gripper, with the exchange slowed down so the recovery
//! handshake spans many control cycles.
class RecoveringGripper : public RobotiqGripperHardwareInterface
{
public:
   RecoveringGripper()
   {
      logger_ = std::make_shared<RclcppLogger>(rclcpp::get_logger("RecoveringGripperTest"));
      parameters_.use_dummy = true;
      parameters_.closed_position = 0.7929;
      parameters_.connection.connectionFrequency = 20.0;
   }

   void requestRecovery() { reactivate_gripper_cmd_ = 1.0; }
   void commandPosition(double position) { gripper_position_command_ = position; }
   void commandSpeed(double speed) { gripper_speed_ = speed; }
   uint8_t commandedPositionRegister() const { return gripper_->getCommand().positionRequest; }
   uint8_t commandedSpeedRegister() const { return gripper_->getCommand().speed; }
   bool recoveryInFlight() const { return recovery_.valid(); }
   bool recoveryHasFinished() const
   {
      return recovery_.valid() && recovery_.wait_for(std::chrono::seconds{0}) == std::future_status::ready;
   }
   bool gripperActivated() const { return gripper_->getStatus().gripperStatus.activated(); }
};
} // namespace

/**
 * A GPIO recovery runs the reset handshake on a background thread. A
 * deactivation arriving while it is in flight must wait for it: both drive
 * rACT, and racing them lets the recovery re-assert rACT after the reset,
 * leaving the gripper activated after on_deactivate reported success.
 */
TEST(TestRobotiqGripperHardwareInterface, DeactivationWaitsForAnInFlightRecovery)
{
   RecoveringGripper gripper;
   const rclcpp_lifecycle::State state;
   using CallbackReturn = RobotiqGripperHardwareInterface::CallbackReturn;

   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_configure(state));
   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_activate(state));

   gripper.requestRecovery();
   ASSERT_EQ(hardware_interface::return_type::OK, gripper.read(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   ASSERT_TRUE(gripper.recoveryInFlight());

   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_deactivate(state));
   EXPECT_TRUE(gripper.recoveryHasFinished()) << "on_deactivate returned while the recovery was still running";
   // on_deactivate does not return until the gripper reports the bit cleared,
   // and the recovery that could re-assert it has been awaited above.
   EXPECT_FALSE(gripper.gripperActivated());

   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_cleanup(state));
}

/**
 * write() is suppressed while a recovery is in flight: the handshake drives
 * rACT itself, and a position command landing mid-reset aborts it.
 */
TEST(TestRobotiqGripperHardwareInterface, WriteIsSuppressedDuringARecovery)
{
   RecoveringGripper gripper;
   const rclcpp_lifecycle::State state;
   using CallbackReturn = RobotiqGripperHardwareInterface::CallbackReturn;

   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_configure(state));
   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_activate(state));

   gripper.commandPosition(0.0);
   ASSERT_EQ(hardware_interface::return_type::OK, gripper.write(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   const uint8_t before = gripper.commandedPositionRegister();

   gripper.requestRecovery();
   ASSERT_EQ(hardware_interface::return_type::OK, gripper.read(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   ASSERT_TRUE(gripper.recoveryInFlight());

   gripper.commandPosition(0.7929);
   EXPECT_EQ(hardware_interface::return_type::OK, gripper.write(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   EXPECT_EQ(before, gripper.commandedPositionRegister()) << "write() reached the gripper during a recovery";

   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_deactivate(state));
   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_cleanup(state));
}

/**
 * A controller can write a NaN to set_gripper_max_velocity. That register then
 * has no value to compute, so it keeps the one it had while the position it
 * could compute goes through.
 */
TEST(TestRobotiqGripperHardwareInterface, AnUnmappableSpeedKeepsThePreviousOne)
{
   RecoveringGripper gripper;
   const rclcpp_lifecycle::State state;
   using CallbackReturn = RobotiqGripperHardwareInterface::CallbackReturn;

   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_configure(state));
   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_activate(state));

   gripper.commandPosition(0.0);
   gripper.commandSpeed(0.05);
   ASSERT_EQ(hardware_interface::return_type::OK, gripper.write(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   const uint8_t speed = gripper.commandedSpeedRegister();
   const uint8_t position = gripper.commandedPositionRegister();
   ASSERT_GT(speed, 0);

   gripper.commandSpeed(std::numeric_limits<double>::quiet_NaN());
   gripper.commandPosition(0.7929);
   EXPECT_EQ(hardware_interface::return_type::OK, gripper.write(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   EXPECT_EQ(speed, gripper.commandedSpeedRegister()) << "a NaN speed reached the gripper as a register";
   EXPECT_NE(position, gripper.commandedPositionRegister()) << "the position command stopped following";

   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_deactivate(state));
   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_cleanup(state));
}

/**
 * on_cleanup with a recovery still in flight has to await it: the recovery
 * borrows the gripper it is about to destroy.
 */
TEST(TestRobotiqGripperHardwareInterface, CleanupAwaitsAnInFlightRecovery)
{
   RecoveringGripper gripper;
   const rclcpp_lifecycle::State state;
   using CallbackReturn = RobotiqGripperHardwareInterface::CallbackReturn;

   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_configure(state));
   ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_activate(state));

   gripper.requestRecovery();
   ASSERT_EQ(hardware_interface::return_type::OK, gripper.read(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   ASSERT_TRUE(gripper.recoveryInFlight());

   EXPECT_EQ(CallbackReturn::SUCCESS, gripper.on_cleanup(state));
   EXPECT_FALSE(gripper.recoveryInFlight()) << "on_cleanup returned with the recovery still outstanding";
}

/**
 * on_shutdown and on_error take the same path as on_cleanup, so an error
 * transition does not leave the exchange thread running.
 */
TEST(TestRobotiqGripperHardwareInterface, ShutdownAndErrorReleaseTheGripper)
{
   const rclcpp_lifecycle::State state;
   using CallbackReturn = RobotiqGripperHardwareInterface::CallbackReturn;

   for(const bool via_error : {false, true})
   {
      RecoveringGripper gripper;
      ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_configure(state));
      ASSERT_EQ(CallbackReturn::SUCCESS, gripper.on_activate(state));

      EXPECT_EQ(CallbackReturn::SUCCESS, via_error ? gripper.on_error(state) : gripper.on_shutdown(state));
      // read() reports ERROR once the gripper is gone, which is how we know it
      // was released rather than left running.
      EXPECT_EQ(hardware_interface::return_type::ERROR,
                gripper.read(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)));
   }
}

namespace {
// pluginlib finds the hardware interface through the ament index in the install
// tree, so the tests that load it by name need this build's install prefix on
// AMENT_PREFIX_PATH. Nothing else provides it — not ctest, not an IDE, not a
// shell — and prepending a prefix that is already there costs nothing.
void announceTheInstalledPlugin()
{
   const char* const search_path = std::getenv("AMENT_PREFIX_PATH");
   const std::string prefix = ROBOTIQ_DRIVER_INSTALL_PREFIX;
   setenv("AMENT_PREFIX_PATH", (search_path ? prefix + ":" + search_path : prefix).c_str(), 1);
}
} // namespace
} // namespace robotiq_driver::test

// main() for the whole package suite: the test files link into one binary, and
// rclcpp has to be up before any node is constructed.
int main(int argc, char** argv)
{
   robotiq_driver::test::announceTheInstalledPlugin();
   rclcpp::init(argc, argv);
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
