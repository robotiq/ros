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

#include "robotiq_controllers/gripper_status_broadcaster.hpp"

#include <limits>
#include <string>
#include <vector>

#include "robotiq_controllers/ros2_control_compat.hpp"

namespace robotiq_controllers {
using Status = robotiq_msgs::msg::GripperStatus;

controller_interface::InterfaceConfiguration GripperStatusBroadcaster::command_interface_configuration() const
{
   controller_interface::InterfaceConfiguration config;
   config.type = controller_interface::interface_configuration_type::NONE;

   return config;
}

controller_interface::InterfaceConfiguration GripperStatusBroadcaster::state_interface_configuration() const
{
   controller_interface::InterfaceConfiguration config;
   config.type = controller_interface::interface_configuration_type::ALL;

   return config;
}

controller_interface::return_type GripperStatusBroadcaster::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& /*period*/)
{
   if(!interfaces_)
   {
      return controller_interface::return_type::OK;
   }
   if(const std::optional<Status> status = gripper_status::decode(read(), time))
   {
      compat::tryPublish(*realtime_publisher_, status.value());
   }

   return controller_interface::return_type::OK;
}

gripper_status::Readings GripperStatusBroadcaster::read() const
{
   gripper_status::Readings readings;
   for(std::size_t field = 0; field < gripper_status::FIELD_COUNT; ++field)
   {
      const std::optional<std::size_t>& index = interfaces_->at(field);
      // An unbound field and a read that failed both leave the reading empty:
      // a contended handle has no value to report this cycle either.
      readings.at(field) = index ? compat::getValue(state_interfaces_.at(index.value())) : std::optional<double>{};
   }
   return readings;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperStatusBroadcaster::on_init()
{
   return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperStatusBroadcaster::on_configure(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   try
   {
      publisher_ = get_node()->create_publisher<Status>("~/status", rclcpp::SystemDefaultsQoS());
      realtime_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<Status>>(publisher_);
   }
   catch(const std::exception& e)
   {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create the status publisher: %s", e.what());
      return LifecycleNodeInterface::CallbackReturn::ERROR;
   }

   return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperStatusBroadcaster::on_activate(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   std::vector<gripper_status::InterfaceName> names;
   names.reserve(state_interfaces_.size());
   for(const auto& interface : state_interfaces_)
   {
      names.push_back({interface.get_prefix_name(), interface.get_interface_name()});
   }

   interfaces_ = gripper_status::bindInterfaces(names);
   if(!interfaces_)
   {
      RCLCPP_WARN(get_node()->get_logger(),
                  "This hardware exports no gripper status, so nothing will be published on ~/status. Mock and "
                  "topic-based hardware do not report it; use the driver against a gripper or its simulation.");
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
   }

   const std::string& joint = names.at(interfaces_->at(gripper_status::OBJECT_STATUS).value()).prefix;
   RCLCPP_INFO(get_node()->get_logger(), "Publishing the status of joint '%s' on ~/status.", joint.c_str());

   return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperStatusBroadcaster::on_deactivate(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   interfaces_.reset();

   return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn GripperStatusBroadcaster::on_cleanup(
   const rclcpp_lifecycle::State& /*previous_state*/)
{
   realtime_publisher_.reset();
   publisher_.reset();

   return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
} // namespace robotiq_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::GripperStatusBroadcaster, controller_interface::ControllerInterface)
