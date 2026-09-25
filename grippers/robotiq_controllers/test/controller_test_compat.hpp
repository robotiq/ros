// Copyright (c) 2022 PickNik, Inc.
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

//! Shims over the parts of the controller and hardware-interface APIs that a
//!  test drives directly, and that differ across Humble, Jazzy and Lyrical.

#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#if __has_include(<controller_interface/controller_interface_params.hpp>)
#include <controller_interface/controller_interface_params.hpp>
#define ROBOTIQ_HAS_CONTROLLER_INTERFACE_PARAMS 1
#endif

namespace robotiq_controllers::test_compat {
namespace detail {
template <typename ControllerT, typename = void>
struct HasUrdfInit : std::false_type
{
};

template <typename ControllerT>
struct HasUrdfInit<
   ControllerT,
   std::void_t<decltype(std::declval<ControllerT&>()
                           .init(std::string{}, std::string{}, 0U, std::string{}, rclcpp::NodeOptions{}))>>
   : std::true_type
{
};
} // namespace detail

// Humble takes the controller name alone; Jazzy adds the description and the
// update rates, and its 4.4x line moves them into a struct. Parameter overrides
// stand in for the controller_manager's yaml.
// Humble EOL: delete the HasUrdfInit trait and the name-only branch.
template <typename ControllerT>
controller_interface::return_type init(ControllerT& controller,
                                       const std::string& name,
                                       unsigned int update_rate,
                                       const std::vector<rclcpp::Parameter>& parameter_overrides = {})
{
#ifdef ROBOTIQ_HAS_CONTROLLER_INTERFACE_PARAMS
   controller_interface::ControllerInterfaceParams params;
   params.controller_name = name;
   params.update_rate = update_rate;
   params.controller_manager_update_rate = update_rate;
   params.node_options = controller.define_custom_node_options().parameter_overrides(parameter_overrides);
   return controller.init(params);
#else
   if constexpr(detail::HasUrdfInit<ControllerT>::value)
   {
      return controller.init(name,
                             "",
                             update_rate,
                             "",
                             controller.define_custom_node_options().parameter_overrides(parameter_overrides));
   }
   else
   {
      // Humble's own default options, plus the overrides.
      return controller.init(name,
                             "",
                             rclcpp::NodeOptions()
                                .allow_undeclared_parameters(true)
                                .automatically_declare_parameters_from_overrides(true)
                                .parameter_overrides(parameter_overrides));
   }
#endif
}

// Humble loans from a reference, Lyrical from a shared_ptr, and only Jazzy
// accepts both. Templated on the loan so a test can substitute either shape.
// Humble EOL: drop the reference branch.
template <typename LoanedT, typename StateInterfaceT>
LoanedT loanAs(const std::shared_ptr<StateInterfaceT>& state_interface)
{
   if constexpr(std::is_constructible_v<LoanedT, std::shared_ptr<const StateInterfaceT>>)
   {
      return LoanedT(std::shared_ptr<const StateInterfaceT>(state_interface));
   }
   else
   {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      return LoanedT(*state_interface);
#pragma GCC diagnostic pop
   }
}

// The loan holds a reference to the interface, so the caller keeps it alive.
inline hardware_interface::LoanedStateInterface loan(
   const std::shared_ptr<hardware_interface::StateInterface>& state_interface)
{
   return loanAs<hardware_interface::LoanedStateInterface>(state_interface);
}

// The value-pointer constructor lets a test move a reading without going through
// hardware; Jazzy and later deprecate it but keep it.
inline std::shared_ptr<hardware_interface::StateInterface> makeStateInterface(const std::string& joint,
                                                                              const std::string& interface,
                                                                              double* value)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
   return std::make_shared<hardware_interface::StateInterface>(joint, interface, value);
#pragma GCC diagnostic pop
}

inline std::shared_ptr<hardware_interface::CommandInterface> makeCommandInterface(const std::string& joint,
                                                                                  const std::string& interface,
                                                                                  double* value)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
   return std::make_shared<hardware_interface::CommandInterface>(joint, interface, value);
#pragma GCC diagnostic pop
}

// Same two shapes as the state loan, with the shared_ptr one taking a deleter.
// A template, so that the shape a distro lacks is never compiled.
// Humble EOL: drop the reference branch.
template <typename LoanedT, typename CommandInterfaceT>
LoanedT loanCommandAs(const std::shared_ptr<CommandInterfaceT>& command_interface)
{
   if constexpr(std::is_constructible_v<LoanedT, std::shared_ptr<CommandInterfaceT>, typename LoanedT::Deleter>)
   {
      return LoanedT(command_interface, nullptr);
   }
   else
   {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      return LoanedT(*command_interface);
#pragma GCC diagnostic pop
   }
}

inline hardware_interface::LoanedCommandInterface loanCommand(
   const std::shared_ptr<hardware_interface::CommandInterface>& command_interface)
{
   return loanCommandAs<hardware_interface::LoanedCommandInterface>(command_interface);
}
} // namespace robotiq_controllers::test_compat
