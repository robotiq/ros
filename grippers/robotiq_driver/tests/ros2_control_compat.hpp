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

#pragma once

#include <optional>
#include <type_traits>
#include <utility>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <robotiq_driver/ros2_control_compat.hpp>

// Only the tests drive ros2_control's handles and ResourceManager directly, so
// these shims live here rather than in the package's installed headers.
//
// Humble (ros2_control 2.x) has `void set_value(double)`, `double get_value()`
// and HardwareReadWriteStatus::ok; from Jazzy's 4.x line on, writes report
// success, reads can fail, and the status carries a return_type `result`. Each
// shim keys off the shape of the operation it wraps, not a version threshold,
// because Jazzy's 4.x line keeps moving.
//
// robotiq_controllers ships the same two handle shims in its own header. It does
// not depend on this package — and should not, since that would put the gripper
// SDK's build in front of the controllers — so the copies stay separate.
//
// Humble EOL: delete this header; the tests use set_value, get_optional,
// status.result and HardwareComponentInterfaceParams directly. The near-copy in
// robotiq_controllers needs the same pass.

namespace robotiq_driver::test::compat {
namespace detail {
template <typename HandleT, typename = void>
struct HasGetOptional : std::false_type
{
};

template <typename HandleT>
struct HasGetOptional<HandleT, std::void_t<decltype(std::declval<const HandleT&>().get_optional())>> : std::true_type
{
};

template <typename HandleT>
struct SetValueReturnsBool : std::is_same<decltype(std::declval<HandleT&>().set_value(0.0)), bool>
{
};

template <typename StatusT, typename = void>
struct HasResult : std::false_type
{
};

template <typename StatusT>
struct HasResult<StatusT, std::void_t<decltype(std::declval<const StatusT&>().result)>> : std::true_type
{
};
} // namespace detail

/// Write `value` to `handle`; always true where the API cannot report failure.
template <typename HandleT>
bool setValue(HandleT& handle, double value)
{
   if constexpr(detail::SetValueReturnsBool<HandleT>::value)
   {
      return handle.set_value(value);
   }
   else
   {
      handle.set_value(value);
      return true;
   }
}

/// Read `handle`; std::nullopt only where the API can report a failed read.
template <typename HandleT>
std::optional<double> getValue(const HandleT& handle)
{
   if constexpr(detail::HasGetOptional<HandleT>::value)
   {
      return handle.get_optional();
   }
   else
   {
      return handle.get_value();
   }
}

/// Whether a ResourceManager read()/write() cycle succeeded.
template <typename StatusT>
bool readWriteOk(const StatusT& status)
{
   if constexpr(detail::HasResult<StatusT>::value)
   {
      return status.result == hardware_interface::return_type::OK;
   }
   else
   {
      return status.ok;
   }
}

/// Wrap `info` in whatever SystemInterface::on_init() takes on this distro.
/// Templated so the branch this distro does not have is never instantiated.
template <typename ParamsT = robotiq_driver::OnInitParams>
ParamsT onInitParams(const hardware_interface::HardwareInfo& info)
{
   if constexpr(std::is_same_v<ParamsT, hardware_interface::HardwareInfo>)
   {
      return info;
   }
   else
   {
      ParamsT params;
      params.hardware_info = info;
      return params;
   }
}
} // namespace robotiq_driver::test::compat
