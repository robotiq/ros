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

// ros2_control and realtime_tools API differences across Humble, Jazzy and
// Lyrical. Each shim keys off the shape of the operation rather than a version
// threshold, because Jazzy's 4.x line keeps moving.
//
// Humble (ros2_control 2.x) exposes `void set_value(double)` and
// `double get_value()`. From Jazzy's 4.x line on, writes report success
// (`bool set_value(double)`) and reads can fail (`std::optional<double>
// get_optional()`), because the handles took a lock internally.
//
// realtime_tools renamed rather than split: Humble offers `tryPublish(msg)`,
// newer releases `try_publish(msg)`. Both take the message in one call.
//
// Humble EOL: delete this header, its test and its CMake entries; the call
// sites use set_value, get_optional and try_publish directly. The shims are
// templates so that the branch a distro lacks is never instantiated, a
// constraint that goes with the last of them.

namespace robotiq_controllers::compat {
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

template <typename PublisherT, typename MessageT, typename = void>
struct HasTryPublish : std::false_type
{
};

template <typename PublisherT, typename MessageT>
struct HasTryPublish<PublisherT,
                     MessageT,
                     std::void_t<decltype(std::declval<PublisherT&>().try_publish(std::declval<const MessageT&>()))>>
   : std::true_type
{
};
} // namespace detail

/// @returns whether the write succeeded; always true where the API cannot report failure.
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

/// @returns the value, or std::nullopt where the API can report a failed read.
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

/// @returns whether the message was taken; false when another thread holds the buffer.
template <typename PublisherT, typename MessageT>
bool tryPublish(PublisherT& publisher, const MessageT& message)
{
   if constexpr(detail::HasTryPublish<PublisherT, MessageT>::value)
   {
      return publisher.try_publish(message);
   }
   else
   {
      return publisher.tryPublish(message);
   }
}
} // namespace robotiq_controllers::compat
