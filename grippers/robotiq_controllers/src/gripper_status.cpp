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

#include "robotiq_controllers/gripper_status.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robotiq_controllers::gripper_status {
namespace {
// State interfaces are doubles, so a code arrives as one.
// Humble EOL: remove this cast. In jazzy+, a uint8 interface can be used.
std::optional<uint8_t> asCode(double value)
{
   const double rounded = std::round(value);
   if(!std::isfinite(value) || rounded < 0.0 || rounded > 255.0)
   {
      return std::nullopt;
   }
   return static_cast<uint8_t>(rounded);
}

std::optional<uint8_t> asCode(const std::optional<double>& value)
{
   return value ? asCode(value.value()) : std::nullopt;
}
} // namespace

std::optional<Indices> bindInterfaces(const std::vector<InterfaceName>& interfaces)
{
   // OBJECT_STATUS anchors the binding because decode() cannot publish without
   // it: every object_detection value is a real state, so none can mean unknown.
   const auto gripper = std::find_if(interfaces.begin(), interfaces.end(), [](const InterfaceName& interface) {
      return interface.name == kInterfaceNames.at(OBJECT_STATUS);
   });
   if(gripper == interfaces.end())
   {
      return std::nullopt;
   }

   Indices indices;
   indices.fill(std::nullopt);
   for(std::size_t i = 0; i < interfaces.size(); ++i)
   {
      if(interfaces[i].prefix != gripper->prefix)
      {
         continue;
      }
      for(std::size_t field = 0; field < FIELD_COUNT; ++field)
      {
         if(interfaces[i].name == kInterfaceNames.at(field))
         {
            indices.at(field) = i;
         }
      }
   }

   return indices;
}

std::optional<robotiq_msgs::msg::GripperStatus> decode(const Readings& readings, const rclcpp::Time& time)
{
   const std::optional<uint8_t> object_detection = asCode(readings.at(OBJECT_STATUS));
   if(!object_detection)
   {
      return std::nullopt;
   }

   robotiq_msgs::msg::GripperStatus status;
   status.header.stamp = time;
   status.object_detection = object_detection.value();
   status.motor_current = readings.at(MOTOR_CURRENT).value_or(std::numeric_limits<double>::quiet_NaN());

   // A fault field the joint does not export keeps the message default, no fault.
   for(const auto& [field, code] : {std::pair{GRIPPER_FAULT, &status.gripper_fault},
                                    std::pair{GRIPPER_FAULT_SEVERITY, &status.gripper_fault_severity}})
   {
      if(!readings.at(field))
      {
         continue;
      }
      const std::optional<uint8_t> value = asCode(readings.at(field));
      if(!value)
      {
         return std::nullopt;
      }
      *code = value.value();
   }

   return status;
}

} // namespace robotiq_controllers::gripper_status
