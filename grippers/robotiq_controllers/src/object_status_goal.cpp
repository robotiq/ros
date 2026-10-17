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

#include "robotiq_controllers/object_status_goal.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <exception>
#include <string>
#include <string_view>

#include "rclcpp/logging.hpp"
#include "robotiq_controllers/gripper_status.hpp"
#include "robotiq_driver/gripper_scaling.hpp"

namespace robotiq_controllers::object_status_goal {
namespace {
constexpr const char* kInterface = gripper_status::kInterfaceNames.at(gripper_status::OBJECT_STATUS);

bool drivesJoint(const tinyxml2::XMLElement& control, const std::string& joint)
{
   for(const tinyxml2::XMLElement* j = control.FirstChildElement("joint"); j; j = j->NextSiblingElement("joint"))
   {
      if(const char* name = j->Attribute("name"); name && joint == name)
      {
         return true;
      }
   }
   return false;
}

const tinyxml2::XMLElement* hardwareParameter(const tinyxml2::XMLElement& control, std::string_view name)
{
   const tinyxml2::XMLElement* hardware = control.FirstChildElement("hardware");
   for(const tinyxml2::XMLElement* param = hardware ? hardware->FirstChildElement("param") : nullptr; param;
       param = param->NextSiblingElement("param"))
   {
      if(const char* n = param->Attribute("name"); n && name == n)
      {
         return param;
      }
   }
   return nullptr;
}

// The text of the driver parameter \p name in the hardware block of \p urdf that
// drives \p joint; nothing when there is no such block or parameter.
std::optional<std::string> jointHardwareParameter(const std::string& urdf, const std::string& joint, const char* name)
{
   // A walk of the two tags needed rather than hardware_interface's parser,
   // which validates the whole description again, with rules that differ per
   // distro, to answer the same question.
   tinyxml2::XMLDocument document;
   if(document.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS || !document.RootElement())
   {
      return std::nullopt;
   }
   for(const tinyxml2::XMLElement* control = document.RootElement()->FirstChildElement("ros2_control"); control;
       control = control->NextSiblingElement("ros2_control"))
   {
      if(drivesJoint(*control, joint))
      {
         if(const tinyxml2::XMLElement* param = hardwareParameter(*control, name))
         {
            const char* text = param->GetText();
            return std::string(text ? text : "");
         }
      }
   }
   return std::nullopt;
}

std::optional<double> closedPosition(const std::string& text)
{
   try
   {
      const double closed_position = std::stod(text);
      return robotiq_driver::isValidClosedPosition(closed_position) ? std::optional(closed_position) : std::nullopt;
   }
   catch(const std::exception&)
   {
      return std::nullopt;
   }
}
} // namespace

std::string interfaceName(const std::string& joint)
{
   return joint + "/" + kInterface;
}

std::optional<double> closedPositionFromUrdf(const std::string& urdf, const std::string& joint)
{
   const std::optional<std::string> text = jointHardwareParameter(urdf, joint, robotiq_driver::kClosedPositionParam);
   return text ? closedPosition(*text) : std::nullopt;
}

std::optional<Robotiq::DeviceProfile> profileFromUrdf(const std::string& urdf, const std::string& joint)
{
   const std::optional<std::string> name = jointHardwareParameter(urdf, joint, robotiq_driver::kProfileParam);
   return name ? robotiq_driver::profileNamed(*name) : Robotiq::profiles::k2F85;
}

std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> findInterface(
   std::vector<hardware_interface::LoanedStateInterface>& interfaces,
   const std::string& joint,
   const rclcpp::Logger& logger)
{
   const auto object_status =
      std::find_if(interfaces.begin(), interfaces.end(), [&](const hardware_interface::LoanedStateInterface& i) {
         return i.get_prefix_name() == joint && i.get_interface_name() == kInterface;
      });
   if(object_status == interfaces.end())
   {
      RCLCPP_ERROR(logger,
                   "%s is set but joint '%s' exports no %s. Mock and topic-based hardware do not report it; "
                   "use the driver against a gripper or its simulation, or unset the parameter.",
                   kUseParameter,
                   joint.c_str(),
                   kInterface);
      return std::nullopt;
   }
   return *object_status;
}

void Verdict::reset(const rclcpp::Time& time,
                    const std::optional<Robotiq::ObjectDetection>& objectDetection,
                    double timeout,
                    const std::optional<uint8_t>& positionRequest)
{
   const Goal previous = goal_;
   timed_from_ = time;
   timeout_ = timeout;
   goal_ = Goal{positionRequest, objectDetection, std::nullopt};
   // Only the goal just decided can repeat: one accepted in between may have
   // moved the fingers already.
   if(previous.outcome && positionRequest && positionRequest == previous.positionRequest
      && objectDetection == previous.objectDetection)
   {
      goal_.outcome = previous.outcome;
   }
}

std::optional<Outcome> Verdict::decide(const rclcpp::Time& time,
                                       const std::optional<Robotiq::ObjectDetection>& objectDetection)
{
   if(goal_.outcome)
   {
      return goal_.outcome;
   }
   if(!goal_.objectDetection)
   {
      // No reading at acceptance: the first one stands in for it.
      goal_.objectDetection = objectDetection;
   }
   else if(objectDetection && objectDetection != goal_.objectDetection)
   {
      goal_.objectDetection = objectDetection;
      if(objectDetection == Robotiq::ObjectDetection::Moving)
      {
         // Motion seen: whatever the gripper settles on next is this goal's
         // verdict, even the value it started from, as when it tightens on the
         // object it already held.
         timed_from_ = time;
      }
      else
      {
         const bool reached = objectDetection == Robotiq::ObjectDetection::AtRequestedPosition;
         goal_.outcome = Outcome{reached, !reached};
         return goal_.outcome;
      }
   }
   if((time - timed_from_).seconds() >= timeout_)
   {
      return Outcome{false, false};
   }
   return std::nullopt;
}
} // namespace robotiq_controllers::object_status_goal
