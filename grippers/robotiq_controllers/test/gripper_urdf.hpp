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

//! A URDF whose ros2_control block carries the driver's hardware parameters,
//! for tests that need a robot description.

#pragma once

#include <string>

namespace robotiq_controllers::test {

inline std::string gripperUrdf(const std::string& joint, const std::string& closed_position_param)
{
   return R"(<?xml version="1.0"?>
<robot name="gripper">
  <link name="base"/>
  <link name="finger"/>
  <joint name=")"
        + joint + R"(" type="revolute">
    <parent link="base"/>
    <child link="finger"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="0.8" effort="1" velocity="1"/>
  </joint>
  <ros2_control name="gripper" type="system">
    <hardware>
      <plugin>robotiq_driver/RobotiqGripperHardwareInterface</plugin>
      )" + closed_position_param
        +
          R"(
    </hardware>
    <joint name=")"
        + joint + R"(">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>)";
}

inline std::string gripperUrdf(const std::string& joint, double closed_position)
{
   return gripperUrdf(joint,
                      R"(<param name="gripper_closed_position">)" + std::to_string(closed_position) + "</param>");
}
} // namespace robotiq_controllers::test
