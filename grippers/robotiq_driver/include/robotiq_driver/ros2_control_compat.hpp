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

// ros2_control API differences across the ROS 2 distros this package supports
// (Humble, Jazzy, Lyrical).
//
// SystemInterface::on_init() takes a HardwareInfo on Humble (ros2_control 2.x)
// and a HardwareComponentInterfaceParams from Jazzy's 4.x line on, where the
// HardwareInfo overload is deprecated. Detection is by header presence rather
// than a HARDWARE_INTERFACE_VERSION_GTE threshold: the struct arrived with its
// own header, and Jazzy's 4.x line keeps moving, so there is no stable version
// number to compare against.
//
// Humble EOL: delete this header and its CMake entry; OnInitParams collapses to
// HardwareComponentInterfaceParams everywhere it is named.

#if __has_include(<hardware_interface/types/hardware_component_interface_params.hpp>)
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#else
#include <hardware_interface/hardware_info.hpp>
#endif

namespace robotiq_driver {
#if __has_include(<hardware_interface/types/hardware_component_interface_params.hpp>)
/// Argument type of SystemInterface::on_init() on this distro.
using OnInitParams = hardware_interface::HardwareComponentInterfaceParams;
#else
using OnInitParams = hardware_interface::HardwareInfo;
#endif

} // namespace robotiq_driver
