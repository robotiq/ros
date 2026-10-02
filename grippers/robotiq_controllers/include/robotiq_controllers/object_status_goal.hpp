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

//! Deciding a gripper goal from the joint's object_status interface: the
//! parameters, the interface lookup and the verdict rule the two action
//! controllers share.

#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <Robotiq/gripper/status.hpp>

#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace robotiq_controllers::object_status_goal {

constexpr const char* kUseParameter = "use_object_status";
constexpr const char* kTimeoutParameter = "object_status_timeout";
constexpr double kDefaultTimeout = 10.0;

std::string interfaceName(const std::string& joint);

// The driver's closed position for \p joint, from the hardware block in \p urdf
// that drives it; nothing when no such block carries one the driver would take.
std::optional<double> closedPositionFromUrdf(const std::string& urdf, const std::string& joint);

// The joint's object_status among a controller's loaned state interfaces; logs
// what to do when there is none.
std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> findInterface(
   std::vector<hardware_interface::LoanedStateInterface>& interfaces,
   const std::string& joint,
   const rclcpp::Logger& logger);

struct Outcome
{
   bool reached_goal;
   bool stalled;
};

// One goal's verdict from the readings that follow its acceptance. The gripper
// keeps reporting the previous goal's outcome until it acts on the new target,
// so a settled reading counts only once it differs from the one at acceptance,
// or once motion was seen. A goal still undecided at timeout after its
// acceptance, or after the last motion seen, gets an outcome with neither flag.
//
// The gripper does not act on a position request it already settled on, so a
// goal whose request, in register counts, repeats the one just decided, with
// the reading still the one that decided it, gets that outcome at once.
class Verdict
{
public:
   void reset(const rclcpp::Time& time,
              const std::optional<Robotiq::ObjectDetection>& objectDetection,
              double timeout,
              const std::optional<uint8_t>& positionRequest);
   std::optional<Outcome> decide(const rclcpp::Time& time,
                                 const std::optional<Robotiq::ObjectDetection>& objectDetection);

private:
   struct Goal
   {
      std::optional<uint8_t> positionRequest;
      // The reading a settled one must differ from; once decided, the one that decided it.
      std::optional<Robotiq::ObjectDetection> objectDetection;
      // Set only by the gripper's verdict, never by the timeout.
      std::optional<Outcome> outcome;
   };

   rclcpp::Time timed_from_;
   double timeout_ = 0.0;
   Goal goal_;
};
} // namespace robotiq_controllers::object_status_goal
