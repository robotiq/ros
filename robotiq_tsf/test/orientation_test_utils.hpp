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

// Shared helpers for the orientation tests: Euler comparison with readable
// failure messages, and tilt error between two attitudes.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

#include "robotiq_tsf/MadgwickAHRS.h"

namespace robotiq_tsf::test {

constexpr float kPi = 3.14159265358979f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

struct EulerAngles
{
   float roll;
   float pitch;
   float yaw;
};

inline EulerAngles eulerDegOf(const Eigen::Quaternionf& q)
{
   EulerAngles e;
   quatToEulerDeg(q, e.roll, e.pitch, e.yaw);
   return e;
}

inline float maxAbsDeg(const EulerAngles& e)
{
   return std::max(std::fabs(e.roll), std::max(std::fabs(e.pitch), std::fabs(e.yaw)));
}

inline ::testing::AssertionResult eulerNear(const EulerAngles& actual, const EulerAngles& expected, float tol_deg)
{
   if(std::fabs(actual.roll - expected.roll) <= tol_deg && std::fabs(actual.pitch - expected.pitch) <= tol_deg
      && std::fabs(actual.yaw - expected.yaw) <= tol_deg)
   {
      return ::testing::AssertionSuccess();
   }
   return ::testing::AssertionFailure() << "(roll, pitch, yaw) = (" << actual.roll << ", " << actual.pitch << ", "
                                        << actual.yaw << ") deg, expected (" << expected.roll << ", " << expected.pitch
                                        << ", " << expected.yaw << ") deg +/- " << tol_deg;
}

// Angle between the gravity directions two attitudes predict in the body
// frame: the roll/pitch error, blind to yaw, which no accelerometer observes.
inline float tiltErrorDeg(const Eigen::Quaternionf& estimate, const Eigen::Quaternionf& truth)
{
   const Eigen::Vector3f g_est = estimate.conjugate() * Eigen::Vector3f::UnitZ();
   const Eigen::Vector3f g_true = truth.conjugate() * Eigen::Vector3f::UnitZ();
   return std::acos(std::clamp(g_est.dot(g_true), -1.0f, 1.0f)) * kRadToDeg;
}

} // namespace robotiq_tsf::test
