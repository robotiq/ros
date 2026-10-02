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

// OrientationFilter against the MadgwickFilter it replaces, fed identical
// input: the evidence that the swap leaves the published orientation where
// users had it. Deleted with MadgwickFilter.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

#include "orientation_test_utils.hpp"
#include "robotiq_tsf/MadgwickAHRS.h"
#include "robotiq_tsf/orientation_filter.hpp"

using Eigen::Quaternionf;
using robotiq_tsf::OrientationFilter;
using robotiq_tsf::test::kDegToRad;
using robotiq_tsf::test::kRadToDeg;
using robotiq_tsf::test::tiltErrorDeg;

namespace {

constexpr float kDt = 0.001f;
constexpr int kStepsPerSecond = 1000;

// Full rotation between the two estimates, yaw included.
float rotationDeg(const Quaternionf& a, const Quaternionf& b)
{
   return Eigen::AngleAxisf(a.conjugate() * b).angle() * kRadToDeg;
}

struct PeakDivergence
{
   float tiltDeg = 0.0f;
   float rotationDeg = 0.0f;
};

void track(PeakDivergence& peak, const MadgwickFilter& m, const OrientationFilter& o)
{
   peak.tiltDeg = std::max(peak.tiltDeg, tiltErrorDeg(o.quaternion(), m.quaternion()));
   peak.rotationDeg = std::max(peak.rotationDeg, rotationDeg(o.quaternion(), m.quaternion()));
}

TEST(FilterParity, HandheldMotionMatchesMadgwick)
{
   constexpr int kSteps = 60 * kStepsPerSecond;
   constexpr float kMaxTiltDivergenceDeg = 0.3f;
   constexpr float kMaxRotationDivergenceDeg = 0.5f;

   MadgwickFilter m;
   OrientationFilter o;
   m.initFromAccel(0.0f, 0.0f, 1.0f);
   o.initFromAccel(0.0f, 0.0f, 1.0f);
   PeakDivergence peak;
   for(const auto& s : robotiq_tsf::test::handheldMotion(kSteps, kDt))
   {
      m.updateIMU(s.gyro.x(), s.gyro.y(), s.gyro.z(), s.accel.x(), s.accel.y(), s.accel.z(), kDt);
      o.updateIMU(s.gyro.x(), s.gyro.y(), s.gyro.z(), s.accel.x(), s.accel.y(), s.accel.z(), kDt);
      track(peak, m, o);
   }
   EXPECT_LT(peak.tiltDeg, kMaxTiltDivergenceDeg);
   EXPECT_LT(peak.rotationDeg, kMaxRotationDivergenceDeg);
}

TEST(FilterParity, TiltCorrectionMatchesMadgwick)
{
   // Below ~20 deg the two correction rates agree within 6%, so the
   // trajectories of a settling error stay close the whole way down.
   constexpr float kInitialErrorDeg = 20.0f;
   constexpr int kSteps = 8 * kStepsPerSecond;
   constexpr float kMaxDivergenceDeg = 0.5f;

   MadgwickFilter m;
   OrientationFilter o;
   const float ay = std::sin(kInitialErrorDeg * kDegToRad);
   const float az = std::cos(kInitialErrorDeg * kDegToRad);
   m.initFromAccel(0.0f, ay, az);
   o.initFromAccel(0.0f, ay, az);
   PeakDivergence peak;
   for(int i = 0; i < kSteps; ++i)
   {
      m.updateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, kDt);
      o.updateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, kDt);
      track(peak, m, o);
   }
   EXPECT_LT(peak.rotationDeg, kMaxDivergenceDeg);
}

TEST(FilterParity, NonDefaultTuningMatchesMadgwick)
{
   // A user's beta and accel-gate overrides must mean the same to both.
   constexpr float kBeta = 0.1f;
   constexpr float kGateLo = 0.9f;
   constexpr float kGateHi = 1.1f;
   constexpr int kSteps = 30 * kStepsPerSecond;
   constexpr float kMaxTiltDivergenceDeg = 0.3f;

   MadgwickFilter m(kBeta);
   OrientationFilter o(kBeta);
   m.setAccelGate(kGateLo, kGateHi);
   o.setAccelGate(kGateLo, kGateHi);
   m.initFromAccel(0.0f, 0.0f, 1.0f);
   o.initFromAccel(0.0f, 0.0f, 1.0f);
   PeakDivergence peak;
   for(const auto& s : robotiq_tsf::test::handheldMotion(kSteps, kDt, 3))
   {
      m.updateIMU(s.gyro.x(), s.gyro.y(), s.gyro.z(), s.accel.x(), s.accel.y(), s.accel.z(), kDt);
      o.updateIMU(s.gyro.x(), s.gyro.y(), s.gyro.z(), s.accel.x(), s.accel.y(), s.accel.z(), kDt);
      track(peak, m, o);
   }
   EXPECT_LT(peak.tiltDeg, kMaxTiltDivergenceDeg);
}

} // namespace
