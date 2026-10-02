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

// Numeric regression specific to MadgwickFilter. Behaviour any orientation
// filter must keep lives in test_orientation_filter.cpp.

#include <gtest/gtest.h>

#include <cmath>

#include "orientation_test_utils.hpp"
#include "robotiq_tsf/MadgwickAHRS.h"

using robotiq_tsf::test::kDegToRad;

namespace {

TEST(MadgwickFilter, GoldenTrajectoryCheckpoints)
{
   // End-to-end numeric regression: a fixed synthetic IMU stream must keep
   // reproducing the quaternion checkpoints recorded from the pre-Eigen
   // implementation. Guards the library migration against any behavior
   // change beyond float-op reordering: 1e-5 (~80 float32 ulps at unit
   // scale) absorbs reordering drift accumulated over the 350 updates while
   // staying orders of magnitude below a real algorithmic change.
   constexpr float kGoldenTol = 1e-5f;
   constexpr float kDt = 0.005f;
   struct Checkpoint
   {
      const char* tag;
      float q[4];
   };

   MadgwickFilter f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);

   auto expectCheckpoint = [&f](const Checkpoint& c) {
      float q[4];
      f.getQuaternion(q[0], q[1], q[2], q[3]);
      for(int i = 0; i < 4; ++i)
      {
         EXPECT_NEAR(q[i], c.q[i], kGoldenTol) << "checkpoint " << c.tag << " component " << i;
      }
   };

   // Segment A: 100 steps stationary (accel gate open, gradient active).
   for(int i = 0; i < 100; ++i)
   {
      f.updateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, kDt);
   }
   expectCheckpoint({"A", {1.00000000f, 0.00000000f, 0.00000000f, 0.00000000f}});

   // Segment B: 150 steps rolling at 60 deg/s with attitude-consistent accel.
   for(int i = 0; i < 150; ++i)
   {
      const float roll = 60.0f * kDt * static_cast<float>(i) * kDegToRad;
      f.updateIMU(60.0f * kDegToRad, 0.0f, 0.0f, 0.0f, std::sin(roll), std::cos(roll), kDt);
   }
   expectCheckpoint({"B", {0.92394269f, 0.38253102f, 0.00000000f, 0.00000000f}});

   // Segment C: 100 steps yawing at 45 deg/s while accel is gated
   // (|a| = 1.4 g): pure gyro integration path.
   for(int i = 0; i < 100; ++i)
   {
      f.updateIMU(0.0f, 0.0f, 45.0f * kDegToRad, 0.98f, 0.0f, 1.0f, kDt);
   }
   expectCheckpoint({"C", {0.90618920f, 0.37518126f, -0.07462808f, 0.18025191f}});
}

} // namespace
