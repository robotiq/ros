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

// Quaternion/Euler helpers the driver publishes through: ZYX extraction, the
// Hamilton convention of the Eigen types the filter is built on, and relative
// rotation. Exact analytic expectations, independent of the AHRS algorithm.

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "orientation_test_utils.hpp"

using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using robotiq_tsf::quatToEulerDeg;
using robotiq_tsf::test::eulerDegOf;
using robotiq_tsf::test::eulerNear;
using robotiq_tsf::test::kDegToRad;
using robotiq_tsf::test::kPi;

namespace {

// Euler angles with an analytically-known expected value: the actual value
// goes through a handful of float32 trig ops, so the real error is ~1e-5 deg;
// 1e-3 deg keeps margin while still catching any sign/axis/order mistake.
constexpr float kExactTolDeg = 1e-3f;

// Individual components of a unit quaternion: a few float32 multiply-adds
// away from exact, so near machine epsilon.
constexpr float kQuatTol = 1e-6f;

TEST(QuatHelpers, EulerFromSingleAxisQuaternions)
{
   constexpr float kRollDeg = 30.0f;
   constexpr float kPitchDeg = 45.0f;
   constexpr float kYawDeg = -60.0f;

   Quaternionf q(AngleAxisf(kRollDeg * kDegToRad, Vector3f::UnitX()));
   EXPECT_TRUE(eulerNear(eulerDegOf(q), {kRollDeg, 0.0f, 0.0f}, kExactTolDeg));

   q = AngleAxisf(kPitchDeg * kDegToRad, Vector3f::UnitY());
   EXPECT_TRUE(eulerNear(eulerDegOf(q), {0.0f, kPitchDeg, 0.0f}, kExactTolDeg));

   q = AngleAxisf(kYawDeg * kDegToRad, Vector3f::UnitZ());
   EXPECT_TRUE(eulerNear(eulerDegOf(q), {0.0f, 0.0f, kYawDeg}, kExactTolDeg));
}

TEST(QuatHelpers, EulerFromComposedZyxRotation)
{
   // ZYX Tait-Bryan: q = qz(yaw) * qy(pitch) * qx(roll) must decompose back
   // into the same three angles.
   constexpr float kRollDeg = 10.0f;
   constexpr float kPitchDeg = 20.0f;
   constexpr float kYawDeg = 30.0f;

   const Quaternionf q = AngleAxisf(kYawDeg * kDegToRad, Vector3f::UnitZ())
                       * AngleAxisf(kPitchDeg * kDegToRad, Vector3f::UnitY())
                       * AngleAxisf(kRollDeg * kDegToRad, Vector3f::UnitX());

   EXPECT_TRUE(eulerNear(eulerDegOf(q), {kRollDeg, kPitchDeg, kYawDeg}, kExactTolDeg));
}

TEST(QuatHelpers, MulByConjugateGivesRelativeRotation)
{
   // Zeroing as done for relative orientation: q_rel = conj(q_ref) * q.
   constexpr float kRefRollDeg = 30.0f;
   constexpr float kCurRollDeg = 50.0f;
   constexpr float kRelRollDeg = kCurRollDeg - kRefRollDeg;

   const Quaternionf q_ref(AngleAxisf(kRefRollDeg * kDegToRad, Vector3f::UnitX()));
   const Quaternionf q_cur(AngleAxisf(kCurRollDeg * kDegToRad, Vector3f::UnitX()));
   Quaternionf q_rel = q_ref.conjugate() * q_cur;

   EXPECT_TRUE(eulerNear(eulerDegOf(q_rel), {kRelRollDeg, 0.0f, 0.0f}, kExactTolDeg));

   // Self-relative must be identity.
   q_rel = q_cur.conjugate() * q_cur;
   EXPECT_NEAR(q_rel.w(), 1.0f, kQuatTol);
   EXPECT_NEAR(q_rel.x(), 0.0f, kQuatTol);
   EXPECT_NEAR(q_rel.y(), 0.0f, kQuatTol);
   EXPECT_NEAR(q_rel.z(), 0.0f, kQuatTol);
}

TEST(QuatHelpers, HamiltonConventionGoldenComponents)
{
   // Pins the Hamilton product order and right-handed rotation signs of the
   // Eigen types the filter is built on, with exact component values, so a
   // convention regression cannot slip in silently.
   constexpr float kHalfSqrt2 = 0.70710678f;

   const Quaternionf qx(AngleAxisf(0.5f * kPi, Vector3f::UnitX()));
   EXPECT_NEAR(qx.w(), kHalfSqrt2, kQuatTol);
   EXPECT_NEAR(qx.x(), kHalfSqrt2, kQuatTol);
   EXPECT_NEAR(qx.y(), 0.0f, kQuatTol);
   EXPECT_NEAR(qx.z(), 0.0f, kQuatTol);

   const Quaternionf qy(AngleAxisf(0.5f * kPi, Vector3f::UnitY()));
   EXPECT_NEAR(qy.w(), kHalfSqrt2, kQuatTol);
   EXPECT_NEAR(qy.y(), kHalfSqrt2, kQuatTol);

   const Quaternionf qz(AngleAxisf(0.5f * kPi, Vector3f::UnitZ()));
   EXPECT_NEAR(qz.w(), kHalfSqrt2, kQuatTol);
   EXPECT_NEAR(qz.z(), kHalfSqrt2, kQuatTol);

   // Hamilton product: qx(90) * qy(90) = (0.5, 0.5, 0.5, 0.5) exactly.
   const Quaternionf q = qx * qy;
   EXPECT_NEAR(q.w(), 0.5f, kQuatTol);
   EXPECT_NEAR(q.x(), 0.5f, kQuatTol);
   EXPECT_NEAR(q.y(), 0.5f, kQuatTol);
   EXPECT_NEAR(q.z(), 0.5f, kQuatTol);
}

TEST(QuatHelpers, EulerExtractionClampsAtGimbalLock)
{
   // A slightly super-unit quaternion pushes |sin(pitch)| past 1; the
   // extraction must clamp to exactly +/-90 deg and stay finite (no NaN from
   // asin out of domain).
   const Quaternionf q_up(0.7071f, 0.0f, 0.7080f, 0.0f); // sinp < -1 -> +90
   const Vector3f up = quatToEulerDeg(q_up);
   EXPECT_NEAR(up.y(), 90.0f, kExactTolDeg);
   EXPECT_TRUE(up.allFinite());

   const Quaternionf q_down(0.7071f, 0.0f, -0.7080f, 0.0f); // sinp > 1 -> -90
   const Vector3f down = quatToEulerDeg(q_down);
   EXPECT_NEAR(down.y(), -90.0f, kExactTolDeg);
   EXPECT_TRUE(down.allFinite());
}

} // namespace
