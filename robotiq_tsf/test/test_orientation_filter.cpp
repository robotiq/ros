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

// Behaviour every orientation filter the driver may run must keep: frame and
// Euler conventions, start-up seeding, gyro integration, accelerometer gating,
// and the tilt-correction envelope (rate, bias rejection, noise, recovery).
//
// Expectations are analytic or physical, never one implementation's exact
// numbers, so the suite outlives an algorithm swap. Envelope bounds are the
// filter in service when they were written, plus margin.
//
// All math is done in float, matching the filters under test: the IMU samples
// are 16-bit (±2 g / ±250 deg/s), so sensor noise and bias sit far above
// float32 rounding.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <random>

#include "orientation_test_utils.hpp"
#include "robotiq_tsf/MadgwickAHRS.h"
#include "robotiq_tsf/orientation_filter.hpp"

using Eigen::AngleAxisf;
using Eigen::Quaternionf;
using Eigen::Vector3f;
using robotiq_tsf::test::eulerNear;
using robotiq_tsf::test::kDegToRad;
using robotiq_tsf::test::kRadToDeg;
using robotiq_tsf::test::maxAbsDeg;
using robotiq_tsf::test::tiltErrorDeg;
using EulerAngles = robotiq_tsf::test::EulerAngles;

namespace {

// Analytically-known Euler angles through a handful of float32 trig ops.
constexpr float kExactTolDeg = 1e-3f;
constexpr float kQuatTol = 1e-6f;

// Integration step close to the sensor's ~700 Hz packet rate.
constexpr float kDt = 0.001f;
constexpr int kStepsPerSecond = 1000;

// The driver's default madgwick.beta.
constexpr float kDefaultBeta = 0.041f;

// |a| = 1.4 g: outside the default [0.85, 1.15] g gate, so accel is ignored.
constexpr float kGatedLateralG = 0.98f;

template <typename Filter>
EulerAngles eulerDegOf(const Filter& f)
{
   EulerAngles e;
   f.getEulerDeg(e.roll, e.pitch, e.yaw);
   return e;
}

// Seed `f` believing it is rolled by `roll_deg` while the sensor actually lies
// flat, so the following stationary updates measure how the filter corrects a
// tilt error.
template <typename Filter>
void seedRollError(Filter& f, float roll_deg)
{
   f.initFromAccel(0.0f, std::sin(roll_deg * kDegToRad), std::cos(roll_deg * kDegToRad));
}

template <typename Filter>
void updateStillFlat(Filter& f, int steps)
{
   for(int i = 0; i < steps; ++i)
   {
      f.updateIMU(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, kDt);
   }
}

template <typename Filter>
class OrientationFilterTest : public ::testing::Test
{
};

using FilterTypes = ::testing::Types<MadgwickFilter, robotiq_tsf::OrientationFilter>;
TYPED_TEST_SUITE(OrientationFilterTest, FilterTypes);

TYPED_TEST(OrientationFilterTest, InitFromAccelMatchesTilt)
{
   constexpr float kRollDeg = 30.0f;
   constexpr float kPitchDeg = 20.0f;
   constexpr float g = 9.81f; // m/s^2

   TypeParam f;

   // Flat: gravity along body +Z.
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   EXPECT_TRUE(eulerNear(eulerDegOf(f), {0.0f, 0.0f, 0.0f}, kExactTolDeg));

   // Pure roll: gravity_body = (0, sin r, cos r). Units must not matter,
   // so feed m/s^2 rather than g.
   f.initFromAccel(0.0f, g * std::sin(kRollDeg * kDegToRad), g * std::cos(kRollDeg * kDegToRad));
   EXPECT_TRUE(eulerNear(eulerDegOf(f), {kRollDeg, 0.0f, 0.0f}, kExactTolDeg));

   // Pure pitch: gravity_body = (-sin p, 0, cos p).
   f.initFromAccel(-std::sin(kPitchDeg * kDegToRad), 0.0f, std::cos(kPitchDeg * kDegToRad));
   EXPECT_TRUE(eulerNear(eulerDegOf(f), {0.0f, kPitchDeg, 0.0f}, kExactTolDeg));
}

TYPED_TEST(OrientationFilterTest, InitFromAccelCombinedRollPitch)
{
   // Gravity in body frame for roll r, pitch p (ZYX, yaw-free):
   // g_body = (-sin p, sin r * cos p, cos r * cos p).
   constexpr float kRollDeg = 30.0f;
   constexpr float kPitchDeg = 20.0f;
   const float sr = std::sin(kRollDeg * kDegToRad);
   const float cr = std::cos(kRollDeg * kDegToRad);
   const float sp = std::sin(kPitchDeg * kDegToRad);
   const float cp = std::cos(kPitchDeg * kDegToRad);

   TypeParam f;
   f.initFromAccel(-sp, sr * cp, cr * cp);
   EXPECT_TRUE(eulerNear(eulerDegOf(f), {kRollDeg, kPitchDeg, 0.0f}, kExactTolDeg));
}

TYPED_TEST(OrientationFilterTest, InitFromZeroAccelResetsToIdentity)
{
   TypeParam f;
   f.initFromAccel(0.0f, 1.0f, 0.0f); // some non-identity state first
   f.initFromAccel(0.0f, 0.0f, 0.0f);
   float q0, q1, q2, q3;
   f.getQuaternion(q0, q1, q2, q3);
   EXPECT_NEAR(q0, 1.0f, kQuatTol);
   EXPECT_NEAR(q1, 0.0f, kQuatTol);
   EXPECT_NEAR(q2, 0.0f, kQuatTol);
   EXPECT_NEAR(q3, 0.0f, kQuatTol);
}

TYPED_TEST(OrientationFilterTest, NonPositiveDtIsIgnored)
{
   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   float before[4], after[4];
   f.getQuaternion(before[0], before[1], before[2], before[3]);
   f.updateIMU(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 1.0f, 0.0f);
   f.updateIMU(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 1.0f, -0.01f);
   f.getQuaternion(after[0], after[1], after[2], after[3]);
   for(int i = 0; i < 4; ++i)
   {
      EXPECT_EQ(before[i], after[i]);
   }
}

TYPED_TEST(OrientationFilterTest, GyroOnlyIntegrationMatchesAnalyticRotation)
{
   // Accel gated: the update is pure gyro integration, which has one right
   // answer whatever the correction algorithm.
   constexpr float kYawRateDegS = 45.0f;
   constexpr int kSteps = 2 * kStepsPerSecond;
   constexpr float kExpectedYawDeg = kYawRateDegS * kDt * kSteps;
   // First-order integration over 2000 steps of 0.045 deg.
   constexpr float kIntegrationTolDeg = 0.05f;

   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   for(int i = 0; i < kSteps; ++i)
   {
      f.updateIMU(0.0f, 0.0f, kYawRateDegS * kDegToRad, kGatedLateralG, 0.0f, 1.0f, kDt);
   }
   EXPECT_TRUE(eulerNear(eulerDegOf(f), {0.0f, 0.0f, kExpectedYawDeg}, kIntegrationTolDeg));
}

TYPED_TEST(OrientationFilterTest, TracksRotationWithMeasuredDt)
{
   // 90 deg/s roll for 1 s, sampled at 200 Hz. Accel stays consistent with
   // the true attitude (pure tilt, |a| = 1 g), gyro is exact. The legacy
   // filter integrated every sample as 1/1000 s regardless of the real rate,
   // so it tracked this rotation ~5x too slow — the "drift" in PR #5.
   constexpr float kRateDegS = 90.0f;
   constexpr float kStep = 0.005f;
   constexpr int kSteps = 200;
   constexpr float kExpectedRollDeg = kRateDegS * kStep * kSteps;
   constexpr float kTrackingTolDeg = 3.0f;

   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   for(int i = 0; i < kSteps; ++i)
   {
      const float rollTrue = kRateDegS * kStep * static_cast<float>(i) * kDegToRad;
      f.updateIMU(kRateDegS * kDegToRad, 0.0f, 0.0f, 0.0f, std::sin(rollTrue), std::cos(rollTrue), kStep);
   }
   EXPECT_TRUE(eulerNear(eulerDegOf(f), {kExpectedRollDeg, 0.0f, 0.0f}, kTrackingTolDeg));
}

TYPED_TEST(OrientationFilterTest, AccelGateRejectsMotionBursts)
{
   // Stationary sensor, zero rotation, but a sustained linear-acceleration
   // burst (|a| ~ 1.35 g). The legacy filter treated the burst as a tilted
   // gravity vector and walked the attitude away — the "noisy in transition"
   // in PR #5. Accel feedback outside [0.85, 1.15] g must be gated, so the
   // attitude must not move at all.
   constexpr int kBurstSteps = 5 * kStepsPerSecond;
   constexpr float kBurstLateralG = 0.9f;
   constexpr float kGateLeakTolDeg = 0.1f;

   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);

   float peakDeg = 0.0f;
   for(int i = 0; i < kBurstSteps; ++i)
   {
      f.updateIMU(0.0f, 0.0f, 0.0f, kBurstLateralG, 0.0f, 1.0f, kDt);
      peakDeg = std::max(peakDeg, maxAbsDeg(eulerDegOf(f)));
   }

   EXPECT_LT(peakDeg, kGateLeakTolDeg);
}

TYPED_TEST(OrientationFilterTest, BetaIsTheTiltCorrectionRate)
{
   // madgwick.beta is user-facing: a tilt error of a few degrees closes at
   // 2 * beta rad/s, independently of its size. Users tuned it on that
   // meaning, so it must keep it. (Madgwick's gradient step falls below that
   // rate as the error grows — 94% at 20 deg, 45% at 90 deg — so the check
   // stays at the errors seen in service.)
   constexpr float kInitialErrorDeg = 10.0f;
   constexpr int kSteps = kStepsPerSecond / 2;
   constexpr float kRateTolDeg = 0.1f;

   for(const float beta : {kDefaultBeta, 2.0f * kDefaultBeta})
   {
      TypeParam f;
      f.setBeta(beta);
      seedRollError(f, kInitialErrorDeg);
      updateStillFlat(f, kSteps);

      const float expectedErrorDeg = kInitialErrorDeg - 2.0f * beta * kRadToDeg * kSteps * kDt;
      EXPECT_NEAR(tiltErrorDeg(f.quaternion(), Quaternionf::Identity()), expectedErrorDeg, kRateTolDeg)
         << "beta = " << beta;
   }
}

TYPED_TEST(OrientationFilterTest, SettlesFromTiltErrorWithoutOvershoot)
{
   // 20 deg closes at 4.7 deg/s in ~4.1 s.
   constexpr float kInitialErrorDeg = 20.0f;
   constexpr float kSettledDeg = 1.0f;
   constexpr int kSettleDeadlineSteps = 5 * kStepsPerSecond;
   constexpr int kObservedSteps = 10 * kStepsPerSecond;

   TypeParam f;
   seedRollError(f, kInitialErrorDeg);

   int settledAt = -1;
   float peakAfterSettling = 0.0f;
   for(int i = 0; i < kObservedSteps; ++i)
   {
      updateStillFlat(f, 1);
      const float error = tiltErrorDeg(f.quaternion(), Quaternionf::Identity());
      if(settledAt < 0 && error < kSettledDeg)
      {
         settledAt = i;
      }
      if(settledAt >= 0)
      {
         peakAfterSettling = std::max(peakAfterSettling, error);
      }
   }
   ASSERT_GE(settledAt, 0) << "never settled below " << kSettledDeg << " deg";
   EXPECT_LT(settledAt, kSettleDeadlineSteps);
   EXPECT_LT(peakAfterSettling, kSettledDeg);
}

TYPED_TEST(OrientationFilterTest, RecoversFromQuarterTurnTiltError)
{
   constexpr float kInitialErrorDeg = 90.0f;
   constexpr float kSettledDeg = 1.0f;
   constexpr int kRecoveryDeadlineSteps = 30 * kStepsPerSecond;

   TypeParam f;
   seedRollError(f, kInitialErrorDeg);
   updateStillFlat(f, kRecoveryDeadlineSteps);
   EXPECT_LT(tiltErrorDeg(f.quaternion(), Quaternionf::Identity()), kSettledDeg);
}

TYPED_TEST(OrientationFilterTest, ConstantGyroBiasLeavesSmallTiltError)
{
   // Residual bias after calibration and online trim is a fraction of a
   // deg/s; 1 deg/s is a pessimistic case. The tilt error it leaves once the
   // correction balances it must stay well under the 1 deg users resolve.
   constexpr float kBiasDegS = 1.0f;
   constexpr int kSteps = 120 * kStepsPerSecond;
   constexpr float kMaxBiasTiltDeg = 0.5f;

   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   for(int i = 0; i < kSteps; ++i)
   {
      f.updateIMU(kBiasDegS * kDegToRad, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, kDt);
   }
   EXPECT_LT(tiltErrorDeg(f.quaternion(), Quaternionf::Identity()), kMaxBiasTiltDeg);
}

TYPED_TEST(OrientationFilterTest, StationaryNoiseStaysBelowJitterBound)
{
   // Seeded white noise above the sensor's measured floor: 0.004 g on accel,
   // 0.1 deg/s on gyro.
   constexpr float kAccelNoiseG = 0.004f;
   constexpr float kGyroNoiseDegS = 0.1f;
   constexpr int kSettleSteps = 10 * kStepsPerSecond;
   constexpr int kMeasuredSteps = 30 * kStepsPerSecond;
   constexpr float kMaxRmsDeg = 0.05f;
   constexpr float kMaxPeakDeg = 0.15f;

   std::mt19937 rng(1);
   std::normal_distribution<float> accelNoise(0.0f, kAccelNoiseG);
   std::normal_distribution<float> gyroNoise(0.0f, kGyroNoiseDegS * kDegToRad);

   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   double sumSq = 0.0;
   float peak = 0.0f;
   for(int i = 0; i < kSettleSteps + kMeasuredSteps; ++i)
   {
      f.updateIMU(gyroNoise(rng),
                  gyroNoise(rng),
                  gyroNoise(rng),
                  accelNoise(rng),
                  accelNoise(rng),
                  1.0f + accelNoise(rng),
                  kDt);
      if(i >= kSettleSteps)
      {
         const float error = tiltErrorDeg(f.quaternion(), Quaternionf::Identity());
         sumSq += static_cast<double>(error) * error;
         peak = std::max(peak, error);
      }
   }
   EXPECT_LT(std::sqrt(sumSq / kMeasuredSteps), kMaxRmsDeg);
   EXPECT_LT(peak, kMaxPeakDeg);
}

TYPED_TEST(OrientationFilterTest, TracksHandheldMotionWithinTolerance)
{
   // A minute of hand-held-like motion with bursts, noise and bias: tilt
   // error against ground truth stays below what a user would notice.
   constexpr int kSteps = 60 * kStepsPerSecond;
   constexpr int kWarmupSteps = 5 * kStepsPerSecond;
   constexpr float kMaxTiltErrorDeg = 1.5f;
   constexpr float kUnitNormTol = 1e-5f;

   TypeParam f;
   f.initFromAccel(0.0f, 0.0f, 1.0f);
   float peakError = 0.0f;
   const auto samples = robotiq_tsf::test::handheldMotion(kSteps, kDt);
   for(int i = 0; i < kSteps; ++i)
   {
      const auto& s = samples[i];
      f.updateIMU(s.gyro.x(), s.gyro.y(), s.gyro.z(), s.accel.x(), s.accel.y(), s.accel.z(), kDt);
      if(i >= kWarmupSteps)
      {
         peakError = std::max(peakError, tiltErrorDeg(f.quaternion(), s.truth));
      }
   }
   EXPECT_LT(peakError, kMaxTiltErrorDeg);
   EXPECT_NEAR(f.quaternion().norm(), 1.0f, kUnitNormTol);
}

TEST(OrientationFilter, RecoversFromUpsideDownSeed)
{
   // Seeded nearly upside down, e.g. calibrated while held inverted and then
   // set down. MadgwickFilter's gradient vanishes near 180 deg and never
   // recovers; this filter must, within the 30 s allowed a quarter turn.
   constexpr float kInitialErrorDeg = 170.0f;
   constexpr float kSettledDeg = 1.0f;
   constexpr int kRecoveryDeadlineSteps = 30 * kStepsPerSecond;

   robotiq_tsf::OrientationFilter f;
   seedRollError(f, kInitialErrorDeg);
   updateStillFlat(f, kRecoveryDeadlineSteps);
   EXPECT_LT(tiltErrorDeg(f.quaternion(), Quaternionf::Identity()), kSettledDeg);
}

} // namespace
