// All math is done in float, matching the filter under test: the Madgwick
// reference implementation is float32 (embedded origin), and the IMU samples
// are 16-bit (±2 g / ±250 deg/s), so sensor noise and bias sit far above
// float32 rounding — double would add no accuracy. The quaternion is
// renormalized every update, so float error does not accumulate either.

#include "robotiq_tsf/MadgwickAHRS.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

constexpr float kPi = 3.14159265358979f;
constexpr float kDegToRad = kPi / 180.0f;

// Euler angles with an analytically-known expected value: the actual value
// goes through a handful of float32 trig ops, so the real error is ~1e-5 deg;
// 1e-3 deg keeps margin while still catching any sign/axis/order mistake.
constexpr float kExactTolDeg = 1e-3f;

// Individual components of a unit quaternion: a few float32 multiply-adds
// away from exact, so near machine epsilon.
constexpr float kQuatTol = 1e-6f;

// Filter attitude after integrating 1 s of motion: dominated by algorithmic
// error (discrete integration + gradient-descent correction), not rounding.
constexpr float kTrackingTolDeg = 3.0f;

// Attitude change while accel is gated and gyro is zero: the update must be
// a no-op, so anything beyond float dust is a gating failure.
constexpr float kGateLeakTolDeg = 0.1f;

struct EulerAngles
{
  float roll;
  float pitch;
  float yaw;
};

EulerAngles eulerDegOf(const float q[4])
{
  EulerAngles e;
  quatToEulerDeg(q, e.roll, e.pitch, e.yaw);
  return e;
}

EulerAngles eulerDegOf(const MadgwickFilter& f)
{
  EulerAngles e;
  f.getEulerDeg(e.roll, e.pitch, e.yaw);
  return e;
}

float maxAbsDeg(const EulerAngles& e)
{
  return std::max(std::fabs(e.roll), std::max(std::fabs(e.pitch), std::fabs(e.yaw)));
}

::testing::AssertionResult eulerNear(const EulerAngles& actual, const EulerAngles& expected, float tol_deg)
{
  if (std::fabs(actual.roll - expected.roll) <= tol_deg && std::fabs(actual.pitch - expected.pitch) <= tol_deg &&
      std::fabs(actual.yaw - expected.yaw) <= tol_deg)
  {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << "(roll, pitch, yaw) = (" << actual.roll << ", " << actual.pitch << ", "
                                       << actual.yaw << ") deg, expected (" << expected.roll << ", " << expected.pitch
                                       << ", " << expected.yaw << ") deg +/- " << tol_deg;
}

TEST(QuatHelpers, EulerFromSingleAxisQuaternions)
{
  constexpr float kRollDeg = 30.0f;
  constexpr float kPitchDeg = 45.0f;
  constexpr float kYawDeg = -60.0f;

  float q[4];

  quatFromAxisX(kRollDeg * kDegToRad, q);
  EXPECT_TRUE(eulerNear(eulerDegOf(q), { kRollDeg, 0.0f, 0.0f }, kExactTolDeg));

  quatFromAxisY(kPitchDeg * kDegToRad, q);
  EXPECT_TRUE(eulerNear(eulerDegOf(q), { 0.0f, kPitchDeg, 0.0f }, kExactTolDeg));

  quatFromAxisZ(kYawDeg * kDegToRad, q);
  EXPECT_TRUE(eulerNear(eulerDegOf(q), { 0.0f, 0.0f, kYawDeg }, kExactTolDeg));
}

TEST(QuatHelpers, EulerFromComposedZyxRotation)
{
  // ZYX Tait-Bryan: q = qz(yaw) * qy(pitch) * qx(roll) must decompose back
  // into the same three angles.
  constexpr float kRollDeg = 10.0f;
  constexpr float kPitchDeg = 20.0f;
  constexpr float kYawDeg = 30.0f;

  float qx[4], qy[4], qz[4], tmp[4], q[4];
  quatFromAxisX(kRollDeg * kDegToRad, qx);
  quatFromAxisY(kPitchDeg * kDegToRad, qy);
  quatFromAxisZ(kYawDeg * kDegToRad, qz);
  quatMul(qz, qy, tmp);
  quatMul(tmp, qx, q);

  EXPECT_TRUE(eulerNear(eulerDegOf(q), { kRollDeg, kPitchDeg, kYawDeg }, kExactTolDeg));
}

TEST(QuatHelpers, MulByConjugateGivesRelativeRotation)
{
  // Zeroing as done for relative orientation: q_rel = conj(q_ref) * q.
  constexpr float kRefRollDeg = 30.0f;
  constexpr float kCurRollDeg = 50.0f;
  constexpr float kRelRollDeg = kCurRollDeg - kRefRollDeg;

  float q_ref[4], q_cur[4], q_ref_conj[4], q_rel[4];
  quatFromAxisX(kRefRollDeg * kDegToRad, q_ref);
  quatFromAxisX(kCurRollDeg * kDegToRad, q_cur);
  quatConj(q_ref, q_ref_conj);
  quatMul(q_ref_conj, q_cur, q_rel);

  EXPECT_TRUE(eulerNear(eulerDegOf(q_rel), { kRelRollDeg, 0.0f, 0.0f }, kExactTolDeg));

  // Self-relative must be identity.
  quatConj(q_cur, q_ref_conj);
  quatMul(q_ref_conj, q_cur, q_rel);
  EXPECT_NEAR(q_rel[0], 1.0f, kQuatTol);
  EXPECT_NEAR(q_rel[1], 0.0f, kQuatTol);
  EXPECT_NEAR(q_rel[2], 0.0f, kQuatTol);
  EXPECT_NEAR(q_rel[3], 0.0f, kQuatTol);
}

TEST(QuatHelpers, NormalizeScalesToUnitAndHandlesZero)
{
  float q[4] = { 2.0f, 0.0f, 0.0f, 0.0f };
  quatNormalize(q);
  EXPECT_NEAR(q[0], 1.0f, kQuatTol);

  float z[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  quatNormalize(z);  // must not produce NaN/inf
  EXPECT_TRUE(std::isfinite(z[0]) && std::isfinite(z[1]) && std::isfinite(z[2]) && std::isfinite(z[3]));
}

TEST(MadgwickFilter, InitFromAccelMatchesTilt)
{
  constexpr float kRollDeg = 30.0f;
  constexpr float kPitchDeg = 20.0f;
  constexpr float g = 9.81f;  // m/s^2

  MadgwickFilter f;

  // Flat: gravity along body +Z.
  f.initFromAccel(0.0f, 0.0f, 1.0f);
  EXPECT_TRUE(eulerNear(eulerDegOf(f), { 0.0f, 0.0f, 0.0f }, kExactTolDeg));

  // Pure roll: gravity_body = (0, sin r, cos r). Units must not matter,
  // so feed m/s^2 rather than g.
  f.initFromAccel(0.0f, g * std::sin(kRollDeg * kDegToRad), g * std::cos(kRollDeg * kDegToRad));
  EXPECT_TRUE(eulerNear(eulerDegOf(f), { kRollDeg, 0.0f, 0.0f }, kExactTolDeg));

  // Pure pitch: gravity_body = (-sin p, 0, cos p).
  f.initFromAccel(-std::sin(kPitchDeg * kDegToRad), 0.0f, std::cos(kPitchDeg * kDegToRad));
  EXPECT_TRUE(eulerNear(eulerDegOf(f), { 0.0f, kPitchDeg, 0.0f }, kExactTolDeg));
}

TEST(MadgwickFilter, InitFromZeroAccelResetsToIdentity)
{
  MadgwickFilter f;
  f.initFromAccel(0.0f, 1.0f, 0.0f);  // some non-identity state first
  f.initFromAccel(0.0f, 0.0f, 0.0f);
  float q0, q1, q2, q3;
  f.getQuaternion(q0, q1, q2, q3);
  EXPECT_NEAR(q0, 1.0f, kQuatTol);
  EXPECT_NEAR(q1, 0.0f, kQuatTol);
  EXPECT_NEAR(q2, 0.0f, kQuatTol);
  EXPECT_NEAR(q3, 0.0f, kQuatTol);
}

TEST(MadgwickFilter, NonPositiveDtIsIgnored)
{
  MadgwickFilter f;
  f.initFromAccel(0.0f, 0.0f, 1.0f);
  float before[4], after[4];
  f.getQuaternion(before[0], before[1], before[2], before[3]);
  f.updateIMU(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 1.0f, 0.0f);
  f.updateIMU(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 1.0f, -0.01f);
  f.getQuaternion(after[0], after[1], after[2], after[3]);
  for (int i = 0; i < 4; ++i)
    EXPECT_EQ(before[i], after[i]);
}

TEST(MadgwickFilter, TracksRotationWithMeasuredDt)
{
  // 90 deg/s roll for 1 s, sampled at 200 Hz. Accel stays consistent with
  // the true attitude (pure tilt, |a| = 1 g), gyro is exact. The legacy
  // filter integrated every sample as 1/1000 s regardless of the real rate,
  // so it tracked this rotation ~5x too slow — the "drift" in PR #5.
  constexpr float kRateDegS = 90.0f;
  constexpr float kDt = 0.005f;
  constexpr int kSteps = 200;
  constexpr float kExpectedRollDeg = kRateDegS * kDt * kSteps;

  MadgwickFilter f;
  f.initFromAccel(0.0f, 0.0f, 1.0f);
  for (int i = 0; i < kSteps; ++i)
  {
    const float rollTrue = kRateDegS * kDt * static_cast<float>(i) * kDegToRad;
    f.updateIMU(kRateDegS * kDegToRad, 0.0f, 0.0f, 0.0f, std::sin(rollTrue), std::cos(rollTrue), kDt);
  }
  EXPECT_TRUE(eulerNear(eulerDegOf(f), { kExpectedRollDeg, 0.0f, 0.0f }, kTrackingTolDeg));
}

TEST(MadgwickFilter, AccelGateRejectsMotionBursts)
{
  // Stationary sensor, zero rotation, but a sustained linear-acceleration
  // burst (|a| ~ 1.35 g). The legacy filter treated the burst as a tilted
  // gravity vector and walked the attitude away — the "noisy in transition"
  // in PR #5. Accel feedback outside [0.85, 1.15] g must be gated, so the
  // attitude must not move at all.
  constexpr float kDt = 0.005f;
  constexpr int kBurstSteps = 1000;  // 5 s of burst at 200 Hz
  // gravity (1 g on Z) + 0.9 g lateral -> |a| ~ 1.35 g, outside the accel gate
  constexpr float kBurstLateralG = 0.9f;

  MadgwickFilter f;
  f.initFromAccel(0.0f, 0.0f, 1.0f);

  float peakDeg = 0.0f;
  for (int i = 0; i < kBurstSteps; ++i)
  {
    f.updateIMU(0.0f, 0.0f, 0.0f, kBurstLateralG, 0.0f, 1.0f, kDt);
    peakDeg = std::max(peakDeg, maxAbsDeg(eulerDegOf(f)));
  }

  EXPECT_LT(peakDeg, kGateLeakTolDeg);
}

}  // namespace
