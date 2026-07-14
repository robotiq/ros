// MadgwickAHRS.cpp
//
// Class-based Madgwick IMU AHRS filter (gyro+accel only). Derived from the
// reference implementation by Sebastian O.H. Madgwick (x-io.co.uk).
//
// Changes from the original:
//   - Filter state lives on the instance, not in globals.
//   - dt is supplied by the caller from a real clock; no hardcoded sampleFreq.
//   - Gradient-descent feedback is only applied when |a| is near 1 g, so motion
//     bursts do not corrupt the gravity reference.
//   - initFromAccel() seeds the quaternion from the calibration-time gravity
//     vector instead of always starting at identity.
//   - invSqrt bit-hack replaced with 1.0f/sqrtf (no UB, faster on modern HW).

#include "robotiq_tsf/MadgwickAHRS.h"

#include <cmath>

namespace
{
constexpr float kEpsNorm = 1e-12f;

inline float safeRecipSqrt(float x)
{
  if (x <= kEpsNorm)
    return 0.0f;
  return 1.0f / std::sqrt(x);
}
}  // namespace

MadgwickFilter::MadgwickFilter(float beta)
  : q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f), beta_(beta), accel_gate_lo_(0.85f), accel_gate_hi_(1.15f)
{
}

void MadgwickFilter::reset()
{
  q0_ = 1.0f;
  q1_ = 0.0f;
  q2_ = 0.0f;
  q3_ = 0.0f;
}

void MadgwickFilter::setBeta(float beta)
{
  beta_ = beta;
}

void MadgwickFilter::setAccelGate(float lo, float hi)
{
  accel_gate_lo_ = lo;
  accel_gate_hi_ = hi;
}

void MadgwickFilter::initFromAccel(float ax, float ay, float az)
{
  // Normalise the accelerometer reading to get the gravity direction in body
  // frame. Build the quaternion that rotates body gravity onto world +Z, with
  // yaw fixed at zero (no magnetometer reference available).
  const float norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm <= kEpsNorm)
  {
    reset();
    return;
  }
  const float gx = ax / norm;
  const float gy = ay / norm;
  const float gz = az / norm;

  // Tait-Bryan: with gravity = (-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch))
  // in body frame for ZYX rotation from world (assuming world gravity = +Z).
  const float roll = std::atan2(gy, gz);
  const float pitch = std::atan2(-gx, std::sqrt(gy * gy + gz * gz));
  const float yaw = 0.0f;

  const float cr = std::cos(roll * 0.5f);
  const float sr = std::sin(roll * 0.5f);
  const float cp = std::cos(pitch * 0.5f);
  const float sp = std::sin(pitch * 0.5f);
  const float cy = std::cos(yaw * 0.5f);
  const float sy = std::sin(yaw * 0.5f);

  // ZYX composition: q = qz * qy * qx
  q0_ = cr * cp * cy + sr * sp * sy;
  q1_ = sr * cp * cy - cr * sp * sy;
  q2_ = cr * sp * cy + sr * cp * sy;
  q3_ = cr * cp * sy - sr * sp * cy;
}

void MadgwickFilter::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
  if (dt <= 0.0f)
    return;

  // Rate of change of quaternion from gyroscope.
  float qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  float qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  float qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  float qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  const float accelNormSq = ax * ax + ay * ay + az * az;
  if (accelNormSq > kEpsNorm)
  {
    const float accelNorm = std::sqrt(accelNormSq);
    // Only trust accel as a gravity reference when its magnitude is close
    // to 1 g; outside this band the sensor is in non-gravity motion and
    // the gradient step would corrupt the estimate.
    if (accelNorm > accel_gate_lo_ && accelNorm < accel_gate_hi_)
    {
      const float recipNorm = 1.0f / accelNorm;
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      const float _2q0 = 2.0f * q0_;
      const float _2q1 = 2.0f * q1_;
      const float _2q2 = 2.0f * q2_;
      const float _2q3 = 2.0f * q3_;
      const float _4q0 = 4.0f * q0_;
      const float _4q1 = 4.0f * q1_;
      const float _4q2 = 4.0f * q2_;
      const float _8q1 = 8.0f * q1_;
      const float _8q2 = 8.0f * q2_;
      const float q0q0 = q0_ * q0_;
      const float q1q1 = q1_ * q1_;
      const float q2q2 = q2_ * q2_;
      const float q3q3 = q3_ * q3_;

      float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      float s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      float s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;
      const float recipStep = safeRecipSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipStep;
      s1 *= recipStep;
      s2 *= recipStep;
      s3 *= recipStep;

      qDot1 -= beta_ * s0;
      qDot2 -= beta_ * s1;
      qDot3 -= beta_ * s2;
      qDot4 -= beta_ * s3;
    }
  }

  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  const float recipQ = safeRecipSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipQ;
  q1_ *= recipQ;
  q2_ *= recipQ;
  q3_ *= recipQ;
}

void MadgwickFilter::getQuaternion(float& q0, float& q1, float& q2, float& q3) const
{
  q0 = q0_;
  q1 = q1_;
  q2 = q2_;
  q3 = q3_;
}

void MadgwickFilter::getEulerDeg(float& roll, float& pitch, float& yaw) const
{
  const float q[4] = { q0_, q1_, q2_, q3_ };
  quatToEulerDeg(q, roll, pitch, yaw);
}

// --- free helpers --------------------------------------------------------------

void quatMul(const float a[4], const float b[4], float out[4])
{
  // Hamilton product. out = a ⊗ b. Local temporaries so out may alias a or b.
  const float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  const float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  const float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  const float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  out[0] = w;
  out[1] = x;
  out[2] = y;
  out[3] = z;
}

void quatConj(const float q[4], float out[4])
{
  out[0] = q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = -q[3];
}

void quatNormalize(float q[4])
{
  const float n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  const float r = safeRecipSqrt(n);
  q[0] *= r;
  q[1] *= r;
  q[2] *= r;
  q[3] *= r;
}

void quatFromAxisX(float angle_rad, float out[4])
{
  const float h = 0.5f * angle_rad;
  out[0] = std::cos(h);
  out[1] = std::sin(h);
  out[2] = 0.0f;
  out[3] = 0.0f;
}

void quatFromAxisY(float angle_rad, float out[4])
{
  const float h = 0.5f * angle_rad;
  out[0] = std::cos(h);
  out[1] = 0.0f;
  out[2] = std::sin(h);
  out[3] = 0.0f;
}

void quatFromAxisZ(float angle_rad, float out[4])
{
  const float h = 0.5f * angle_rad;
  out[0] = std::cos(h);
  out[1] = 0.0f;
  out[2] = 0.0f;
  out[3] = std::sin(h);
}

void quatToEulerRad(const float q[4], float& roll, float& pitch, float& yaw)
{
  // Matches the legacy ZYX extraction used by PollData.cpp.
  roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  const float sinp = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  pitch = -std::asin(sinp < -1.0f ? -1.0f : (sinp > 1.0f ? 1.0f : sinp));
  yaw = std::atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}

void quatToEulerDeg(const float q[4], float& roll, float& pitch, float& yaw)
{
  quatToEulerRad(q, roll, pitch, yaw);
  constexpr float k = 57.2957795130823f;  // 180/pi
  roll *= k;
  pitch *= k;
  yaw *= k;
}
