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
//   - Generic quaternion algebra (composition, integration, normalization)
//     delegated to Eigen; only the Madgwick gradient step stays expanded.

#include "robotiq_tsf/MadgwickAHRS.h"

#include <cmath>

namespace {
constexpr float kEpsNorm = 1e-12f;

inline float safeRecipSqrt(float x)
{
   if(x <= kEpsNorm)
   {
      return 0.0f;
   }
   return 1.0f / std::sqrt(x);
}
} // namespace

MadgwickFilter::MadgwickFilter(float beta)
   : q_(Eigen::Quaternionf::Identity())
   , beta_(beta)
   , accel_gate_lo_(0.85f)
   , accel_gate_hi_(1.15f)
{
}

void MadgwickFilter::reset()
{
   q_ = Eigen::Quaternionf::Identity();
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
   const Eigen::Vector3f a(ax, ay, az);
   // Guard compares the norm (not squaredNorm) against kEpsNorm, matching
   // the pre-Eigen implementation's degenerate-input threshold.
   const float norm = a.norm();
   if(norm <= kEpsNorm)
   {
      reset();
      return;
   }
   const Eigen::Vector3f g = a / norm;

   // Tait-Bryan: with gravity = (-sin(pitch), sin(roll)*cos(pitch), cos(roll)*cos(pitch))
   // in body frame for ZYX rotation from world (assuming world gravity = +Z).
   const float roll = std::atan2(g.y(), g.z());
   const float pitch = std::atan2(-g.x(), std::sqrt(g.y() * g.y() + g.z() * g.z()));

   // ZYX composition with yaw = 0: q = qy(pitch) * qx(roll).
   q_ = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
}

void MadgwickFilter::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
   if(dt <= 0.0f)
   {
      return;
   }

   // Rate of change of quaternion from gyroscope: qDot = 0.5 * q ⊗ (0, ω).
   const Eigen::Quaternionf omega(0.0f, gx, gy, gz);
   Eigen::Quaternionf qDot = q_ * omega;
   qDot.coeffs() *= 0.5f;

   const Eigen::Vector3f accel(ax, ay, az);
   const float accelNormSq = accel.squaredNorm();
   if(accelNormSq > kEpsNorm)
   {
      const float accelNorm = std::sqrt(accelNormSq);
      // Only trust accel as a gravity reference when its magnitude is close
      // to 1 g; outside this band the sensor is in non-gravity motion and
      // the gradient step would corrupt the estimate.
      if(accelNorm > accel_gate_lo_ && accelNorm < accel_gate_hi_)
      {
         // Reciprocal-multiply (not division) to match the reference
         // implementation's rounding exactly.
         const Eigen::Vector3f an = accel * (1.0f / accelNorm);

         // Madgwick gradient-descent step, kept as the reference scalar
         // expansion (it is the algorithm itself, not generic algebra).
         const float q0 = q_.w();
         const float q1 = q_.x();
         const float q2 = q_.y();
         const float q3 = q_.z();
         const float _2q0 = 2.0f * q0;
         const float _2q1 = 2.0f * q1;
         const float _2q2 = 2.0f * q2;
         const float _2q3 = 2.0f * q3;
         const float _4q0 = 4.0f * q0;
         const float _4q1 = 4.0f * q1;
         const float _4q2 = 4.0f * q2;
         const float _8q1 = 8.0f * q1;
         const float _8q2 = 8.0f * q2;
         const float q0q0 = q0 * q0;
         const float q1q1 = q1 * q1;
         const float q2q2 = q2 * q2;
         const float q3q3 = q3 * q3;

         float s0 = _4q0 * q2q2 + _2q2 * an.x() + _4q0 * q1q1 - _2q1 * an.y();
         float s1 = _4q1 * q3q3 - _2q3 * an.x() + 4.0f * q0q0 * q1 - _2q0 * an.y() - _4q1 + _8q1 * q1q1 + _8q1 * q2q2
                  + _4q1 * an.z();
         float s2 = 4.0f * q0q0 * q2 + _2q0 * an.x() + _4q2 * q3q3 - _2q3 * an.y() - _4q2 + _8q2 * q1q1 + _8q2 * q2q2
                  + _4q2 * an.z();
         float s3 = 4.0f * q1q1 * q3 - _2q1 * an.x() + 4.0f * q2q2 * q3 - _2q2 * an.y();
         Eigen::Quaternionf s(s0, s1, s2, s3);
         s.coeffs() *= safeRecipSqrt(s.squaredNorm());

         qDot.coeffs() -= beta_ * s.coeffs();
      }
   }

   q_.coeffs() += qDot.coeffs() * dt;
   q_.coeffs() *= safeRecipSqrt(q_.squaredNorm());
}

void MadgwickFilter::getQuaternion(float& q0, float& q1, float& q2, float& q3) const
{
   q0 = q_.w();
   q1 = q_.x();
   q2 = q_.y();
   q3 = q_.z();
}

void MadgwickFilter::getEulerDeg(float& roll, float& pitch, float& yaw) const
{
   quatToEulerDeg(q_, roll, pitch, yaw);
}
