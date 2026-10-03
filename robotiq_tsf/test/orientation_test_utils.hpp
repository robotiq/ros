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
// failure messages, tilt error between two attitudes, and a synthetic IMU
// stream with ground truth.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "robotiq_tsf/euler_angles.hpp"

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
   const Eigen::Vector3f rpy = robotiq_tsf::quatToEulerDeg(q);
   return {rpy.x(), rpy.y(), rpy.z()};
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

struct ImuSample
{
   Eigen::Vector3f gyro; // rad/s
   Eigen::Vector3f accel; // g
   Eigen::Quaternionf truth;
};

// Hand-held-like motion: roll and pitch sweeping ±40 deg, steady 20 deg/s
// yaw, a 0.4 s linear-acceleration burst every 7 s, white noise (0.004 g,
// 0.1 deg/s) and a 0.05 deg/s residual gyro bias. Starts level.
inline std::vector<ImuSample> handheldMotion(int steps, float dt, unsigned seed = 2)
{
   std::mt19937 rng(seed);
   std::normal_distribution<float> accelNoise(0.0f, 0.004f);
   std::normal_distribution<float> gyroNoise(0.0f, 0.1f * kDegToRad);
   const float gyroBias = 0.05f * kDegToRad;

   std::vector<ImuSample> samples;
   samples.reserve(steps);
   Eigen::Quaternionf truth = Eigen::Quaternionf::Identity();
   for(int i = 0; i < steps; ++i)
   {
      const float t = static_cast<float>(i) * dt;
      const float roll = 40.0f * kDegToRad * std::sin(2.0f * kPi * 0.3f * t);
      const float pitch = 40.0f * kDegToRad * std::sin(2.0f * kPi * 0.5f * t);
      const float yaw = 20.0f * kDegToRad * t;
      const Eigen::Quaternionf next = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
                                    * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                                    * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
      const Eigen::AngleAxisf step(truth.conjugate() * next);
      const Eigen::Vector3f bodyRate = step.axis() * (step.angle() / dt);
      truth = next;

      Eigen::Vector3f linear = Eigen::Vector3f::Zero();
      if(std::fmod(t, 7.0f) < 0.4f)
      {
         linear = Eigen::Vector3f(0.6f * std::sin(40.0f * t), 0.3f, 0.0f);
      }
      const Eigen::Vector3f accel = truth.conjugate() * (Eigen::Vector3f::UnitZ() + linear)
                                  + Eigen::Vector3f(accelNoise(rng), accelNoise(rng), accelNoise(rng));
      const Eigen::Vector3f gyro =
         bodyRate + Eigen::Vector3f(gyroNoise(rng) + gyroBias, gyroNoise(rng), gyroNoise(rng));
      samples.push_back({gyro, accel, truth});
   }
   return samples;
}

} // namespace robotiq_tsf::test
