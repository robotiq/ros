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

#include <Eigen/Geometry>

#include <memory>

namespace robotiq_tsf {

// Attitude of one finger from its 6-axis IMU (gyro + accel; no magnetometer,
// so yaw is gyro-only and drifts with residual bias). Runs on x-io's Fusion
// AHRS (third_party/fusion), with the interface and tuning the driver exposed
// before: beta is the tilt-correction rate (2 * beta rad/s), and accelerometer
// readings outside [accel_gate_lo, accel_gate_hi] g are not trusted as gravity.
class OrientationFilter
{
public:
   static constexpr float kDefaultBeta = 0.041f;

   explicit OrientationFilter(float beta = kDefaultBeta);
   ~OrientationFilter();
   OrientationFilter(OrientationFilter&&) noexcept;
   OrientationFilter& operator=(OrientationFilter&&) noexcept;

   void reset();
   void setBeta(float beta);
   void setAccelGate(float lo, float hi);

   // Seed the attitude so the gravity vector in the body frame matches the
   // supplied accelerometer reading (any units; only the direction is used).
   // Yaw is set to zero. Use the calibration-time accel mean.
   void initFromAccel(float ax, float ay, float az);

   // gyro in rad/s, accel in any consistent unit (normalised internally),
   // dt in seconds (the measured interval between samples).
   void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

   Eigen::Quaternionf quaternion() const;
   void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;
   void getEulerDeg(float& roll, float& pitch, float& yaw) const;

private:
   // Fusion's C types stay out of this installed header.
   struct Ahrs;
   std::unique_ptr<Ahrs> ahrs_;
   float beta_;
   float accel_gate_lo_; // expected |a| ≈ 1.0 g when stationary
   float accel_gate_hi_;
};

} // namespace robotiq_tsf
