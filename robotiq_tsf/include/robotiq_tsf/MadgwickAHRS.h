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

// Deprecated: kept for one release so downstream code written against the
// pre-Fusion MadgwickFilter still builds. Use robotiq_tsf::OrientationFilter
// (robotiq_tsf/orientation_filter.hpp) and robotiq_tsf/euler_angles.hpp.

#include <Eigen/Geometry>

#include "robotiq_tsf/euler_angles.hpp"
#include "robotiq_tsf/orientation_filter.hpp"

class [[deprecated("use robotiq_tsf::OrientationFilter")]] MadgwickFilter
{
public:
   explicit MadgwickFilter(float beta = robotiq_tsf::OrientationFilter::kDefaultBeta)
      : filter_(beta)
   {
   }

   void reset() { filter_.reset(); }
   void setBeta(float beta) { filter_.setBeta(beta); }
   void setAccelGate(float lo, float hi) { filter_.setAccelGate(lo, hi); }
   void initFromAccel(float ax, float ay, float az) { filter_.initFromAccel({ax, ay, az}); }
   void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
   {
      filter_.updateIMU({gx, gy, gz}, {ax, ay, az}, dt);
   }

   Eigen::Quaternionf quaternion() const { return filter_.quaternion(); }
   void getQuaternion(float& q0, float& q1, float& q2, float& q3) const
   {
      const Eigen::Quaternionf q = filter_.quaternion();
      q0 = q.w();
      q1 = q.x();
      q2 = q.y();
      q3 = q.z();
   }
   void getEulerDeg(float& roll, float& pitch, float& yaw) const
   {
      const Eigen::Vector3f rpy = filter_.eulerDeg();
      roll = rpy.x();
      pitch = rpy.y();
      yaw = rpy.z();
   }

private:
   robotiq_tsf::OrientationFilter filter_;
};

[[deprecated("use robotiq_tsf::quatToEulerRad")]] inline void quatToEulerRad(const Eigen::Quaternionf& q,
                                                                             float& roll,
                                                                             float& pitch,
                                                                             float& yaw)
{
   const Eigen::Vector3f rpy = robotiq_tsf::quatToEulerRad(q);
   roll = rpy.x();
   pitch = rpy.y();
   yaw = rpy.z();
}

[[deprecated("use robotiq_tsf::quatToEulerDeg")]] inline void quatToEulerDeg(const Eigen::Quaternionf& q,
                                                                             float& roll,
                                                                             float& pitch,
                                                                             float& yaw)
{
   const Eigen::Vector3f rpy = robotiq_tsf::quatToEulerDeg(q);
   roll = rpy.x();
   pitch = rpy.y();
   yaw = rpy.z();
}
