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

#include "robotiq_tsf/orientation_filter.hpp"

#include <algorithm>
#include <cmath>

#include "robotiq_tsf/euler_angles.hpp"

extern "C" {
#include "FusionAhrs.h"
}

namespace robotiq_tsf {

namespace {
constexpr float kEpsNorm = 1e-12f;
constexpr float kRadToDeg = 57.2957795130823f;

// Fusion's accelerometer feedback is proportional to sin(tilt error), where
// the filter this one replaced closed any error at a constant 2 * beta rad/s.
// Scheduling the gain as 2 * beta / sin(error) keeps that rate, so beta keeps
// its meaning for users who tuned it. Below this error the gain stops growing
// and the correction turns proportional: a constant rate down to zero error
// would dither around it.
const float kSinProportionalBelow = std::sin(1.0f / kRadToDeg);

FusionVector toFusion(const Eigen::Vector3f& v)
{
   return FusionVector{{v.x(), v.y(), v.z()}};
}
} // namespace

struct OrientationFilter::Ahrs
{
   FusionAhrs state;
   FusionAhrsSettings settings;
};

OrientationFilter::OrientationFilter(float beta)
   : ahrs_(std::make_unique<Ahrs>())
   , beta_(beta)
   , accel_gate_lo_(kDefaultAccelGateLo)
   , accel_gate_hi_(kDefaultAccelGateHi)
{
   // Fusion's own acceleration rejection, gyro-overrange recovery and
   // magnetometer stay off (all 0 in the defaults): the gate above does the
   // rejection, and the driver predates the rest.
   ahrs_->settings = fusionAhrsDefaultSettings;
   ahrs_->settings.convention = FusionConventionNwu; // +Z up, like the driver's world frame
   FusionAhrsInitialise(&ahrs_->state);
   FusionAhrsSetSettings(&ahrs_->state, &ahrs_->settings);
   reset();
}

OrientationFilter::~OrientationFilter() = default;
OrientationFilter::OrientationFilter(OrientationFilter&&) noexcept = default;
OrientationFilter& OrientationFilter::operator=(OrientationFilter&&) noexcept = default;

void OrientationFilter::reset()
{
   FusionAhrsRestart(&ahrs_->state);
   // Fusion's start-up ramps its gain down from 10 over 3 s, a different
   // convergence than beta promises; the driver seeds the attitude from the
   // calibration instead.
   FusionAhrsSkipStartup(&ahrs_->state);
}

void OrientationFilter::setBeta(float beta)
{
   beta_ = beta;
}

void OrientationFilter::setAccelGate(float lo, float hi)
{
   accel_gate_lo_ = lo;
   accel_gate_hi_ = hi;
}

void OrientationFilter::initFromAccel(const Eigen::Vector3f& accel)
{
   const float norm = accel.norm();
   if(norm <= kEpsNorm)
   {
      reset();
      return;
   }
   const Eigen::Vector3f g = accel / norm;

   // Gravity in the body frame for ZYX roll r, pitch p is
   // (-sin p, sin r cos p, cos r cos p); yaw is unobservable, so zero.
   const float roll = std::atan2(g.y(), g.z());
   const float pitch = std::atan2(-g.x(), std::sqrt(g.y() * g.y() + g.z() * g.z()));
   const Eigen::Quaternionf q =
      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

   reset();
   FusionAhrsSetQuaternion(&ahrs_->state, FusionQuaternion{{q.w(), q.x(), q.y(), q.z()}});
}

void OrientationFilter::updateIMU(const Eigen::Vector3f& gyro, const Eigen::Vector3f& accel, float dt)
{
   if(dt <= 0.0f)
   {
      return;
   }

   const float accelNorm = accel.norm();
   const bool accelTrusted = accelNorm > accel_gate_lo_ && accelNorm < accel_gate_hi_;
   if(accelTrusted)
   {
      const Eigen::Vector3f measuredGravity = accel / accelNorm;
      const FusionVector fg = FusionAhrsGetGravity(&ahrs_->state);
      const Eigen::Vector3f predictedGravity(fg.axis.x, fg.axis.y, fg.axis.z);
      // Past 90 deg Fusion's feedback saturates at its sin = 1 value.
      const float sinError =
         measuredGravity.dot(predictedGravity) > 0.0f ? measuredGravity.cross(predictedGravity).norm() : 1.0f;
      ahrs_->settings.gain = 2.0f * beta_ / std::max(sinError, kSinProportionalBelow);
      FusionAhrsSetSettings(&ahrs_->state, &ahrs_->settings);
   }

   // Fusion skips the accelerometer correction for a zero vector, which is
   // how a reading outside the gate (non-gravity motion) is withheld.
   const Eigen::Vector3f fusionAccel = accelTrusted ? accel : Eigen::Vector3f(Eigen::Vector3f::Zero());
   FusionAhrsSetSamplePeriod(&ahrs_->state, dt);
   FusionAhrsUpdateNoMagnetometer(&ahrs_->state, toFusion(gyro * kRadToDeg), toFusion(fusionAccel));
}

Eigen::Quaternionf OrientationFilter::quaternion() const
{
   const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs_->state);
   return {q.element.w, q.element.x, q.element.y, q.element.z};
}

Eigen::Vector3f OrientationFilter::eulerDeg() const
{
   return quatToEulerDeg(quaternion());
}

} // namespace robotiq_tsf
