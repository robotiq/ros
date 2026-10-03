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

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robotiq_tsf/orientation_filter.hpp"

// Small, hardware-independent helpers for the AHRS (Attitude and Heading
// Reference System) fusion path, factored out of the node so they can be
// unit-tested directly (like sdk_bridge/device_autodetect).

namespace robotiq_tsf {

// AHRS / fusion tunables, populated from ROS parameters (madgwick.*). Defaults
// match the legacy poll_data_node.
struct AhrsConfig
{
   float beta = OrientationFilter::kDefaultBeta;
   float accel_gate_lo = OrientationFilter::kDefaultAccelGateLo;
   float accel_gate_hi = OrientationFilter::kDefaultAccelGateHi;
   float bias_learn_rate = 0.0005f; // EMA step toward residual gyro when still
   float still_gyro_eps_deg_s = 0.8f; // |omega| below this counts as still
   float still_accel_eps_g = 0.05f; // ||a| - 1g| below this counts as still
   float dt_clamp_lo = 1e-4f; // 0.1 ms
   float dt_clamp_hi = 0.1f; // 100 ms
};

// Integration step (seconds) from consecutive per-finger MCU timestamps (ms).
// The hardware clock is immune to host scheduling jitter, so it reflects the
// interval the firmware actually produced. Returns 0 when the delta isn't
// usable — not yet seeded (prev == 0) or non-increasing (duplicate / backwards
// / wrapped) — signalling "skip integration this frame". A usable delta is
// clamped into [lo, hi] as cheap insurance against an outsized step (a
// scheduling stall, or a stop()->start() service cycle) corrupting the estimate.
inline float deriveDt(uint64_t prev_ts_ms, uint64_t ts_ms, float lo, float hi)
{
   if(prev_ts_ms == 0 || ts_ms <= prev_ts_ms)
   {
      return 0.0f;
   }
   return std::clamp(static_cast<float>(ts_ms - prev_ts_ms) * 1e-3f, lo, hi);
}

// True when the sample looks stationary: small angular rate and accel norm near
// 1 g. gyro in deg/s, accel in g. Gates the online gyro-bias trim.
inline bool sampleIsStill(const Eigen::Vector3f& gyro,
                          const Eigen::Vector3f& accel,
                          float gyro_eps_deg_s,
                          float accel_eps_g)
{
   return gyro.norm() < gyro_eps_deg_s && std::fabs(accel.norm() - 1.0f) < accel_eps_g;
}

// One EMA step of the online gyro-bias trim: nudge `bias` toward the residual
// (post-bias-subtraction) gyro `rate` by `alpha`. Call only when still, so a
// frozen bias can't let thermal drift accumulate as yaw drift over a session.
inline Eigen::Vector3f trimBias(const Eigen::Vector3f& bias, const Eigen::Vector3f& rate, float alpha)
{
   return bias + alpha * rate;
}

} // namespace robotiq_tsf
