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

// Implementation of PollDataSdkNode (declared in poll_data_sdk_node.hpp).
// The USB / packet / parsing logic lives in the SDK; this file is the bridge
// into ROS plus the AHRS fusion. main() is in poll_data_sdk_main.cpp so the
// class can be constructed by unit tests without a hardware sensor.

#include <strings.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>

#include "RobotiqTactileSensor.h"
#include "rclcpp/rclcpp.hpp"
#include "robotiq_tsf/device_autodetect.hpp"
// Full per-field message headers: create_publisher<T> needs each type's
// typesupport (the header only needs their struct definitions, via sensor.hpp).
#include "robotiq_tsf/msg/accelerometer.hpp"
#include "robotiq_tsf/msg/dynamic.hpp"
#include "robotiq_tsf/msg/euler_angle.hpp"
#include "robotiq_tsf/msg/gyroscope.hpp"
#include "robotiq_tsf/msg/quaternion.hpp"
#include "robotiq_tsf/msg/static_data.hpp"
#include "robotiq_tsf/msg/timestamp.hpp"
#include "robotiq_tsf/poll_data_sdk_node.hpp"
#include "robotiq_tsf/sdk_bridge.hpp"

namespace msg = robotiq_tsf::msg;
namespace srv = robotiq_tsf::srv;

namespace {
constexpr int kBiasCalculationIterations = 5000;
constexpr const char* kDefaultDevice = robotiq_tsf::kDefaultSensorDevice;
constexpr float kAccelRes = 2.0f / 32768.0f;
constexpr float kGyroRes = 250.0f / 32768.0f;

// Raw IMU triple (int16 counts) -> float vector, before resolution scaling.
Eigen::Vector3f toVector3f(const int16_t (&v)[3])
{
   return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}
// SDK sampling period: 1 ms (~1 kHz request rate).
constexpr unsigned int kSamplePeriodMs = 1;

// The SDK callback is a plain C function pointer with no user-data argument, so
// we reach the node instance through this file-scoped pointer. The node sets it
// to `this` at the end of construction (before startSensor() creates the SDK
// reader thread) and clears it in the destructor; atomic because the reader
// thread reads it concurrently with those writes.
std::atomic<PollDataSdkNode*> g_node_for_callback{nullptr};

void onSensorData(const Fingers& fingers)
{
   if(auto* node = g_node_for_callback.load(std::memory_order_acquire))
   {
      node->handleFingers(fingers);
   }
}
} // namespace

// Stop dispatching to this node, then join the SDK reader thread (in sensor_'s
// destructor) while every member it touches is still alive. Doing this here
// rather than relying on member-destruction order avoids a use-after-free: the
// reader thread outlives the node otherwise.
PollDataSdkNode::~PollDataSdkNode()
{
   g_node_for_callback.store(nullptr, std::memory_order_release);
   sensor_.reset();
}

PollDataSdkNode::PollDataSdkNode()
   : rclcpp::Node("poll_data_sdk")
{
   declare_parameter<std::string>("device", kDefaultDevice);

   // Cap the published topic rate (Hz). Default 0 = unthrottled: publish on
   // every sensor packet, matching the legacy poll_data_node. Set a positive
   // rate to decimate (fusion still runs at the full packet rate).
   const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 0.0);
   publish_period_s_ = publish_rate_hz > 0.0 ? 1.0 / publish_rate_hz : 0.0;

   // AHRS tunables (madgwick.*), matching the legacy poll_data_node so existing
   // launch files / configs that set them keep working.
   ahrs_cfg_.beta = static_cast<float>(declare_parameter<double>("madgwick.beta", ahrs_cfg_.beta));
   ahrs_cfg_.accel_gate_lo =
      static_cast<float>(declare_parameter<double>("madgwick.accel_gate_lo", ahrs_cfg_.accel_gate_lo));
   ahrs_cfg_.accel_gate_hi =
      static_cast<float>(declare_parameter<double>("madgwick.accel_gate_hi", ahrs_cfg_.accel_gate_hi));
   ahrs_cfg_.bias_learn_rate =
      static_cast<float>(declare_parameter<double>("madgwick.bias_learn_rate", ahrs_cfg_.bias_learn_rate));
   ahrs_cfg_.still_gyro_eps_deg_s =
      static_cast<float>(declare_parameter<double>("madgwick.still_gyro_eps_deg_s", ahrs_cfg_.still_gyro_eps_deg_s));
   ahrs_cfg_.still_accel_eps_g =
      static_cast<float>(declare_parameter<double>("madgwick.still_accel_eps_g", ahrs_cfg_.still_accel_eps_g));
   ahrs_cfg_.dt_clamp_lo = static_cast<float>(declare_parameter<double>("madgwick.dt_clamp_lo", ahrs_cfg_.dt_clamp_lo));
   ahrs_cfg_.dt_clamp_hi = static_cast<float>(declare_parameter<double>("madgwick.dt_clamp_hi", ahrs_cfg_.dt_clamp_hi));

   for(int f = 0; f < FINGER_COUNT; ++f)
   {
      filter_[f].setBeta(ahrs_cfg_.beta);
      filter_[f].setAccelGate(ahrs_cfg_.accel_gate_lo, ahrs_cfg_.accel_gate_hi);
      accel_bias_[f].setZero();
      gyro_bias_[f].setZero();
   }

   auto qos = rclcpp::SensorDataQoS();
   static_pub_ = create_publisher<msg::StaticData>("TactileSensor/StaticData", qos);
   dynamic_pub_ = create_publisher<msg::Dynamic>("TactileSensor/Dynamic", qos);
   accel_pub_ = create_publisher<msg::Accelerometer>("TactileSensor/Accelerometer", qos);
   gyro_pub_ = create_publisher<msg::Gyroscope>("TactileSensor/Gyroscope", qos);
   euler_pub_ = create_publisher<msg::EulerAngle>("TactileSensor/EulerAngle", qos);
   quat_pub_ = create_publisher<msg::Quaternion>("TactileSensor/Quaternion", qos);
   ts_pub_ = create_publisher<msg::Timestamp>("TactileSensor/Timestamp", qos);

   service_ = create_service<srv::TactileSensors>(
      "tactile_sensors_service",
      [this](const std::shared_ptr<srv::TactileSensors::Request> req,
             std::shared_ptr<srv::TactileSensors::Response> res) { serviceCallback(req, res); });

   // Register for the SDK C callback last: everything it touches now exists,
   // and the reader thread that invokes it isn't created until startSensor().
   g_node_for_callback.store(this, std::memory_order_release);
}

bool PollDataSdkNode::startSensor()
{
   std::string device = get_parameter("device").as_string();

   // If the user left the default device and it isn't present, fall back to
   // the shared autodetect (rq_tsf85_*, then ttyACM*/ttyUSB* by USB
   // descriptor string).
   if(device == kDefaultDevice && !robotiq_tsf::deviceExists(device))
   {
      const std::string detected = robotiq_tsf::autoDetectSensorDevice();
      if(!detected.empty())
      {
         RCLCPP_INFO(get_logger(),
                     "Auto-detected tactile sensor on %s (default %s not present)",
                     detected.c_str(),
                     device.c_str());
         device = detected;
         set_parameter(rclcpp::Parameter("device", device));
      }
      else
      {
         RCLCPP_WARN(get_logger(), "Could not auto-detect tactile sensor; falling back to %s", device.c_str());
      }
   }

   RCLCPP_INFO(get_logger(), "Opening tactile sensor at %s via SDK", device.c_str());

   // The SDK (libserialport) opens the port in raw mode. That is load-bearing,
   // not incidental: the sensor stream is binary, and a freshly re-enumerated
   // CDC-ACM tty defaults to cooked line discipline (ICRNL rewrites 0x0D, IXON
   // eats 0x11/0x13), which mangles the stream and makes the taxels read as
   // rail-to-rail garbage ("flashing red"). This is exactly the bug the old
   // hand-rolled poll_data_node hit (it never forced raw mode); the SDK path is
   // immune only because libserialport sets raw mode for us. If this ever moves
   // off libserialport, the replacement MUST put the tty in raw mode.
   sensor_ = std::make_unique<RobotiqTactileSensor>(device.c_str(), &onSensorData, kSamplePeriodMs);
   if(!sensor_->isConnected())
   {
      RCLCPP_FATAL(get_logger(), "SDK failed to open sensor: %s", sensor_->getLastError());
      sensor_.reset();
      return false;
   }

   // NOTE: the firmware version is purely informational and nothing here
   // uses it, and this sensor/firmware doesn't reliably reply to GET_VERSION
   // (it consistently times out). So we don't query it at all — no point
   // delaying startup for a value we don't need.
   return true;
}

// The sensor can come up opened-but-silent (seen after an unclean shutdown of
// a previous reader: the port opens fine but no packets flow until the stream
// is kicked). Wait for the first callback; if none arrives, cycle the SDK's
// stop/start to re-trigger streaming before giving up.
bool PollDataSdkNode::waitForStreaming()
{
   using namespace std::chrono; // NOLINT(build/namespaces)
   constexpr int kAttempts = 3;
   constexpr auto kFirstDataTimeout = seconds(2);

   for(int attempt = 1; attempt <= kAttempts; ++attempt)
   {
      const uint64_t before = frames_received_.load();
      const auto deadline = steady_clock::now() + kFirstDataTimeout;
      while(steady_clock::now() < deadline)
      {
         if(frames_received_.load() > before)
         {
            return true;
         }
         std::this_thread::sleep_for(milliseconds(50));
      }
      if(attempt < kAttempts)
      {
         RCLCPP_WARN(get_logger(),
                     "No data from sensor after %lds; cycling the stream "
                     "(attempt %d/%d)",
                     static_cast<int64_t>(kFirstDataTimeout.count()),
                     attempt,
                     kAttempts);
         sensor_->stop();
         std::this_thread::sleep_for(milliseconds(200));
         sensor_->start();
      }
   }
   RCLCPP_FATAL(get_logger(),
                "Sensor opened but never streamed data; replug the sensor "
                "(or USB-reset it) and relaunch.");
   return false;
}

void PollDataSdkNode::serviceCallback(const std::shared_ptr<srv::TactileSensors::Request> req,
                                      std::shared_ptr<srv::TactileSensors::Response> res)
{
   RCLCPP_INFO(get_logger(), "TactileSensors service request: [%s]", req->request.c_str());
   if(strcasecmp(req->request.c_str(), "start") == 0)
   {
      stopped_ = false;
      if(sensor_)
      {
         sensor_->start();
      }
      res->response = true;
      return;
   }
   if(strcasecmp(req->request.c_str(), "stop") == 0)
   {
      stopped_ = true;
      if(sensor_)
      {
         sensor_->stop();
      }
      res->response = true;
      return;
   }
   RCLCPP_WARN(get_logger(), "Invalid TactileSensors request");
   res->response = false;
}

void PollDataSdkNode::handleFingers(const Fingers& fingers)
{
   frames_received_.fetch_add(1, std::memory_order_relaxed);
   if(stopped_.load())
   {
      return;
   }

   robotiq_tsf::fillSensorMessages(fingers, sensors_data_);

   if(bias_iter_ > kBiasCalculationIterations)
   {
      constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0);
      for(int f = 0; f < FINGER_COUNT; ++f)
      {
         const auto& finger = fingers.finger[f];
         // Bias-corrected IMU: accel in g, gyro in deg/s. Explicit Vector3f
         // on the left (not auto): the right side is a lazy Eigen expression
         // referencing temporaries, and only the assignment evaluates it.
         const Eigen::Vector3f accel = toVector3f(finger.accelerometer) * kAccelRes - accel_bias_[f];
         const Eigen::Vector3f gyro = toVector3f(finger.gyroscope) * kGyroRes - gyro_bias_[f];

         // Integration step from the finger's MCU timestamp (immune to host
         // jitter, and to the stall a stop()->start() cycle would inject).
         // 0 = not usable (first sample after calibration, or a duplicate/
         // backwards timestamp) -> skip integration for this finger.
         const float dt =
            robotiq_tsf::deriveDt(last_ts_ms_[f], finger.timestamp, ahrs_cfg_.dt_clamp_lo, ahrs_cfg_.dt_clamp_hi);
         last_ts_ms_[f] = finger.timestamp;

         if(dt > 0.0f)
         {
            filter_[f].updateIMU(gyro * deg_to_rad, accel, dt);

            // Online gyro-bias trim while stationary: drift the stored bias
            // slowly toward the residual, so a frozen bias can't leak
            // thermal drift into yaw.
            if(robotiq_tsf::sampleIsStill(gyro, accel, ahrs_cfg_.still_gyro_eps_deg_s, ahrs_cfg_.still_accel_eps_g))
            {
               gyro_bias_[f] = robotiq_tsf::trimBias(gyro_bias_[f], gyro, ahrs_cfg_.bias_learn_rate);
            }
         }

         // One absolute filter quaternion sources both orientation topics;
         // Euler wraps at ±180°, so continuous-orientation consumers use
         // Quaternion.
         const Eigen::Quaternionf q = filter_[f].quaternion();
         sensors_data_.quaternion.data[f].values[0] = q.w();
         sensors_data_.quaternion.data[f].values[1] = q.x();
         sensors_data_.quaternion.data[f].values[2] = q.y();
         sensors_data_.quaternion.data[f].values[3] = q.z();

         const Eigen::Vector3f rpy = filter_[f].eulerDeg();
         sensors_data_.eulerangle.data[f].values[0] = rpy.x();
         sensors_data_.eulerangle.data[f].values[1] = rpy.y();
         sensors_data_.eulerangle.data[f].values[2] = rpy.z();
      }
   }
   else if(bias_iter_ == kBiasCalculationIterations)
   {
      for(int f = 0; f < FINGER_COUNT; ++f)
      {
         gyro_bias_[f] /= kBiasCalculationIterations;
         accel_bias_[f] /= kBiasCalculationIterations;

         // Gravity in the body frame at rest is the signal, not a bias: seed
         // each filter's quaternion from the accel mean, then stop subtracting
         // the accel offset.
         filter_[f].initFromAccel(accel_bias_[f]);
         accel_bias_[f].setZero();

         // Reseed the per-finger dt so the first post-calibration sample skips
         // integration instead of using a stale delta spanning the calibration.
         last_ts_ms_[f] = 0;
      }
      ++bias_iter_;
   }
   else
   {
      for(int f = 0; f < FINGER_COUNT; ++f)
      {
         const auto& finger = fingers.finger[f];
         gyro_bias_[f] += toVector3f(finger.gyroscope) * kGyroRes;
         accel_bias_[f] += toVector3f(finger.accelerometer) * kAccelRes;
      }
      ++bias_iter_;
   }

   // Decimate publishing to publish_period_s_ (fusion above already ran at
   // full rate). Skip the very first packet's elapsed check by seeding
   // last_pub_ on first use.
   if(publish_period_s_ > 0.0)
   {
      const auto now = std::chrono::steady_clock::now();
      if(last_pub_.time_since_epoch().count() != 0)
      {
         const double elapsed = std::chrono::duration<double>(now - last_pub_).count();
         if(elapsed < publish_period_s_)
         {
            return;
         }
      }
      last_pub_ = now;
   }

   static_pub_->publish(sensors_data_.staticdata);
   dynamic_pub_->publish(sensors_data_.dynamic);
   accel_pub_->publish(sensors_data_.accelerometer);
   gyro_pub_->publish(sensors_data_.gyroscope);
   ts_pub_->publish(sensors_data_.timestamp);
   if(bias_iter_ > kBiasCalculationIterations)
   {
      euler_pub_->publish(sensors_data_.eulerangle);
      quat_pub_->publish(sensors_data_.quaternion);
   }
}
