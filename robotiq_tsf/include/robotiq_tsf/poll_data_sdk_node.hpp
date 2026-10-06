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

// SDK-backed ROS 2 wrapper for the Robotiq TSF tactile sensor.
//
// This node owns a RobotiqTactileSensor (from extern/tactile_sensors/sdk_cpp)
// and republishes the SDK's Fingers callback onto the same topics and message
// types as the legacy PollData.cpp node. The USB / packet / parsing logic lives
// in the SDK — the node contains only the bridge into ROS + the AHRS fusion.

#include <Eigen/Core>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include "finger_data.h" // NOLINT(build/include_subdir) - external SDK header (Fingers, FINGER_COUNT)
#include "rclcpp/node.hpp" // brings rclcpp::{Node, Publisher, Service}
#include "robotiq_tsf/fusion.hpp" // AhrsConfig (by-value member)
// Sensor is the aggregate message and transitively defines every per-field
// message type the publisher members are typed on (StaticData, Dynamic, …), so
// the individual msg headers are pulled in by the .cpp (where create_publisher
// needs their typesupport), not here.
#include "robotiq_tsf/msg/sensor.hpp"
#include "robotiq_tsf/orientation_filter.hpp" // OrientationFilter (by-value array member)
#include "robotiq_tsf/srv/tactile_sensors.hpp"

// Defined by the SDK (extern/tactile_sensors/sdk_cpp); forward-declared here so
// this header doesn't pull in libserialport. The node's out-of-line destructor
// keeps the unique_ptr happy with the incomplete type.
class RobotiqTactileSensor;

class PollDataSdkNode : public rclcpp::Node
{
public:
   PollDataSdkNode();
   ~PollDataSdkNode() override;

   bool startSensor();
   bool waitForStreaming();
   void handleFingers(const Fingers& fingers);

private:
   void serviceCallback(const std::shared_ptr<robotiq_tsf::srv::TactileSensors::Request> req,
                        std::shared_ptr<robotiq_tsf::srv::TactileSensors::Response> res);

   rclcpp::Publisher<robotiq_tsf::msg::StaticData>::SharedPtr static_pub_;
   rclcpp::Publisher<robotiq_tsf::msg::Dynamic>::SharedPtr dynamic_pub_;
   rclcpp::Publisher<robotiq_tsf::msg::Accelerometer>::SharedPtr accel_pub_;
   rclcpp::Publisher<robotiq_tsf::msg::Gyroscope>::SharedPtr gyro_pub_;
   rclcpp::Publisher<robotiq_tsf::msg::EulerAngle>::SharedPtr euler_pub_;
   rclcpp::Publisher<robotiq_tsf::msg::Quaternion>::SharedPtr quat_pub_;
   rclcpp::Publisher<robotiq_tsf::msg::Timestamp>::SharedPtr ts_pub_;
   rclcpp::Service<robotiq_tsf::srv::TactileSensors>::SharedPtr service_;

   std::unique_ptr<RobotiqTactileSensor> sensor_;
   robotiq_tsf::msg::Sensor sensors_data_;

   // Publish-rate throttle: set via the publish_rate_hz parameter, from which
   // this is derived as 1/publish_rate_hz. Default 0 = publish every packet at
   // the sensor's native rate (~700 Hz), as poll_data_node did. A positive
   // publish_rate_hz decimates the publishes for downstream consumers that
   // don't need the full rate; the IMU fusion + bias calc still run on every
   // packet regardless (fusion accuracy needs the full rate).
   // Read/written only from the single SDK callback thread.
   std::chrono::duration<double> publish_period_{0.0};
   std::chrono::steady_clock::time_point last_pub_{};

   // Per-finger IMU bias estimates: accumulated as sums during the startup
   // calibration window, then converted to means. The accel "bias" is really
   // the gravity direction and seeds the filter (see handleFingers). Zeroed
   // in the constructor (Eigen vectors default-construct uninitialized).
   int bias_iter_ = 0;
   std::array<Eigen::Vector3f, FINGER_COUNT> accel_bias_;
   std::array<Eigen::Vector3f, FINGER_COUNT> gyro_bias_;

   // AHRS filter — one instance per finger; integrated on every SDK packet.
   robotiq_tsf::AhrsConfig ahrs_cfg_;
   std::array<robotiq_tsf::OrientationFilter, FINGER_COUNT> filter_;
   // Per-finger MCU timestamp of the last integrated sample; zero = not yet
   // seeded (start of stream, or just after the bias calibration).
   std::array<std::chrono::microseconds, FINGER_COUNT> last_ts_{};

   std::atomic<bool> stopped_{false};
   std::atomic<uint64_t> frames_received_{0};
};
