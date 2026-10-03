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

// Regression test pinning the ROS surface of poll_data_sdk_node: the set of
// advertised topics and the service. This is exactly what silently regressed
// once (the TactileSensor/Quaternion topic was dropped in the SDK-node port),
// and the sdk_bridge/device_autodetect gtests don't cover the node's own graph.
//
// The node's publishers/service are created in its constructor, independently
// of startSensor()/waitForStreaming(), so this needs no hardware sensor.

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robotiq_tsf/msg/euler_angle.hpp"
#include "robotiq_tsf/msg/quaternion.hpp"
#include "robotiq_tsf/poll_data_sdk_node.hpp"

namespace {
// Wait (spinning to let intra-process graph discovery settle) until `node`
// reports all of `expected`, or the timeout elapses. Returns the last snapshot.
template <typename QueryFn>
std::map<std::string, std::vector<std::string>> waitForNames(const rclcpp::Node::SharedPtr& node,
                                                             const std::vector<std::string>& expected,
                                                             QueryFn query)
{
   using namespace std::chrono; // NOLINT(build/namespaces)
   const auto deadline = steady_clock::now() + seconds(10);
   // Executor rather than the free rclcpp::spin_some(node), which is deprecated
   // from Lyrical on.
   rclcpp::executors::SingleThreadedExecutor executor;
   executor.add_node(node);
   std::map<std::string, std::vector<std::string>> names;
   while(steady_clock::now() < deadline)
   {
      executor.spin_some();
      names = query();
      bool all = true;
      for(const auto& e : expected)
      {
         if(names.find(e) == names.end())
         {
            all = false;
            break;
         }
      }
      if(all)
      {
         break;
      }
      std::this_thread::sleep_for(milliseconds(50));
   }
   return names;
}
} // namespace

TEST(PollDataSdkNodeSurface, AdvertisesExpectedTopicsAndService)
{
   rclcpp::init(0, nullptr);
   {
      auto node = std::make_shared<PollDataSdkNode>();

      const std::vector<std::string> expected_topics = {
         "/TactileSensor/StaticData",
         "/TactileSensor/Dynamic",
         "/TactileSensor/Accelerometer",
         "/TactileSensor/Gyroscope",
         "/TactileSensor/EulerAngle",
         "/TactileSensor/Quaternion",
         "/TactileSensor/Timestamp",
      };

      const auto topics = waitForNames(node, expected_topics, [&] { return node->get_topic_names_and_types(); });
      for(const auto& t : expected_topics)
      {
         EXPECT_TRUE(topics.find(t) != topics.end()) << "missing advertised topic: " << t;
      }

      const std::vector<std::string> expected_services = {
         "/tactile_sensors_service",
      };
      const auto services = waitForNames(node, expected_services, [&] { return node->get_service_names_and_types(); });
      EXPECT_TRUE(services.find("/tactile_sensors_service") != services.end()) << "missing tactile_sensors_service";
   }
   rclcpp::shutdown();
}

TEST(PollDataSdkNodeOrientation, FirstOrientationAfterCalibrationIsTheSeededAttitude)
{
   // The node calibrates on its first 5000 frames and seeds the filters on the
   // next one, which is also the first frame it publishes orientation on.
   // That message must carry the seeded attitude, not unfilled zeros.
   constexpr int kFramesUntilFirstOrientation = 5001;
   constexpr float kRollDeg = 30.0f;
   constexpr float kCountsPerG = 32768.0f / 2.0f;
   constexpr uint64_t kSamplePeriodUs = 1000;
   constexpr double kTolDeg = 0.1;
   constexpr double kUnitNormTol = 1e-5;

   const float rollRad = kRollDeg * static_cast<float>(M_PI) / 180.0f;
   Fingers fingers{};
   for(auto& finger : fingers.finger)
   {
      finger.accelerometer[1] = static_cast<int16_t>(std::lround(std::sin(rollRad) * kCountsPerG));
      finger.accelerometer[2] = static_cast<int16_t>(std::lround(std::cos(rollRad) * kCountsPerG));
   }

   rclcpp::init(0, nullptr);
   {
      auto node = std::make_shared<PollDataSdkNode>();
      auto probe = std::make_shared<rclcpp::Node>("orientation_probe");
      robotiq_tsf::msg::Quaternion::SharedPtr quaternion;
      robotiq_tsf::msg::EulerAngle::SharedPtr euler;
      auto qSub =
         probe->create_subscription<robotiq_tsf::msg::Quaternion>("TactileSensor/Quaternion",
                                                                  rclcpp::SensorDataQoS(),
                                                                  [&](robotiq_tsf::msg::Quaternion::SharedPtr m) {
                                                                     if(!quaternion)
                                                                     {
                                                                        quaternion = m;
                                                                     }
                                                                  });
      auto eSub =
         probe->create_subscription<robotiq_tsf::msg::EulerAngle>("TactileSensor/EulerAngle",
                                                                  rclcpp::SensorDataQoS(),
                                                                  [&](robotiq_tsf::msg::EulerAngle::SharedPtr m) {
                                                                     if(!euler)
                                                                     {
                                                                        euler = m;
                                                                     }
                                                                  });

      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(probe);
      // Let discovery match the publishers before anything is sent.
      const auto discoveryDeadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while((qSub->get_publisher_count() == 0 || eSub->get_publisher_count() == 0)
            && std::chrono::steady_clock::now() < discoveryDeadline)
      {
         executor.spin_some();
         std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      for(int i = 0; i < kFramesUntilFirstOrientation; ++i)
      {
         for(auto& finger : fingers.finger)
         {
            finger.timestamp = kSamplePeriodUs * static_cast<uint64_t>(i + 1);
         }
         node->handleFingers(fingers);
      }

      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while((!quaternion || !euler) && std::chrono::steady_clock::now() < deadline)
      {
         executor.spin_some();
         std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      ASSERT_TRUE(quaternion) << "no Quaternion published";
      ASSERT_TRUE(euler) << "no EulerAngle published";

      for(int f = 0; f < FINGER_COUNT; ++f)
      {
         const auto& q = quaternion->data[f].values; // w, x, y, z
         EXPECT_NEAR(Eigen::Quaterniond(q[0], q[1], q[2], q[3]).norm(), 1.0, kUnitNormTol) << "finger " << f;
         const auto& e = euler->data[f].values;
         EXPECT_NEAR(e[0], kRollDeg, kTolDeg) << "finger " << f;
         EXPECT_NEAR(e[1], 0.0, kTolDeg) << "finger " << f;
         EXPECT_NEAR(e[2], 0.0, kTolDeg) << "finger " << f;
      }
   }
   rclcpp::shutdown();
}
