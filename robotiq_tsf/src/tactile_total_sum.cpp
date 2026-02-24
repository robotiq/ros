#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robotiq_tsf/msg/static_data.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <numeric>
#include <vector>

namespace
{
constexpr std::size_t kTaxelCount = 28;
constexpr std::size_t kInitializationSamples = 1000;
}  // namespace

namespace robotiq_tsf
{

namespace msg = robotiq_tsf::msg;

class TactileTotalSumNode : public rclcpp::Node
{
public:
  TactileTotalSumNode() : Node("tactile_total_sum")
  {
    using std::placeholders::_1;
    auto sensor_qos = rclcpp::SensorDataQoS();
    static_subscription_ = create_subscription<msg::StaticData>(
      "TactileSensor4/StaticData", sensor_qos,
      std::bind(&TactileTotalSumNode::handle_static_data, this, _1));

    force_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "force_gauge_reading", sensor_qos,
      std::bind(&TactileTotalSumNode::handle_force_reading, this, _1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(16),
      std::bind(&TactileTotalSumNode::publish_diagnostics, this));

    tactile_bias_sums_[0].assign(kTaxelCount, 0.0);
    tactile_bias_sums_[1].assign(kTaxelCount, 0.0);
    tactile_bias_[0].assign(kTaxelCount, 0.0);
    tactile_bias_[1].assign(kTaxelCount, 0.0);
    tactile_real_values_[0].assign(kTaxelCount, 0.0);
    tactile_real_values_[1].assign(kTaxelCount, 0.0);

    RCLCPP_INFO(get_logger(),
                "Tactile sensors initialization started (%zu samples required).",
                kInitializationSamples);
  }

private:
  void handle_static_data(const msg::StaticData::SharedPtr msg)
  {
    if (msg->taxels.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Static message did not contain two sensor arrays.");
      return;
    }

    if (!initialization_complete_) {
      for (size_t sensor_idx = 0; sensor_idx < 2; ++sensor_idx) {
        for (size_t taxel_idx = 0; taxel_idx < msg->taxels[sensor_idx].values.size(); ++taxel_idx) {
          tactile_bias_sums_[sensor_idx][taxel_idx] +=
            static_cast<double>(msg->taxels[sensor_idx].values[taxel_idx]);
        }
      }
      ++initialization_samples_;
      if (initialization_samples_ >= kInitializationSamples) {
        finalize_biases();
      }
      return;
    }

    for (size_t sensor_idx = 0; sensor_idx < 2; ++sensor_idx) {
      tactile_sum_[sensor_idx] = 0.0;
      for (size_t taxel_idx = 0; taxel_idx < msg->taxels[sensor_idx].values.size(); ++taxel_idx) {
        const double raw_value = static_cast<double>(msg->taxels[sensor_idx].values[taxel_idx]);
        const double corrected = raw_value - tactile_bias_[sensor_idx][taxel_idx];
        tactile_real_values_[sensor_idx][taxel_idx] = corrected;
        tactile_sum_[sensor_idx] += corrected;
      }
    }

    largest_taxel_idx_[0] = find_largest_taxel(tactile_real_values_[0]);
    largest_taxel_idx_[1] = find_largest_taxel(tactile_real_values_[1]);
    new_data_available_ = true;
  }

  void handle_force_reading(const std_msgs::msg::Float64::SharedPtr msg)
  {
    force_gauge_ = msg->data;
  }

  std::size_t find_largest_taxel(const std::vector<double> & taxels) const
  {
    return static_cast<std::size_t>(
      std::max_element(taxels.begin(), taxels.end()) - taxels.begin());
  }

  void finalize_biases()
  {
    for (size_t sensor_idx = 0; sensor_idx < 2; ++sensor_idx) {
      for (size_t taxel_idx = 0; taxel_idx < tactile_bias_sums_[sensor_idx].size(); ++taxel_idx) {
        tactile_bias_[sensor_idx][taxel_idx] =
          tactile_bias_sums_[sensor_idx][taxel_idx] /
          static_cast<double>(kInitializationSamples);
      }
    }
    initialization_complete_ = true;
    RCLCPP_INFO(get_logger(), "Tactile sensors initialization completed.");
  }

  void publish_diagnostics()
  {
    if (!initialization_complete_ || !new_data_available_) {
      return;
    }
    new_data_available_ = false;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "Force %.2f | S1 taxel[%zu]=%.2f sum=%.2f | S2 taxel[%zu]=%.2f sum=%.2f",
                         force_gauge_,
                         largest_taxel_idx_[0],
                         tactile_real_values_[0][largest_taxel_idx_[0]],
                         tactile_sum_[0],
                         largest_taxel_idx_[1],
                         tactile_real_values_[1][largest_taxel_idx_[1]],
                         tactile_sum_[1]);
  }

  rclcpp::Subscription<msg::StaticData>::SharedPtr static_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<std::vector<double>, 2> tactile_bias_sums_;
  std::array<std::vector<double>, 2> tactile_bias_;
  std::array<std::vector<double>, 2> tactile_real_values_;
  std::array<double, 2> tactile_sum_ {0.0, 0.0};
  std::array<std::size_t, 2> largest_taxel_idx_ {0, 0};

  std::size_t initialization_samples_ {0};
  bool initialization_complete_ {false};
  bool new_data_available_ {false};
  double force_gauge_ {0.0};
};

}  // namespace robotiq_tsf

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotiq_tsf::TactileTotalSumNode>());
  rclcpp::shutdown();
  return 0;
}
