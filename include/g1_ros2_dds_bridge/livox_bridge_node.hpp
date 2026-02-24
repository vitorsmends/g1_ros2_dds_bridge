#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace g1_ros2_dds_bridge {

class LivoxBridgeNode final : public rclcpp::Node {
public:
  explicit LivoxBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LivoxBridgeNode() override;

private:
  void start_dds();
  void stop_dds();

  void on_cloud(
      int32_t sec,
      uint32_t nanosec,
      const std::string& frame_id,
      uint32_t height,
      uint32_t width,
      bool is_bigendian,
      uint32_t point_step,
      uint32_t row_step,
      const std::vector<uint8_t>& data,
      bool is_dense);

  int dds_domain_id_{0};
  std::string dds_topic_{"rt/utlidar/cloud_livox_mid360"};

  std::string ros_topic_{"/livox/lidar"};
  std::string frame_id_override_{""};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  std::atomic<bool> dds_running_{false};
  std::thread dds_thread_;

  struct DdsImpl;
  std::unique_ptr<DdsImpl> dds_;
};

}  // namespace g1_ros2_dds_bridge