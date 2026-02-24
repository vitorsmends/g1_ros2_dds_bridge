#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace g1_ros2_dds_bridge {

class LowStateOdomNode final : public rclcpp::Node {
public:
  explicit LowStateOdomNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LowStateOdomNode() override;

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist& msg);
  void on_timer();

  void start_dds();
  void stop_dds();

  void on_lowstate(
      const float quat4[4],
      const float gyro3[3],
      const float acc3[3],
      const float rpy3[3],
      uint32_t tick);

  static double quat_to_yaw_xyzw(double qx, double qy, double qz, double qw);

  int dds_domain_id_{0};
  std::string dds_topic_{"rt/lowstate"};

  std::string ros_topic_{"/dog_odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  bool have_imu_{false};
  uint32_t last_tick_{0};

  double qx_{0.0}, qy_{0.0}, qz_{0.0}, qw_{1.0};
  double yaw_{0.0};
  double x_{0.0}, y_{0.0}, z_{0.0};

  double vx_cmd_{0.0}, vy_cmd_{0.0}, wz_cmd_{0.0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  std::atomic<bool> dds_running_{false};
  std::thread dds_thread_;

  struct DdsImpl;
  std::unique_ptr<DdsImpl> dds_;
};

}  // namespace g1_ros2_dds_bridge