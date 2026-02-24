#include <rclcpp/rclcpp.hpp>

#include "g1_ros2_dds_bridge/lowstate_odom_node.hpp"
#include "g1_ros2_dds_bridge/livox_bridge_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;

  rclcpp::NodeOptions low_opts;
  low_opts.arguments({"--ros-args", "-r", "__node:=lowstate_odom"});
  auto low = std::make_shared<g1_ros2_dds_bridge::LowStateOdomNode>(low_opts);

  rclcpp::NodeOptions livox_opts;
  livox_opts.arguments({"--ros-args", "-r", "__node:=livox_bridge"});
  auto livox = std::make_shared<g1_ros2_dds_bridge::LivoxBridgeNode>(livox_opts);

  exec.add_node(low);
  exec.add_node(livox);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}