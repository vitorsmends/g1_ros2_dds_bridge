#include "g1_ros2_dds_bridge/livox_bridge_node.hpp"

namespace g1_ros2_dds_bridge {

struct LivoxBridgeNode::DdsImpl {
  void run(std::atomic<bool>& running, LivoxBridgeNode* self) {
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

LivoxBridgeNode::LivoxBridgeNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("livox_bridge", options) {

  this->declare_parameter("dds_domain_id", dds_domain_id_);
  this->declare_parameter("dds_topic", dds_topic_);
  this->declare_parameter("ros_topic", ros_topic_);
  this->declare_parameter("frame_id", frame_id_override_);

  this->get_parameter("dds_domain_id", dds_domain_id_);
  this->get_parameter("dds_topic", dds_topic_);
  this->get_parameter("ros_topic", ros_topic_);
  this->get_parameter("frame_id", frame_id_override_);

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ros_topic_, 10);

  dds_ = std::make_unique<DdsImpl>();
  start_dds();

  RCLCPP_INFO(this->get_logger(), "LivoxBridgeNode ready");
}

LivoxBridgeNode::~LivoxBridgeNode() {
  stop_dds();
}

void LivoxBridgeNode::start_dds() {
  if (dds_running_.exchange(true)) return;
  dds_thread_ = std::thread([this]() { dds_->run(dds_running_, this); });
}

void LivoxBridgeNode::stop_dds() {
  if (!dds_running_.exchange(false)) return;
  if (dds_thread_.joinable()) dds_thread_.join();
}

void LivoxBridgeNode::on_cloud(
    int32_t sec,
    uint32_t nanosec,
    const std::string& frame_id,
    uint32_t height,
    uint32_t width,
    bool is_bigendian,
    uint32_t point_step,
    uint32_t row_step,
    const std::vector<uint8_t>& data,
    bool is_dense) {

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp.sec = sec;
  msg.header.stamp.nanosec = nanosec;
  msg.header.frame_id = frame_id_override_.empty() ? frame_id : frame_id_override_;

  msg.height = height;
  msg.width = width;
  msg.is_bigendian = is_bigendian;
  msg.point_step = point_step;
  msg.row_step = row_step;
  msg.is_dense = is_dense;

  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.data = data;

  pub_->publish(msg);
}

}  // namespace g1_ros2_dds_bridge