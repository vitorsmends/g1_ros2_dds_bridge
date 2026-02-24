#include "g1_ros2_dds_bridge/lowstate_odom_node.hpp"

#include <cmath>

namespace g1_ros2_dds_bridge {

struct LowStateOdomNode::DdsImpl {
  void run(std::atomic<bool>& running, LowStateOdomNode* self) {
    while (running.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

static void normalize_quat(double& x, double& y, double& z, double& w) {
  const double n = std::sqrt(x*x + y*y + z*z + w*w);
  if (n > 1e-12) { x/=n; y/=n; z/=n; w/=n; }
}

double LowStateOdomNode::quat_to_yaw_xyzw(double qx, double qy, double qz, double qw) {
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

LowStateOdomNode::LowStateOdomNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("lowstate_odom", options) {

  this->declare_parameter("dds_domain_id", dds_domain_id_);
  this->declare_parameter("dds_topic", dds_topic_);
  this->declare_parameter("ros_topic", ros_topic_);
  this->declare_parameter("odom_frame", odom_frame_);
  this->declare_parameter("base_frame", base_frame_);

  this->get_parameter("dds_domain_id", dds_domain_id_);
  this->get_parameter("dds_topic", dds_topic_);
  this->get_parameter("ros_topic", ros_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(ros_topic_, 10);
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&LowStateOdomNode::on_cmd_vel, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&LowStateOdomNode::on_timer, this));

  dds_ = std::make_unique<DdsImpl>();
  start_dds();

  RCLCPP_INFO(this->get_logger(), "LowStateOdomNode ready");
}

LowStateOdomNode::~LowStateOdomNode() {
  stop_dds();
}

void LowStateOdomNode::start_dds() {
  if (dds_running_.exchange(true)) return;
  dds_thread_ = std::thread([this]() { dds_->run(dds_running_, this); });
}

void LowStateOdomNode::stop_dds() {
  if (!dds_running_.exchange(false)) return;
  if (dds_thread_.joinable()) dds_thread_.join();
}

void LowStateOdomNode::on_cmd_vel(const geometry_msgs::msg::Twist& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  vx_cmd_ = msg.linear.x;
  vy_cmd_ = msg.linear.y;
  wz_cmd_ = msg.angular.z;
}

void LowStateOdomNode::on_lowstate(
    const float quat4[4],
    const float gyro3[3],
    const float acc3[3],
    const float rpy3[3],
    uint32_t tick) {

  (void)gyro3; (void)acc3; (void)rpy3;

  std::lock_guard<std::mutex> lk(mtx_);
  last_tick_ = tick;

  const double a0 = std::abs(quat4[0]);
  const double a3 = std::abs(quat4[3]);

  if (a0 >= a3) {
    qw_ = quat4[0];
    qx_ = quat4[1];
    qy_ = quat4[2];
    qz_ = quat4[3];
  } else {
    qx_ = quat4[0];
    qy_ = quat4[1];
    qz_ = quat4[2];
    qw_ = quat4[3];
  }

  normalize_quat(qx_, qy_, qz_, qw_);
  yaw_ = quat_to_yaw_xyzw(qx_, qy_, qz_, qw_);
  have_imu_ = true;
}

void LowStateOdomNode::on_timer() {
  const auto now = this->get_clock()->now();

  double qx, qy, qz, qw, yaw;
  double vx_cmd, vy_cmd, wz_cmd;
  bool have_imu;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    have_imu = have_imu_;
    qx = qx_; qy = qy_; qz = qz_; qw = qw_;
    yaw = yaw_;
    vx_cmd = vx_cmd_; vy_cmd = vy_cmd_; wz_cmd = wz_cmd_;
  }

  if (!have_imu) return;

  if (last_time_.nanoseconds() == 0) {
    last_time_ = now;
    return;
  }

  const double dt = (now - last_time_).seconds();
  last_time_ = now;
  if (dt <= 0.0) return;

  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);

  const double vx_w = vx_cmd * cos_y - vy_cmd * sin_y;
  const double vy_w = vx_cmd * sin_y + vy_cmd * cos_y;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    x_ += vx_w * dt;
    y_ += vy_w * dt;
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = z_;
  }

  odom.pose.pose.orientation.x = qx;
  odom.pose.pose.orientation.y = qy;
  odom.pose.pose.orientation.z = qz;
  odom.pose.pose.orientation.w = qw;

  odom.twist.twist.linear.x = vx_cmd;
  odom.twist.twist.linear.y = vy_cmd;
  odom.twist.twist.angular.z = wz_cmd;

  odom_pub_->publish(odom);
}

}  // namespace g1_ros2_dds_bridge