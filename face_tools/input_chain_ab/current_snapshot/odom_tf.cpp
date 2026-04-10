#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class TopicSubscribe01 : public rclcpp::Node
{
public:
  explicit TopicSubscribe01(const std::string &name)
  : Node(name)
  {
    odom_timeout_sec_ = this->declare_parameter("odom_timeout_sec", 0.30);
    max_hold_timeout_sec_ = this->declare_parameter("max_hold_timeout_sec", 1.00);

    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "encoder_imu_odom",
      rclcpp::SensorDataQoS(),
      std::bind(&TopicSubscribe01::odom_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  void publish_tf()
  {
    if (!has_valid_odom_ || !is_finite_odom(odom_msg_)) {
      return;
    }

    const auto now = this->now();
    const double odom_age = (now - last_odom_rx_time_).seconds();
    if (odom_age > odom_timeout_sec_) {
      ++stale_skip_count_;
      if (odom_age > max_hold_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "encoder_imu_odom stale for %.3f s, skip TF broadcast to avoid held pose drift "
          "(stale skips=%llu, beyond max_hold=%.2f s)",
          odom_age,
          static_cast<unsigned long long>(stale_skip_count_),
          max_hold_timeout_sec_);
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "encoder_imu_odom stale for %.3f s, skip TF broadcast to avoid held pose drift "
          "(stale skips=%llu)",
          odom_age,
          static_cast<unsigned long long>(stale_skip_count_));
      }
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    transform.transform.translation.x = odom_msg_.pose.pose.position.x;
    transform.transform.translation.y = odom_msg_.pose.pose.position.y;
    transform.transform.translation.z = odom_msg_.pose.pose.position.z;
    transform.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
    transform.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
    transform.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
    transform.transform.rotation.w = odom_msg_.pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(transform);
  }

private:
  static bool is_finite_odom(const nav_msgs::msg::Odometry &odom)
  {
    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!is_finite_odom(*msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "received invalid encoder_imu_odom, skip TF: pos=(%.3f, %.3f, %.3f)",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
      return;
    }

    odom_msg_ = *msg;
    has_valid_odom_ = true;
    last_odom_rx_time_ = this->now();
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "odom_tf using pose=(%.3f, %.3f)",
      odom_msg_.pose.pose.position.x,
      odom_msg_.pose.pose.position.y);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Odometry odom_msg_;
  bool has_valid_odom_ {false};
  rclcpp::Time last_odom_rx_time_;
  double odom_timeout_sec_ {0.30};
  double max_hold_timeout_sec_ {1.00};
  std::uint64_t stale_skip_count_ {0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicSubscribe01>("odom_tf");

  rclcpp::WallRate loop_rate(60.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->publish_tf();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
