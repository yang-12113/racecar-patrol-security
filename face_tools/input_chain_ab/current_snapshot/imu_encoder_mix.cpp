#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite_quaternion(float x, float y, float z, float w)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

bool get_rpy_from_imu(
  const sensor_msgs::msg::Imu & imu,
  double & roll,
  double & pitch,
  double & yaw)
{
  if (!finite_quaternion(
      imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w))
  {
    return false;
  }

  tf2::Quaternion q(
    imu.orientation.x,
    imu.orientation.y,
    imu.orientation.z,
    imu.orientation.w);
  if (q.length2() < 1e-12) {
    return false;
  }

  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
}
}  // namespace

class EncoderImuMixNode : public rclcpp::Node
{
public:
  EncoderImuMixNode()
  : Node("encoder_imu_mix")
  {
    imu_timeout_sec_ = this->declare_parameter("imu_timeout_sec", 0.30);
    max_integration_dt_sec_ = this->declare_parameter("max_integration_dt_sec", 0.20);

    encoder_imu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/encoder_imu_odom", 10);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/IMU_data",
      rclcpp::SensorDataQoS(),
      std::bind(&EncoderImuMixNode::imu_callback, this, std::placeholders::_1));
    encoder_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/encoder",
      rclcpp::SensorDataQoS(),
      std::bind(&EncoderImuMixNode::encoder_callback, this, std::placeholders::_1));
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double roll;
    double pitch;
    double yaw;
    if (!get_rpy_from_imu(*msg, roll, pitch, yaw)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "invalid IMU orientation, skip cache update");
      return;
    }

    latest_imu_ = *msg;
    latest_roll_ = roll;
    latest_pitch_ = pitch;
    latest_yaw_ = yaw;
    latest_imu_rx_time_ = this->now();
    has_valid_imu_ = true;
  }

  void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto now = this->now();
    if (!has_valid_imu_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "waiting for valid IMU before publishing encoder_imu_odom");
      return;
    }

    const double imu_age = (now - latest_imu_rx_time_).seconds();
    if (imu_age > imu_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "latest IMU is stale for %.3f s, skip encoder_imu_odom update",
        imu_age);
      return;
    }

    const double v = msg->twist.twist.linear.x;
    if (!std::isfinite(v)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "encoder velocity is invalid, skip encoder_imu_odom update");
      return;
    }

    if (!has_theta_origin_) {
      theta_first_ = latest_yaw_;
      has_theta_origin_ = true;
    }

    if (last_publish_time_.nanoseconds() == 0) {
      last_publish_time_ = now;
      publish_odom(now, latest_roll_, latest_pitch_, normalize_angle(latest_yaw_ - theta_first_), 0.0);
      return;
    }

    double dt = (now - last_publish_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      last_publish_time_ = now;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "encoder_imu_mix got invalid dt=%.6f, reset timing",
        dt);
      return;
    }

    if (dt > max_integration_dt_sec_) {
      ++large_dt_skip_count_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "encoder_imu_mix large dt=%.6f exceeds %.3f s, skip unreliable integration "
        "(large-dt skips=%llu)",
        dt,
        max_integration_dt_sec_,
        static_cast<unsigned long long>(large_dt_skip_count_));
      last_publish_time_ = now;
      return;
    }

    const double theta = normalize_angle(latest_yaw_ - theta_first_);
    const double vx = v * std::cos(theta);
    const double vy = v * std::sin(theta);
    if (!std::isfinite(vx) || !std::isfinite(vy)) {
      last_publish_time_ = now;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "encoder_imu_mix computed non-finite velocity, skip publish");
      return;
    }

    x_ += vx * dt;
    y_ += vy * dt;
    if (!std::isfinite(x_) || !std::isfinite(y_)) {
      x_ = 0.0;
      y_ = 0.0;
      last_publish_time_ = now;
      RCLCPP_WARN(this->get_logger(), "encoder_imu_mix pose accumulator reset after non-finite update");
      return;
    }

    publish_odom(now, latest_roll_, latest_pitch_, theta, v);
    last_publish_time_ = now;
  }

  void publish_odom(
    const rclcpp::Time & stamp,
    double roll,
    double pitch,
    double theta,
    double speed)
  {
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, theta);
    quat.normalize();

    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = speed * std::cos(theta);
    odom.twist.twist.linear.y = speed * std::sin(theta);
    odom.twist.twist.angular.z = 0.0;
    encoder_imu_pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoder_imu_pub_;

  sensor_msgs::msg::Imu latest_imu_;
  rclcpp::Time latest_imu_rx_time_;
  rclcpp::Time last_publish_time_;
  double latest_roll_ {0.0};
  double latest_pitch_ {0.0};
  double latest_yaw_ {0.0};
  double theta_first_ {0.0};
  double x_ {0.0};
  double y_ {0.0};
  double imu_timeout_sec_ {0.30};
  double max_integration_dt_sec_ {0.20};
  std::uint64_t large_dt_skip_count_ {0};
  bool has_valid_imu_ {false};
  bool has_theta_origin_ {false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EncoderImuMixNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
