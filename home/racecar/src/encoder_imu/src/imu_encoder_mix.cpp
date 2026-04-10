#include <cmath>
#include <iostream>
#include <limits>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define PI 3.14159

using namespace std;
using namespace message_filters;

rclcpp::Time current_time, last_time;
double x = 0.0;
double y = 0.0;
double s = 0.0;
double th = 0.0;
double vth = 0.0;
double th_init = 0.0;
static double theta_first = 0.0;
static bool flag = true;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr encoder_imu_pub;
static rclcpp::Clock::SharedPtr g_clock;
static rclcpp::Logger g_logger = rclcpp::get_logger("encoder_imu_mix");

double invalid_angle()
{
  return std::numeric_limits<double>::quiet_NaN();
}

bool finite_quaternion(float x, float yv, float z, float w)
{
  return std::isfinite(x) && std::isfinite(yv) && std::isfinite(z) && std::isfinite(w);
}

double getYawFromPose(const sensor_msgs::msg::Imu::ConstSharedPtr &carPose)
{
  const auto &imu = *carPose;
  float qx = imu.orientation.x;
  float qy = imu.orientation.y;
  float qz = imu.orientation.z;
  float qw = imu.orientation.w;

  if (!finite_quaternion(qx, qy, qz, qw)) {
    return invalid_angle();
  }

  tf2::Quaternion q(qx, qy, qz, qw);
  if (q.length2() < 1e-12) {
    return invalid_angle();
  }

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

double getRollFromPose(const sensor_msgs::msg::Imu::ConstSharedPtr &carPose)
{
  const auto &imu = *carPose;
  float qx = imu.orientation.x;
  float qy = imu.orientation.y;
  float qz = imu.orientation.z;
  float qw = imu.orientation.w;

  if (!finite_quaternion(qx, qy, qz, qw)) {
    return invalid_angle();
  }

  tf2::Quaternion q(qx, qy, qz, qw);
  if (q.length2() < 1e-12) {
    return invalid_angle();
  }

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return roll;
}

double getPitchFromPose(const sensor_msgs::msg::Imu::ConstSharedPtr &carPose)
{
  const auto &imu = *carPose;
  float qx = imu.orientation.x;
  float qy = imu.orientation.y;
  float qz = imu.orientation.z;
  float qw = imu.orientation.w;

  if (!finite_quaternion(qx, qy, qz, qw)) {
    return invalid_angle();
  }

  tf2::Quaternion q(qx, qy, qz, qw);
  if (q.length2() < 1e-12) {
    return invalid_angle();
  }

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return pitch;
}

void callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr &imu_data,
  const nav_msgs::msg::Odometry::ConstSharedPtr &speed)
{
  if (flag) {
    theta_first = getYawFromPose(imu_data);
    if (!std::isfinite(theta_first)) {
      RCLCPP_WARN(g_logger, "invalid IMU yaw during initialization, skip encoder_imu_odom update");
      return;
    }
    flag = false;
  }

  double v = speed->twist.twist.linear.x;
  double theta = getYawFromPose(imu_data);
  double roll = getRollFromPose(imu_data);
  double pitch = getPitchFromPose(imu_data);
  if (!std::isfinite(v) || !std::isfinite(theta) || !std::isfinite(roll) || !std::isfinite(pitch)) {
    RCLCPP_WARN(g_logger, "invalid IMU or encoder input, skip encoder_imu_odom publish");
    return;
  }

  theta = theta - theta_first;
  th_init = theta_first * 180 / PI;
  th = theta * 180 / PI;

  double vx = v * cos(theta);
  double vy = v * sin(theta);
  current_time = g_clock ? g_clock->now() : rclcpp::Clock().now();

  if (last_time.nanoseconds() == 0) {
    last_time = current_time;
    return;
  }

  double dt = (current_time - last_time).seconds();
  if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
    last_time = current_time;
    RCLCPP_WARN(g_logger, "ignore abnormal dt=%.6f in encoder_imu_mix", dt);
    return;
  }

  double delta_x = vx * dt;
  double delta_y = vy * dt;
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(delta_x) || !std::isfinite(delta_y)) {
    last_time = current_time;
    RCLCPP_WARN(g_logger, "non-finite displacement computed, skip encoder_imu_odom publish");
    return;
  }

  x += delta_x;
  y += delta_y;
  s = sqrt(x * x + y * y);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(s)) {
    last_time = current_time;
    RCLCPP_WARN(g_logger, "pose accumulator became non-finite, skip encoder_imu_odom publish");
    return;
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, theta);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);
  if (!std::isfinite(odom_quat.x) || !std::isfinite(odom_quat.y) ||
      !std::isfinite(odom_quat.z) || !std::isfinite(odom_quat.w)) {
    last_time = current_time;
    RCLCPP_WARN(g_logger, "non-finite orientation computed, skip encoder_imu_odom publish");
    return;
  }

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = 0.0;

  encoder_imu_pub->publish(odom);
  last_time = current_time;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("encoder_imu_mix");

  auto imu_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(
    node.get(), "/IMU_data", rmw_qos_profile_default);
  auto encoder_sub = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
    node.get(), "/encoder", rmw_qos_profile_default);

  encoder_imu_pub = node->create_publisher<nav_msgs::msg::Odometry>("/encoder_imu_odom", 10);
  g_clock = node->get_clock();

  using MySyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, nav_msgs::msg::Odometry>;
  auto sync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(10), *imu_sub, *encoder_sub);
  sync->registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2));

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
