//
// racecar_driver_node.cpp
//

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "../include/racecar_driver.h"
#include <cstring>

class RacecarDriver : public rclcpp::Node
{
public:
RacecarDriver()
: Node("racecar_driver")
{
this->declare_parameter<std::string>("serial_port", "/dev/car");
this->declare_parameter<int>("baud_rate", 38400);

std::string serial_port;
int baud_rate;

this->get_parameter("serial_port", serial_port);
this->get_parameter("baud_rate", baud_rate);

// Convert std::string to char*
char serial_port_char[serial_port.size() + 1];
std::strcpy(serial_port_char, serial_port.c_str());

art_racecar_init(baud_rate, serial_port_char);

subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
"/car_cmd_vel", 1, std::bind(&RacecarDriver::TwistCallback, this, std::placeholders::_1));
}

private:
void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{

double angle;
angle = 2500.0 - twist->angular.z * 2000.0 / 180.0;
send_cmd(static_cast<uint16_t>(twist->linear.x), static_cast<uint16_t>(angle));
}

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<RacecarDriver>());
rclcpp::shutdown();
return 0;
}
