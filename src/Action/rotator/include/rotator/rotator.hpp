#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class Rotator : public rclcpp::Node {
public:
    explicit Rotator(const rclcpp::NodeOptions& options);

private:
    void angular_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angular_cmd_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angular_velocity_sub_;
};