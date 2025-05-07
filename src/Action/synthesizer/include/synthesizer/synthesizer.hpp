#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class Synthesizer : public rclcpp::Node {
public:
    explicit Synthesizer(const rclcpp::NodeOptions& options);

private:
    void angular_vel_callback(const std_msgs::msg::Float32::SharedPtr msg);

    void linear_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angular_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr linear_vel_sub_;

    geometry_msgs::msg::Twist cmd_;
};