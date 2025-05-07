#include "synthesizer/synthesizer.hpp"

Synthesizer::Synthesizer(const rclcpp::NodeOptions& options) : Node("synthesizer", options) {
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    angular_vel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "angular_cmd_vel", 10, std::bind(&Synthesizer::angular_vel_callback, this, std::placeholders::_1));
    linear_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "linear_cmd_vel", 10, std::bind(&Synthesizer::linear_vel_callback, this, std::placeholders::_1));

    geometry_msgs::msg::Twist zero_twist{};
    cmd_ = zero_twist;
}

void Synthesizer::angular_vel_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    cmd_.angular.z = msg->data;
    vel_pub_->publish(cmd_);
}

void Synthesizer::linear_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_.linear = msg->linear;
    vel_pub_->publish(cmd_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Synthesizer)