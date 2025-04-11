#pragma once

#include "information_widgets/chessboard_def.hpp"
#include "information_widgets/prism_def.hpp"
#include "information_widgets/rm_decision_defs.hpp"
#include "iw_interfaces/msg/chessboard.hpp"
#include "iw_interfaces/msg/prism.hpp"
#include "test_taker_interfaces/msg/test_args.hpp"
#include "navigator_interfaces/msg/navigate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace RMDecision {
class DecisionBase : public rclcpp::Node {
public:
    explicit DecisionBase(uint selfId, std::string nodeName, const rclcpp::NodeOptions& options);

protected:
    void nav_to_point(const double& x, const double& y, bool instant = true);
    void nav_to_point(const PlaneCoordinate& targetPoint, bool instant = true);
    void nav_to_pose(const PoseStamped& stampedPose, bool instant);
    PlaneCoordinate get_current_coordinate();

    void rotate_to_vec(const PlaneCoordinate& vec);
    void rotate_to_angle(const double& targetAngle);
    void set_angular_velocity(const double& angularV);
    double get_current_angle();

    Chessboard chessboard_;
    Prism prism_;

private:
    rclcpp::Subscription<iw_interfaces::msg::Chessboard>::SharedPtr chessboard_sub_;
    rclcpp::Subscription<iw_interfaces::msg::Prism>::SharedPtr prism_sub_;
    rclcpp::Subscription<test_taker_interfaces::msg::TestArgs>::SharedPtr test_args_sub_;

    rclcpp::Publisher<navigator_interfaces::msg::Navigate>::SharedPtr nav_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    void chessboard_sub_callback(const iw_interfaces::msg::Chessboard::SharedPtr msg);

    void prism_sub_callback(const iw_interfaces::msg::Prism::SharedPtr msg);

    virtual void test_callback(const test_taker_interfaces::msg::TestArgs::SharedPtr msg) const = 0;
};

}  // namespace RMDecision
