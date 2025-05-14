#include "decision_beta/decision_beta.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"

class DecisionTestAlpha : public RMDecision::DecisionBeta {
public:
    explicit DecisionTestAlpha(const rclcpp::NodeOptions& options);

private:
    void nav_to_point_test(const std::vector<float>& args);

    void rotate_to_angle_test(const std::vector<float>& args);

    void set_angular_velocity_test(const std::vector<float>& args);

    void set_linear_velocity_test(const std::vector<float>& args);

    void move_to_point_test(const std::vector<float>& args);

    void set_gimbal_state_test(const std::vector<float>& args);

    void get_current_point_test(const std::vector<float>& args);

    void get_current_angle_test(const std::vector<float>& args);

    void set_linear_offset_test(const std::vector<float>& args);

    void set_angular_offset_test(const std::vector<float>& args);

    void get_linear_offset_test(const std::vector<float>& args);

    void get_angular_offset_test(const std::vector<float>& args);

    void mark_origin_linear_test(const std::vector<float>& args);

    void mark_origin_angular_test(const std::vector<float>& args);

    void pose_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void hp_sub_callback(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg);

    void set_angular_velocity_debounce(const double& angularV) const;

    void pass_through_hill(
        const RMDecision::PlaneCoordinate& start,
        const RMDecision::PlaneCoordinate& end,
        const double& angle) const;

    void route_a() const;

    void test_response(const std::string& instruction, const std::vector<float>& args) override;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr hp_sub_;
    std::unordered_map<std::string, std::function<void(const std::vector<float>&)>> test_funcs_;
};