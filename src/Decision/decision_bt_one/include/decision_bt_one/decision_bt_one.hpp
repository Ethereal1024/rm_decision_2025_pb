#include "ament_index_cpp/get_package_share_directory.hpp"
#include "decision_bt/decision_bt.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"

class DecisionBTOne : public RMDecision::DecisionBT {
public:
    explicit DecisionBTOne(const rclcpp::NodeOptions& options);

    bool game_running() const;

private:
    void pose_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void hp_sub_callback(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg);

    void game_sub_callback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

    std::string bt_file_path() override;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr hp_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_sub_;
};

class GameRunning : public RMDecision::RMBT::ConditionNode<DecisionBTOne> {
public:
    GameRunning(const std::string& name, DecisionBTOne* host)
        : RMDecision::RMBT::ConditionNode<DecisionBTOne>(name, host, {}) {}

    BT::NodeStatus tick() override {
        return host_->game_running() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};