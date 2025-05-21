#include "ament_index_cpp/get_package_share_directory.hpp"
#include "decision_bt/decision_bt.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"

class DecisionBTOneSim : public RMDecision::DecisionBT {
public:
    explicit DecisionBTOneSim(const rclcpp::NodeOptions& options);

    bool game_running() const;

    bool outpost_shutdown() const;

    uint self_hp() const;

private:
    void pose_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void hp_sub_callback(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg);

    void game_sub_callback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

    std::string bt_file_path() override;

    void register_nodes(RMDecision::RMBT::BehaviorTreeFactory& factory) override;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr hp_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_sub_;

    double enemy_outpost_hp_;
    double self_base_hp_;
    RMDecision::Faction faction_;
};

class GameRunning : public RMDecision::RMBT::ConditionNode<DecisionBTOneSim> {
public:
    GameRunning(const std::string& name, DecisionBTOneSim* host)
        : RMDecision::RMBT::ConditionNode<DecisionBTOneSim>(name, host, {}) {}

    BT::NodeStatus tick() override {
        return host_->game_running() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class OutpostShutdown : public RMDecision::RMBT::ConditionNode<DecisionBTOneSim> {
public:
    OutpostShutdown(const std::string& name, DecisionBTOneSim* host)
        : RMDecision::RMBT::ConditionNode<DecisionBTOneSim>(name, host, {}) {}

    BT::NodeStatus tick() override {
        return host_->outpost_shutdown() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class HPLow : public RMDecision::RMBT::ConditionNode<DecisionBTOneSim> {
public:
    HPLow(const std::string& name, DecisionBTOneSim* host, const BT::NodeConfig& config)
        : RMDecision::RMBT::ConditionNode<DecisionBTOneSim>(name, host, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<uint>("threshold")};
    }

    BT::NodeStatus tick() override {
        BT::Expected<uint> threshold = getInput<uint>("threshold");
        if (!threshold) {
            throw BT::RuntimeError("missing required input [threshold]: ", threshold.error());
        }
        host_->test_display("%d/n", host_->self_hp());
        return host_->self_hp() < threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};