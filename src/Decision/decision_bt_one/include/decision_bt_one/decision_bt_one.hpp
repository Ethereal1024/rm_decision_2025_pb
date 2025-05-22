#include "ament_index_cpp/get_package_share_directory.hpp"
#include "decision_bt/decision_bt.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

class DecisionBTOne : public RMDecision::DecisionBT {
public:
    explicit DecisionBTOne(const rclcpp::NodeOptions& options);

    bool game_running() const;

    bool outpost_shutdown() const;

    uint self_hp() const;

    uint projectile_allowance() const;

private:
    void pose_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void hp_sub_callback(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg);

    void game_sub_callback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);

    void status_sub_callback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

    void gcp_timer_callback() const;

    std::string bt_file_path() override;

    void register_nodes(RMDecision::RMBT::BehaviorTreeFactory& factory) override;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameRobotHP>::SharedPtr hp_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_sub_;
    rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr status_sub_;

    rclcpp::TimerBase::SharedPtr gcp_timer_;

    uint enemy_outpost_hp_;
    uint self_base_hp_;
    RMDecision::Faction faction_;
};

class GameRunning : public RMDecision::RMBT::ConditionNode<DecisionBTOne> {
public:
    GameRunning(const std::string& name, DecisionBTOne* host)
        : RMDecision::RMBT::ConditionNode<DecisionBTOne>(name, host, {}) {}

    BT::NodeStatus tick() override {
        return host_->game_running() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class OutpostShutdown : public RMDecision::RMBT::ConditionNode<DecisionBTOne> {
public:
    OutpostShutdown(const std::string& name, DecisionBTOne* host)
        : RMDecision::RMBT::ConditionNode<DecisionBTOne>(name, host, {}) {}

    BT::NodeStatus tick() override {
        return host_->outpost_shutdown() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class HPLow : public RMDecision::RMBT::ConditionNode<DecisionBTOne> {
public:
    HPLow(const std::string& name, DecisionBTOne* host, const BT::NodeConfig& config)
        : RMDecision::RMBT::ConditionNode<DecisionBTOne>(name, host, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<uint>("threshold")};
    }

    BT::NodeStatus tick() override {
        BT::Expected<uint> threshold = getInput<uint>("threshold");
        if (!threshold) {
            throw BT::RuntimeError("missing required input [threshold]: ", threshold.error());
        }
        return host_->self_hp() < threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class PALow : public RMDecision::RMBT::ConditionNode<DecisionBTOne> {
public:
    PALow(const std::string& name, DecisionBTOne* host, const BT::NodeConfig& config)
        : RMDecision::RMBT::ConditionNode<DecisionBTOne>(name, host, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<uint>("threshold")};
    }

    BT::NodeStatus tick() override {
        BT::Expected<uint> threshold = getInput<uint>("threshold");
        if (!threshold) {
            throw BT::RuntimeError("missing required input [threshold]: ", threshold.error());
        }
        return host_->projectile_allowance() < threshold.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

class Noispin : public RMDecision::RMBT::StatefulActionNode<DecisionBTOne> {
public:
    Noispin(const std::string& name,
            DecisionBTOne* decisionNodeBT,
            const BT::NodeConfig& config)
        : RMDecision::RMBT::StatefulActionNode<DecisionBTOne>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("speed"), BT::InputPort<double>("amplitude")};
    }

    BT::NodeStatus onStart() override {
        if (!getInput<double>("speed", speed_)) {
            throw BT::RuntimeError("missing required input [speed]");
        }
        if (!getInput<double>("amplitude", amplitude_)) {
            throw BT::RuntimeError("missing required input [amplitude]");
        }
        double noispeed = speed_ + RMDecision::PlaneCoordinate::random_point(amplitude_).x;
        host_->test_display("[ Noispin: STARTED ]\n");
        host_->set_angular_velocity(noispeed);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (std::abs(host_->get_current_angle() - previous_angle_) < 0.1) {
            host_->test_display(
                "[ Noispin ] Attempting to spin... (expected speed: %.3f)", speed_);
            double noispeed = speed_ + RMDecision::PlaneCoordinate::random_point(amplitude_).x;
            host_->set_angular_velocity(noispeed);
            return BT::NodeStatus::RUNNING;
        } else {
            host_->test_display("[ Noispin: FINISHED ]\n");
            return BT::NodeStatus::SUCCESS;
        }
    }

    void onHalted() override {
        host_->test_display("[ Noispin: ABORTED ]\n");
    }

private:
    double speed_;
    double amplitude_;
    double previous_angle_;
};