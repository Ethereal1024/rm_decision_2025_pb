#include "decision_bt/node_def.hpp"
#include "decision_bt/rmbt.hpp"

namespace BT {
template <>
inline RMDecision::PlaneCoordinate convertFromString(StringView str) {
    auto parts = splitString(str, ';');
    if (parts.size() != 2) {
        throw RuntimeError("invalid input)");
    } else {
        RMDecision::PlaneCoordinate output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        return output;
    }
}
}  // namespace BT

namespace RMDecision {

class DecisionBT;

class NavToPoint : public RMBT::StatefulActionNode<DecisionBT> {
public:
    NavToPoint(const std::string& name,
               DecisionBT* decisionNodeBT,
               const BT::NodeConfig& config)
        : RMBT::StatefulActionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<PlaneCoordinate>("targetPoint")};
    }

    BT::NodeStatus onStart() override {
        if (!getInput<PlaneCoordinate>("targetPoint", goal_)) {
            throw BT::RuntimeError("missing required input [targetPoint]");
        }
        host_->test_display("[ NavToPoint: START ]\n");
        host_->nav_to_point(goal_);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (host_->get_current_coordinate().coincide_with(goal_, 0.1)) {
            host_->test_display("[ NavToPoint: FINISHED ]\n");
            return BT::NodeStatus::SUCCESS;
        } else {
            host_->test_display(
                "[ NavToPoint ] Distance remaining: %.3f", (host_->get_current_coordinate() - goal_).norm());
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        host_->test_display("[ NavToPoint: ABORTED ]\n");
    }

private:
    PlaneCoordinate goal_;
};

class NavToPointSerially : public RMBT::SyncActionNode<DecisionBT> {
public:
    NavToPointSerially(const std::string& name,
                       DecisionBT* decisionNodeBT,
                       const BT::NodeConfig& config)
        : RMBT::SyncActionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<PlaneCoordinate>("targetPoint")};
    }

    BT::NodeStatus tick() override {
        BT::Expected<PlaneCoordinate> point = getInput<PlaneCoordinate>("targetPoint");
        if (!point) {
            throw BT::RuntimeError("missing required input [targetPoint]: ", point.error());
        }
        host_->test_display("[ NavToPointSerially: TRIGGERED ]\n");
        host_->nav_to_point_serially(point.value());
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveToPoint : public RMBT::StatefulActionNode<DecisionBT> {
public:
    MoveToPoint(const std::string& name,
                DecisionBT* decisionNodeBT,
                const BT::NodeConfig& config)
        : RMBT::StatefulActionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<PlaneCoordinate>("targetPoint")};
    }

    BT::NodeStatus onStart() override {
        if (!getInput<PlaneCoordinate>("targetPoint", goal_)) {
            throw BT::RuntimeError("missing required input [targetPoint]");
        }
        host_->test_display("[ MoveToPoint: START ]\n");
        move_thread_ = std::thread(std::bind(&DecisionBT::move_to_point, host_, goal_));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (host_->get_current_coordinate().coincide_with(goal_, 0.1)) {
            if (move_thread_.joinable()) {
                move_thread_.join();
            }
            host_->test_display("[ MoveToPoint: FINISHED ]\n");
            return BT::NodeStatus::SUCCESS;
        } else {
            host_->test_display(
                "[ MoveToPoint ] Distance remaining: %.3f", (host_->get_current_coordinate() - goal_).norm());
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        host_->abort();
        if (move_thread_.joinable()) {
            move_thread_.join();
        }
        host_->reset();
        host_->test_display("[ MoveToPoint: ABORTED ]\n");
    }

private:
    PlaneCoordinate goal_;
    std::thread move_thread_;
};

class RotateToAngle : public RMBT::StatefulActionNode<DecisionBT> {
public:
    RotateToAngle(const std::string& name,
                  DecisionBT* decisionNodeBT,
                  const BT::NodeConfig& config)
        : RMBT::StatefulActionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("targetAngle")};
    }

    BT::NodeStatus onStart() override {
        if (!getInput<double>("targetAngle", goal_)) {
            throw BT::RuntimeError("missing required input [targetAngle]");
        }
        host_->test_display("[ RotateToAngle: STARTED ]\n");
        rotate_thread_ = std::thread(std::bind(&DecisionBT::rotate_to_angle, host_, goal_));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (std::abs(host_->get_current_angle() - goal_) < 0.05) {
            if (rotate_thread_.joinable()) {
                rotate_thread_.join();
            }
            host_->test_display("[ RotateToAngle: FINISHED ]\n");
            return BT::NodeStatus::SUCCESS;
        } else {
            host_->test_display("[ RotateToAngle ] Angle remaining: %.3f", std::abs(host_->get_current_angle() - goal_));
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        host_->abort();
        if (rotate_thread_.joinable()) {
            rotate_thread_.join();
        }
        host_->reset();
        host_->test_display("[ RotateToAngle: ABORTED ]\n");
    }

private:
    double goal_;
    std::thread rotate_thread_;
};

class RotateToVec : public RMBT::StatefulActionNode<DecisionBT> {
public:
    RotateToVec(const std::string& name,
                DecisionBT* decisionNodeBT,
                const BT::NodeConfig& config)
        : RMBT::StatefulActionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<PlaneCoordinate>("targetVec")};
    }

    BT::NodeStatus onStart() override {
        if (!getInput<PlaneCoordinate>("targetVec", goal_)) {
            throw BT::RuntimeError("missing required input [RotateToVec]");
        }
        host_->test_display("[ RotateToVec: STARTED ]\n");
        rotate_thread_ = std::thread(std::bind(&DecisionBT::rotate_to_vec, host_, goal_));
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override {
        if (std::abs(host_->get_current_angle() - goal_.angle()) < 0.05) {
            if (rotate_thread_.joinable()) {
                rotate_thread_.join();
            }
            host_->test_display("[ RotateToVec: FINISHED ]\n");
            return BT::NodeStatus::SUCCESS;
        } else {
            host_->test_display("[ RotateToVec ] Angle remaining: %.3f", std::abs(host_->get_current_angle() - goal_.angle()));
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        host_->abort();
        if (rotate_thread_.joinable()) {
            rotate_thread_.join();
        }
        host_->reset();
        host_->test_display("[ RotateToVec: ABORTED ]\n");
    }

private:
    PlaneCoordinate goal_;
    std::thread rotate_thread_;
};

class PointAchieved : public RMBT::ConditionNode<DecisionBT> {
public:
    PointAchieved(const std::string& name,
                  DecisionBT* decisionNodeBT,
                  const BT::NodeConfig& config)
        : RMBT::ConditionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<PlaneCoordinate>("targetPoint")};
    }

    BT::NodeStatus tick() override {
        BT::Expected<PlaneCoordinate> point = getInput<PlaneCoordinate>("targetPoint");
        if (!point) {
            throw BT::RuntimeError("missing required input [targetPoint]: ", point.error());
        }
        if (host_->get_current_coordinate().coincide_with(point.value(), 0.05)) {
            host_->test_display("[ PointAchieved: (%.3f, %.3f) | TRUE ]\n", point.value().x, point.value().y);
            return BT::NodeStatus::SUCCESS;
        } else {
            host_->test_display("[ PointAchieved: (%.3f, %.3f) | FALSE ]\n", point.value().x, point.value().y);
            return BT::NodeStatus::FAILURE;
        }
    }
};

class AngleAchieved : public RMBT::ConditionNode<DecisionBT> {
public:
    AngleAchieved(const std::string& name,
                  DecisionBT* decisionNodeBT,
                  const BT::NodeConfig& config)
        : RMBT::ConditionNode<DecisionBT>(name, decisionNodeBT, config) {}

    static BT::PortsList providedPorts() {
        return {BT::InputPort<double>("targetAngle")};
    }

    BT::NodeStatus tick() override {
        BT::Expected<double> angle = getInput<double>("targetAngle");
        if (!angle) {
            throw BT::RuntimeError("missing required input [targetAngle]: ", angle.error());
        }
        if (std::abs(host_->get_current_angle() - angle.value()) < 0.01) {
            host_->test_display("[ AngleAchieved: %.3f | TRUE ]\n", angle.value());
            return BT::NodeStatus::SUCCESS;
        } else {
            host_->test_display("[ AngleAchieved: %.3f | FALSE ]\n", angle.value());
            return BT::NodeStatus::FAILURE;
        }
    }
};

}  // namespace RMDecision