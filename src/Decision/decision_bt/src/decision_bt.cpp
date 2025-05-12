#include "decision_bt/decision_bt.hpp"

using namespace RMDecision;

DecisionBT::DecisionBT(uint selfId, std::string nodeName, const rclcpp::NodeOptions& options)
    : DecisionBeta(selfId, nodeName, options) {}

DecisionBT::~DecisionBT() {
    if (bt_exec_thread_.joinable()) {
        bt_exec_thread_.join();
    }
}

void DecisionBT::bt_exec() {
    test_display("[ ####### TREE TICKING ####### ]\n");
    auto status = tree_.tickOnce();
    DecisionBeta::test_display("[ ###### STATUS:%s ###### ]\n\n", toStr(status).c_str());

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        tree_.sleep(std::chrono::milliseconds(100));
        test_display("[ ####### TREE TICKING ####### ]\n");
        status = tree_.tickOnce();
        test_display("[ ###### STATUS:%s ###### ]\n\n", BT::toStr(status).c_str());
    }
}

void DecisionBT::register_basic_nodes(RMBT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<NavToPoint, DecisionBT>("NavToPoint", this);
    factory.registerNodeType<NavToPointSerially, DecisionBT>("NavToPointSerially", this);
    factory.registerNodeType<MoveToPoint, DecisionBT>("MoveToPoint", this);
    factory.registerNodeType<RotateToAngle, DecisionBT>("RotateToAngle", this);
    factory.registerNodeType<RotateToVec, DecisionBT>("RotateToVec", this);
    factory.registerNodeType<Spin, DecisionBT>("Spin", this);
    factory.registerNodeType<PointAchieved, DecisionBT>("PointAchieved", this);
    factory.registerNodeType<AngleAchieved, DecisionBT>("AngleAchieved", this);
}

void DecisionBT::register_nodes(RMBT::BehaviorTreeFactory& factory) {
    return;
}

std::string DecisionBT::bt_file_path() {
    return "?";
}

void DecisionBT::awaken() {
    RMBT::BehaviorTreeFactory factory;
    this->register_basic_nodes(factory);
    this->register_nodes(factory);
    factory.registerBehaviorTreeFromFile(bt_file_path());
    tree_ = factory.createTree("MainTree");
    bt_exec_thread_ = std::thread(&DecisionBT::bt_exec, this);
}

void DecisionBT::nav_to_point(const PlaneCoordinate& targetPoint) const {
    DecisionBeta::nav_to_point(targetPoint);
}

void DecisionBT::nav_to_point_serially(const PlaneCoordinate& targetPoint) const {
    DecisionBeta::nav_to_point_serially(targetPoint);
}

void DecisionBT::move_to_point(const PlaneCoordinate& targetPoint) const {
    DecisionBeta::move_to_point(targetPoint);
}

void DecisionBT::rotate_to_angle(const double& targetAngle) const {
    DecisionBeta::rotate_to_angle(targetAngle);
}

void DecisionBT::rotate_to_vec(const PlaneCoordinate& vec) const {
    DecisionBeta::rotate_to_vec(vec);
}

void DecisionBT::set_angular_velocity(const double& speed) const {
    DecisionBeta::set_angular_velocity(speed);
}

PlaneCoordinate DecisionBT::get_current_coordinate() const {
    return DecisionBeta::get_current_coordinate();
}

double DecisionBT::get_current_angle() const {
    return DecisionBeta::get_current_angle();
}