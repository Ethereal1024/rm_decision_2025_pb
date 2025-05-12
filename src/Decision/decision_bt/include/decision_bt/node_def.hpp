#include "decision_bt/rmbt.hpp"

namespace RMDecision {
class DecisionBT : public DecisionBeta {
public:
    explicit DecisionBT(uint selfId, std::string nodeName, const rclcpp::NodeOptions& options);

    ~DecisionBT() override;

    void nav_to_point(const PlaneCoordinate& targetPoint) const;

    void nav_to_point_serially(const PlaneCoordinate& targetPoint) const;

    void move_to_point(const PlaneCoordinate& targetPoint) const;

    void rotate_to_angle(const double& targetAngle) const;

    void rotate_to_vec(const PlaneCoordinate& vec) const;

    void set_angular_velocity(const double& speed) const;

    PlaneCoordinate get_current_coordinate() const;

    double get_current_angle() const;

protected:
    virtual void register_nodes(RMBT::BehaviorTreeFactory& factory);

    virtual std::string bt_file_path();

    void awaken();

private:
    void
    bt_exec();

    void register_basic_nodes(RMBT::BehaviorTreeFactory& factory);

    std::thread bt_exec_thread_;
    BT::Tree tree_;
};
}  // namespace RMDecision