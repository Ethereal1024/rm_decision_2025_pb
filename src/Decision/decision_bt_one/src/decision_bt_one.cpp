#include "decision_bt_one/decision_bt_one.hpp"

DecisionBTOne::DecisionBTOne(const rclcpp::NodeOptions& options)
    : RMDecision::DecisionBT(7, "decision", options) {
    this->declare_parameter<std::string>("faction", "RED");
    std::string faction = this->get_parameter("faction").as_string();
    faction_ = faction == "RED" ? RMDecision::Faction::RED : RMDecision::Faction::BLUE;
    RCLCPP_INFO(this->get_logger(), "Faction: %s", faction.c_str());

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "navigator/current_pose", 10, std::bind(&DecisionBTOne::pose_sub_callback, this, std::placeholders::_1));
    hp_sub_ = this->create_subscription<pb_rm_interfaces::msg::GameRobotHP>(
        "referee/all_robot_hp", 10, std::bind(&DecisionBTOne::hp_sub_callback, this, std::placeholders::_1));
    game_sub_ = this->create_subscription<pb_rm_interfaces::msg::GameStatus>(
        "referee/game_status", 10, std::bind(&DecisionBTOne::game_sub_callback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
        "referee/robot_status", 10, std::bind(&DecisionBTOne::status_sub_callback, this, std::placeholders::_1));

    gcp_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DecisionBTOne::gcp_timer_callback, this));

    prism_.self->hp = 400;

    this->awaken();
}

std::string DecisionBTOne::bt_file_path() {
    std::string share_dir = ament_index_cpp::get_package_share_directory("decision_bt_one");
    return share_dir + "/config/bt_one.xml";
}

void DecisionBTOne::register_nodes(RMDecision::RMBT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<GameRunning, DecisionBTOne>("GameRunning", this);
    factory.registerNodeType<OutpostShutdown, DecisionBTOne>("OutpostShutdown", this);
    factory.registerNodeType<HPLow, DecisionBTOne>("HPLow", this);
    factory.registerNodeType<PALow, DecisionBTOne>("PALow", this);
}

void DecisionBTOne::pose_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    prism_.self->pose = *msg;
}

void DecisionBTOne::gcp_timer_callback() const {
    auto p = get_current_coordinate();
    test_display("* (%.3f, %.3f)\n", p.x, p.y);
}

void DecisionBTOne::hp_sub_callback(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg) {
    if (faction_ == RMDecision::Faction::RED) {
        prism_.self->hp = msg->red_7_robot_hp;
        enemy_outpost_hp_ = msg->blue_outpost_hp;
        self_base_hp_ = msg->red_base_hp;
    } else {
        prism_.self->hp = msg->blue_7_robot_hp;
        enemy_outpost_hp_ = msg->red_outpost_hp;
        self_base_hp_ = msg->blue_base_hp;
    }
}

void DecisionBTOne::game_sub_callback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) {
    prism_.game->game_start = (msg->game_progress == msg->RUNNING);
}

void DecisionBTOne::status_sub_callback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) {
    prism_.game->projectile_allowance = msg->projectile_allowance_17mm;
}

bool DecisionBTOne::game_running() const {
    return prism_.game->game_start;
}

bool DecisionBTOne::outpost_shutdown() const {
    return enemy_outpost_hp_ == 0;
}

uint DecisionBTOne::self_hp() const {
    return prism_.self->hp;
}

uint DecisionBTOne::projectile_allowance() const {
    return prism_.game->projectile_allowance;
}

#include "rm_decision_macros/decision_node_regist_macro.hpp"
REGIST_DECISION_NODE(DecisionBTOne);