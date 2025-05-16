#include "decision_test_alpha/decision_test_alpha.hpp"

DecisionTestAlpha::DecisionTestAlpha(const rclcpp::NodeOptions& options)
    : RMDecision::DecisionBeta(7, "decision_test_alpha", options) {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "navigator/current_pose", 10, std::bind(&DecisionTestAlpha::pose_sub_callback, this, std::placeholders::_1));

    hp_sub_ = this->create_subscription<pb_rm_interfaces::msg::GameRobotHP>(
        "referee/all_robot_hp", 10, std::bind(&DecisionTestAlpha::hp_sub_callback, this, std::placeholders::_1));

    test_funcs_ = {
        {"NAV", std::bind(&DecisionTestAlpha::nav_to_point_test, this, std::placeholders::_1)},
        {"ROT", std::bind(&DecisionTestAlpha::rotate_to_angle_test, this, std::placeholders::_1)},
        {"SAV", std::bind(&DecisionTestAlpha::set_angular_velocity_test, this, std::placeholders::_1)},
        {"SLV", std::bind(&DecisionTestAlpha::set_linear_velocity_test, this, std::placeholders::_1)},
        {"MOV", std::bind(&DecisionTestAlpha::move_to_point_test, this, std::placeholders::_1)},
        {"SGS", std::bind(&DecisionTestAlpha::set_gimbal_state_test, this, std::placeholders::_1)},
        {"GCP", std::bind(&DecisionTestAlpha::get_current_point_test, this, std::placeholders::_1)},
        {"*", std::bind(&DecisionTestAlpha::get_current_point_test, this, std::placeholders::_1)},
        {"GCA", std::bind(&DecisionTestAlpha::get_current_angle_test, this, std::placeholders::_1)},
        {"SLO", std::bind(&DecisionTestAlpha::set_linear_offset_test, this, std::placeholders::_1)},
        {"SAO", std::bind(&DecisionTestAlpha::set_angular_offset_test, this, std::placeholders::_1)},
        {"GLO", std::bind(&DecisionTestAlpha::get_linear_offset_test, this, std::placeholders::_1)},
        {"GAO", std::bind(&DecisionTestAlpha::get_angular_offset_test, this, std::placeholders::_1)},
        {"MLO", std::bind(&DecisionTestAlpha::mark_origin_linear_test, this, std::placeholders::_1)},
        {"MAO", std::bind(&DecisionTestAlpha::mark_origin_angular_test, this, std::placeholders::_1)},
    };
}

void DecisionTestAlpha::pose_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    prism_.self->pose = *msg;
}

void DecisionTestAlpha::hp_sub_callback(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg) {
    prism_.self->hp = msg->blue_7_robot_hp;
}

void DecisionTestAlpha::set_angular_velocity_debounce(const double& angularV) const {
    int delay = 50;
    while (delay--) {
        set_angular_velocity(angularV);
        test_display("setting angular velocity: %.3f\n", angularV);
        rclcpp::Rate(10).sleep();
    }
}

void DecisionTestAlpha::pass_through_hill(
    const RMDecision::PlaneCoordinate& start,
    const RMDecision::PlaneCoordinate& end,
    const double& angle) const {
    nav_to_point_serially(start);
    rotate_to_angle(angle);
    move_to_point(end);
}

void DecisionTestAlpha::route_a() const {
    nav_to_point_serially(5.2, -2);
    rotate_to_angle(0);
    move_to_point(RMDecision::PlaneCoordinate(1.8, -2.7));
    move_to_point(RMDecision::PlaneCoordinate(1.8, -3.5));
    nav_to_point_serially(9.553, 2.817);
    if (get_current_coordinate().coincide_with(RMDecision::PlaneCoordinate(9.553, 2.817), 0.05) || true) {
        set_angular_velocity_debounce(4);
    }
    int t = 0;
    while (prism_.self->hp > 100) {
        rclcpp::Rate(5).sleep();
        if (t++ > 10) {
            nav_to_point(RMDecision::PlaneCoordinate::random_point(0.3) + get_current_coordinate());
            set_angular_velocity_debounce(4);
        }
    }
    nav_to_point(0, 0);
}

void DecisionTestAlpha::test_response(const std::string& instruction, const std::vector<float>& args) {
    auto it = test_funcs_.find(instruction);
    if (it == test_funcs_.end()) {
        test_display("Instruction [ %s ] undefined.\n", instruction.c_str());
        return;
    }

    test_funcs_[instruction](args);
}

void DecisionTestAlpha::nav_to_point_test(const std::vector<float>& args) {
    if (args.size() == 2) {
        test_display("Navigating to point: (%.3f, %.3f)\n", args[0], args[1]);
        nav_to_point(args[0], args[1]);
    }
}

void DecisionTestAlpha::rotate_to_angle_test(const std::vector<float>& args) {
    if (args.size() == 1) {
        test_display("Rotating to angle: %.3f\n", args[0]);
        rotate_to_angle(args[0]);
    }
}

void DecisionTestAlpha::set_angular_velocity_test(const std::vector<float>& args) {
    if (args.size() == 1) {
        test_display("Set angular velocity: %.3f\n", args[0]);
        set_angular_velocity(args[0]);
    }
}

void DecisionTestAlpha::set_linear_velocity_test(const std::vector<float>& args) {
    if (args.size() == 2) {
        test_display("Set linear velocity: (%.3f, %.3f)\n", args[0], args[1]);
        set_linear_velocity(RMDecision::PlaneCoordinate(args[0], args[1]));
    }
}

void DecisionTestAlpha::move_to_point_test(const std::vector<float>& args) {
    if (args.size() == 2) {
        test_display("Moving to point: (%.3f, %.3f)\n", args[0], args[1]);
        move_to_point(RMDecision::PlaneCoordinate(args[0], args[1]));
    }
}

void DecisionTestAlpha::set_gimbal_state_test(const std::vector<float>& args) {
    if (args.size() == 2) {
        test_display("Set gimbal state: pitch=%.3f, yaw=%.3f\n", args[0], args[1]);
        set_gimbal_state(args[0], args[1]);
    }
}

void DecisionTestAlpha::get_current_point_test(const std::vector<float>& args) {
    auto currentPoint = get_current_coordinate();
    test_display("Current point: (%.3f, %.3f)\n", currentPoint.x, currentPoint.y);
}

void DecisionTestAlpha::get_current_angle_test(const std::vector<float>& args) {
    test_display("Current angle: %.3f\n", get_current_angle());
}

void DecisionTestAlpha::set_linear_offset_test(const std::vector<float>& args) {
    if (args.size() == 2) {
        test_display("Set linear offset: (%.3f, %.3f)\n", args[0], args[1]);
        set_linear_offset(RMDecision::PlaneCoordinate(args[0], args[1]));
    }
}

void DecisionTestAlpha::set_angular_offset_test(const std::vector<float>& args) {
    if (args.size() == 1) {
        test_display("Set angular offset: %.3f\n", args[0]);
        set_angular_offset(args[0]);
    }
}

void DecisionTestAlpha::get_linear_offset_test(const std::vector<float>& args) {
    RMDecision::PlaneCoordinate offset = get_linear_offset();
    test_display("Linear offset: (%.3f, %.3f)\n", offset.x, offset.y);
}

void DecisionTestAlpha::get_angular_offset_test(const std::vector<float>& args) {
    double offset = get_angular_offset();
    test_display("Angular offset: %.3f\n", offset);
}

void DecisionTestAlpha::mark_origin_linear_test(const std::vector<float>& args) {
    if (args.size() == 0) {
        mark_origin_linear();
        RMDecision::PlaneCoordinate offset = get_linear_offset();
        test_display("Marked linear offset: (%.3f, %.3f)\n", offset.x, offset.y);
    }
}

void DecisionTestAlpha::mark_origin_angular_test(const std::vector<float>& args) {
    if (args.size() == 0) {
        mark_origin_angular();
        double offset = get_angular_offset();
        test_display("Marked angular offset: %.3f\n", offset);
    }
}

#include "rm_decision_macros/decision_node_regist_macro.hpp"
REGIST_DECISION_NODE(DecisionTestAlpha)