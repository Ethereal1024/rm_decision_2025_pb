#include "decision_beta/decision_beta.hpp"
using namespace RMDecision;

void DecisionBeta::rotate_to_angle(const double& targetAngle) const {
    const double KP = 0.8;
    const double KI = 0.15;
    const double KD = 0.02;

    const double TOLARANCE = 0.01;
    const double RATE = 100;
    const double TIME_LIMIT = 4;  // 单位：秒

    double previousError = 0.0;  // 上一次的误差
    double integral = 0.0;       // 误差的积分项
    rclcpp::Time start_time = this->now();
    rclcpp::Time lastTime = start_time;

    while (rclcpp::ok()) {
        if (abort_)
            return;

        auto currentTime = this->now();
        double deltaTime = (currentTime - lastTime).seconds();  // 动态计算时间间隔
        lastTime = currentTime;

        double currentAngle = get_current_angle();
        // RCLCPP_INFO(this->get_logger(), "Current angle: %.3f", currentAngle);
        double error = targetAngle - currentAngle;

        while (error > M_PI) {
            error -= 2 * M_PI;
        }
        while (error < -M_PI) {
            error += 2 * M_PI;
        }
        // RCLCPP_INFO(this->get_logger(), "Error value: %.3f", error);

        if (std::abs(error) < TOLARANCE || (this->now() - start_time).seconds() > TIME_LIMIT) {
            set_angular_velocity(0.0);
            break;
        }

        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double angularV = KP * error + KI * integral + KD * derivative;
        set_angular_velocity(angularV);

        previousError = error;

        rclcpp::Rate(RATE).sleep();
    }
}

void DecisionBeta::rotate_to_vec(const PlaneCoordinate& vec) const {
    double angle = std::atan2(vec.y, vec.x);
    rotate_to_angle(angle);
}

void DecisionBeta::move_to_point(const PlaneCoordinate& targetPoint) const {
    const double KP = 0.8;
    const double KI = 0.03;
    const double KD = 0.001;

    const double TOLARANCE = 0.08;
    const double RATE = 100;
    const double TIME_LIMIT = 8;  // 单位：秒

    auto previousError = PlaneCoordinate(0.0, 0.0);  // 上一次的误差
    auto integral = PlaneCoordinate(0.0, 0.0);       // 误差的积分项
    rclcpp::Time start_time = this->now();
    rclcpp::Time lastTime = start_time;

    while (rclcpp::ok()) {
        if (abort_)
            return;
        auto currentTime = this->now();
        double deltaTime = (currentTime - lastTime).seconds();  // 动态计算时间间隔
        lastTime = currentTime;

        PlaneCoordinate currentCoordinate = get_current_coordinate();
        // RCLCPP_INFO(this->get_logger(), "Current coordinate: (%.3f, %.3f)", currentCoordinate.x, currentCoordinate.y);
        PlaneCoordinate error = targetPoint - currentCoordinate;

        // RCLCPP_INFO(this->get_logger(), "Error value: (%.3f, %.3f)", error.x, error.y);

        if (currentCoordinate.coincide_with(targetPoint, TOLARANCE) || (this->now() - start_time).seconds() > TIME_LIMIT) {
            set_linear_velocity(PlaneCoordinate(0.0, 0.0));
            break;
        }

        integral += error * deltaTime;
        PlaneCoordinate derivative = (error - previousError) / deltaTime;

        PlaneCoordinate mapV = KP * error + KI * integral + KD * derivative;
        double theta = get_current_angle();
        PlaneCoordinate linearV = PlaneCoordinate(
            mapV.x * cos(theta) + mapV.y * sin(theta), -mapV.x * sin(theta) + mapV.y * cos(theta));
        set_linear_velocity(linearV);

        previousError = error;

        rclcpp::Rate(RATE).sleep();
    }
}

void DecisionBeta::nav_to_point_serially(const PlaneCoordinate& targetPoint) const {
    const double TOLARANCE = 0.1;
    const double RATE = 100;
    const double TIME_LIMIT = 10;

    rclcpp::Time start_time = this->now();

    while (rclcpp::ok()) {
        if (abort_)
            return;
        nav_to_point(targetPoint);
        PlaneCoordinate currentCoordinate = get_current_coordinate();
        if (currentCoordinate.coincide_with(targetPoint, TOLARANCE) || (this->now() - start_time).seconds() > TIME_LIMIT) {
            set_linear_velocity(PlaneCoordinate(0.0, 0.0));
            return;
        }
        rclcpp::Rate(RATE).sleep();
    }
}

void DecisionBeta::nav_to_point_serially(const double& x, const double& y) const {
    nav_to_point_serially(PlaneCoordinate(x, y));
}

void DecisionBeta::abort() {
    abort_.store(true);
}

void DecisionBeta::reset() {
    abort_.store(false);
}