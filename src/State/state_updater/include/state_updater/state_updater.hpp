#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "state_base/state_base.hpp"

class HpUpdateHandler
    : public UpdateHandler<pb_rm_interfaces::msg::GameRobotHP> {
public:
    using UpdateHandler::UpdateHandler;

    std::string topic() const override {
        return "referee/all_robot_hp";
    }

    void update_chessboard(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg) const override {
        (*chessboard_ptr_->robots)["R1"]->hp = msg->red_1_robot_hp;
        (*chessboard_ptr_->robots)["R2"]->hp = msg->red_2_robot_hp;
        (*chessboard_ptr_->robots)["R3"]->hp = msg->red_3_robot_hp;
        (*chessboard_ptr_->robots)["R4"]->hp = msg->red_4_robot_hp;
        (*chessboard_ptr_->robots)["R7"]->hp = msg->red_7_robot_hp;

        (*chessboard_ptr_->robots)["B1"]->hp = msg->blue_1_robot_hp;
        (*chessboard_ptr_->robots)["B2"]->hp = msg->blue_2_robot_hp;
        (*chessboard_ptr_->robots)["B3"]->hp = msg->blue_3_robot_hp;
        (*chessboard_ptr_->robots)["B4"]->hp = msg->blue_4_robot_hp;
        (*chessboard_ptr_->robots)["B7"]->hp = msg->blue_7_robot_hp;

        (*chessboard_ptr_->architectures)["Red_Outpost"]->hp = msg->red_outpost_hp;
        (*chessboard_ptr_->architectures)["Blue_Outpost"]->hp = msg->blue_outpost_hp;

        (*chessboard_ptr_->architectures)["Red_Base"]->hp = msg->red_base_hp;
        (*chessboard_ptr_->architectures)["Blue_Base"]->hp = msg->blue_base_hp;

        chessboard_ptr_->timestamp = clock_->now();
    }

    void update_prism(const pb_rm_interfaces::msg::GameRobotHP::SharedPtr msg) const override {
        prism_ptr_->self->hp = chessboard_ptr_->friend_robot(prism_ptr_->self->id)->hp;
        prism_ptr_->track->hp = chessboard_ptr_->enemy_robot(prism_ptr_->track->id)->hp;
    }
};

class GameStatusUpdateHandler
    : public UpdateHandler<pb_rm_interfaces::msg::GameStatus> {
public:
    using UpdateHandler::UpdateHandler;

    std::string topic() const override {
        return "referee/game_status";
    }

    void update_chessboard(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) const override {
        return;
    }

    void update_prism(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg) const override {
        prism_ptr_->game->game_start = (msg->game_progress == msg->RUNNING);
    }
};

class RobotStatusUpdateHandler
    : public UpdateHandler<pb_rm_interfaces::msg::RobotStatus> {
public:
    using UpdateHandler::UpdateHandler;

    std::string topic() const override {
        return "referee/robot_status";
    }

    void update_chessboard(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) const override {
        std::shared_ptr<RMDecision::Robot> robot = chessboard_ptr_->friend_robot(msg->robot_id);
        robot->hp = msg->current_hp;
        robot->level = msg->robot_level;

        robot->pose.header.frame_id = "map";
        robot->pose.header.stamp = clock_->now();
        robot->pose.pose = msg->robot_pos;

        robot->missing = false;

        chessboard_ptr_->timestamp = clock_->now();
    }

    void update_prism(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg) const override {
        if (msg->robot_id != prism_ptr_->self->id)
            return;
        prism_ptr_->self->hp = msg->current_hp;
        prism_ptr_->self->pose.header.frame_id = "map";
        prism_ptr_->self->pose.header.stamp = clock_->now();
        prism_ptr_->self->pose.pose = msg->robot_pos;
    }
};

class RobotPositionsUpdateHandler
    : public UpdateHandler<pb_rm_interfaces::msg::GroundRobotPosition> {
public:
    using UpdateHandler::UpdateHandler;

    std::string topic() const override {
        return "referee/ground_robot_position";
    }

    void update_chessboard(const pb_rm_interfaces::msg::GroundRobotPosition::SharedPtr msg) const override {
        // chessboard_ptr_->friend_robot(1)->pose =
    }
};