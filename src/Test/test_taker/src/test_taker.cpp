#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "test_taker_interfaces/msg/test_args.hpp"

class TestTaker : public rclcpp::Node {
public:
    TestTaker() : Node("test_taker") {
        test_pub_ = this->create_publisher<test_taker_interfaces::msg::TestArgs>("test_command", 10);
        feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
            "test_feedback", 10, std::bind(&TestTaker::feedback_callback, this, std::placeholders::_1));
        init_path();
    }

    void set_input_param(const std::string& input) {
        std::istringstream iss(input);
        std::string instruction;
        iss >> instruction;

        if (instruction == "MARK") {
            std::string mark;
            iss >> mark;
            write("# " + mark);
            return;
        }

        write(">>> " + input);
        std::vector<float> args;
        float num;
        while (iss >> num) {
            args.push_back(num);
        }

        test_taker_interfaces::msg::TestArgs test_msg;
        test_msg.instruction = instruction;
        test_msg.args = args;
        test_pub_->publish(test_msg);
    }

private:
    void feedback_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::cout << "\r" << msg->data << std::flush;
        write("<<< " + msg->data);
    }

    void init_path() {
        log_folder_path_ = "./test_record";
        if (!std::filesystem::exists(log_folder_path_)) {
            std::filesystem::create_directory(log_folder_path_);
        }

        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H:%M:%S", std::localtime(&now_c));
        log_time_ = buffer;

        log_file_path_ = log_folder_path_ + "/test_logs_" + log_time_ + ".txt";

        log_file_.open(log_file_path_, std::ios::app);
        if (log_file_.is_open()) {
            log_file_ << "\n\n[" << log_time_ << "]" << std::endl;
            log_file_.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", log_file_path_.c_str());
        }
    }

    void write(const std::string& text) {
        log_file_.open(log_file_path_, std::ios::app);
        if (log_file_.is_open()) {
            log_file_ << text << std::endl;
            log_file_.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file.");
        }
    }

    rclcpp::Publisher<test_taker_interfaces::msg::TestArgs>::SharedPtr test_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_sub_;

    std::string log_folder_path_;
    std::string log_file_path_;
    std::ofstream log_file_;
    std::string log_time_;
};

void read_input(rclcpp::Node::SharedPtr node) {
    auto test_taker = std::dynamic_pointer_cast<TestTaker>(node);
    std::string input;
    while (rclcpp::ok()) {
        std::getline(std::cin, input);
        if (input == "quit") {
            RCLCPP_INFO(test_taker->get_logger(), "Parameter listening stopped, enter <Ctrl+C> to shutdown.");
            break;
        }
        if (!input.empty() && test_taker) {
            test_taker->set_input_param(input);
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestTaker>();
    auto input_future = std::async(std::launch::async, read_input, node);
    rclcpp::spin(node);
    input_future.wait();
    rclcpp::shutdown();
    return 0;
}