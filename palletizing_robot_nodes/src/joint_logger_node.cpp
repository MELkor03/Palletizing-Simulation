#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <fstream>
#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <filesystem>

class JointLoggerNode : public rclcpp::Node
{
public:
    JointLoggerNode()
    : Node("joint_logger_node")
    {
        start_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();

        // Ustaw ścieżkę pliku
        std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : ".";
        file_path_ = home_dir + "/joint_states_log.csv";

        // Otwórz plik
        file_.open(file_path_);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Nie można otworzyć pliku do zapisu!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "JointLoggerNode uruchomiony. Logowanie do: %s", file_path_.c_str());

        // Subskrypcja na /joint_states
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointLoggerNode::jointStateCallback, this, std::placeholders::_1)
        );
    }

    ~JointLoggerNode()
    {
        if (file_.is_open()) {
            file_.close();
            RCLCPP_INFO(this->get_logger(), "Plik zamknięty.");
        }
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
        double elapsed = (now - start_time_).seconds();


        // Nagłówek (raz)
        if (!header_written_ && !msg->name.empty()) {
            file_ << "time";
            for (const auto &name : msg->name) {
                file_ << "," << name;
            }
            file_ << "\n";
            header_written_ = true;
        }

        // Dane
        file_ << elapsed;
        for (const auto &pos : msg->position) {
            file_ << "," << pos;
        }
        file_ << "\n";
    }

    rclcpp::Time start_time_;
    std::ofstream file_;
    std::string file_path_;
    bool header_written_ = false;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
