#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include <fstream>
#include <chrono>
#include <string>
#include <memory>

class PoseLoggerNode : public rclcpp::Node
{
public:
    PoseLoggerNode()
    : Node("pose_logger_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        start_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();

        // Ścieżka pliku logu
        std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : ".";
        file_path_ = home_dir + "/pose_log.csv";

        file_.open(file_path_);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Nie można otworzyć pliku do zapisu!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Logger uruchomiony. Zapis do: %s", file_path_.c_str());

        file_ << "time,x,y,z\n";

        // Timer 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PoseLoggerNode::logPose, this));
    }

    ~PoseLoggerNode()
    {
        if (file_.is_open()) {
            file_.close();
            RCLCPP_INFO(this->get_logger(), "Plik zamknięty.");
        }
    }

private:
    void logPose()
    {
        std::string parent_frame = "world";
        std::string child_frame = "end_effector";

        try {
            auto transform = tf_buffer_.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);

            rclcpp::Time now = rclcpp::Clock(RCL_STEADY_TIME).now();
            double elapsed = (now - start_time_).seconds();

            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;
            double z = transform.transform.translation.z;

            file_ << elapsed << "," << x << "," << y << "," << z << "\n";

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Transformacja niedostępna: %s", ex.what());
        }
    }

    rclcpp::Time start_time_;
    std::ofstream file_;
    std::string file_path_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


