#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include "robot_motion.hpp"
#include "palletizing_robot_interfaces/srv/palletization.hpp"

class PalletizationService : public rclcpp::Node {
public:
    PalletizationService() : Node("palletization_service_node"), mgi(std::make_shared<rclcpp::Node>("move_group_node"), "arm", nullptr, rclcpp::Duration(10, 0)){
        service_ = this->create_service<palletizing_robot_interfaces::srv::Palletization>(
            "Palletization",
            std::bind(&PalletizationService::palletization_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

        approach_pick.orientation.w = 1.0;
        approach_pick.position.x = 1.0;
        approach_pick.position.y = 0.0;
        approach_pick.position.z = 1.01;

        pick_pose.orientation.w = 1.0;
        pick_pose.position.x = 1.0;
        pick_pose.position.y = 0.0;
        pick_pose.position.z = 0.701;

        approach_place.orientation.z = -0.7068252;
        approach_place.orientation.w = 0.7073883;
        approach_place.position.x = -0.45;
        approach_place.position.y = -0.8;
        approach_place.position.z = 0.54;

        place_pose.orientation.z = -0.7068252;
        place_pose.orientation.w = 0.7073883;
        place_pose.position.x = -0.45;
        place_pose.position.y = -0.8;
        place_pose.position.z = 0.246;

        for(int z = 0; z<3; z++){
            for(int y = 0; y<3; y++){
                for(int x = 0; x<4; x++){
                    approach_place.position.x = -0.45 + x*0.3;
                    approach_place.position.y = -1.2 + y*0.2;
                    approach_place.position.z = 0.54 + z*0.1;

                    place_pose.position.x = -0.45 + x*0.3;
                    place_pose.position.y = -1.2 + y*0.2;
                    place_pose.position.z = 0.246 + z*0.1;

                    approach_place_points.push_back(approach_place);
                    place_pose_points.push_back(place_pose);
                }
            }
        }
    }
    ~PalletizationService()
    {
        rclcpp::shutdown();
    }
private:
    moveit::planning_interface::MoveGroupInterface mgi;
    rclcpp::Service<palletizing_robot_interfaces::srv::Palletization>::SharedPtr service_;
    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;

    // Poses
    geometry_msgs::msg::Pose approach_pick;
    geometry_msgs::msg::Pose pick_pose;
    std::vector<geometry_msgs::msg::Pose> approach_place_points;
    std::vector<geometry_msgs::msg::Pose> place_pose_points;
    geometry_msgs::msg::Pose approach_place;
    geometry_msgs::msg::Pose place_pose;

    void palletization_callback(
        const std::shared_ptr<palletizing_robot_interfaces::srv::Palletization::Request> request, 
        std::shared_ptr<palletizing_robot_interfaces::srv::Palletization::Response> response)
    {
        //approach_place.position.x = -0.45 + static_cast<double>(request->counter) * 0.3;
        //place_pose.position.x = -0.45 + static_cast<double>(request->counter) * 0.3;

        robot_motion::moveInCartesianPath(mgi, approach_pick);
        robot_motion::moveInCartesianPath(mgi, pick_pose);

        // Pick object
        rclcpp::sleep_for(std::chrono::seconds(1));
        mgi.attachObject(request->moveit_name, "wrist_3_link");
        if (!attach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service /ATTACHLINK is not available");
            return;
        }
        auto attach_request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
        attach_request->model1_name = "pr";
        attach_request->link1_name = "wrist_3_link";
        attach_request->model2_name = request->gazebo_name;
        attach_request->link2_name = "link";
        auto attach_future = attach_client_->async_send_request(attach_request);
        //rclcpp::spin_until_future_complete(shared_from_this(), attach_future);

        robot_motion::moveInCartesianPath(mgi, approach_pick);
        robot_motion::moveInCartesianPath(mgi, approach_place_points[request->counter]);
        robot_motion::moveInCartesianPath(mgi, place_pose_points[request->counter]);

        // Place object
        rclcpp::sleep_for(std::chrono::seconds(1));
        mgi.detachObject(request->moveit_name);
        if (!detach_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service /DETACHLINK is not available");
            return;
        }
        auto detach_request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
        detach_request->model1_name = "pr";
        detach_request->link1_name = "wrist_3_link";
        detach_request->model2_name = request->gazebo_name;
        detach_request->link2_name = "link";
        auto detach_future = detach_client_->async_send_request(detach_request);
        //rclcpp::spin_until_future_complete(shared_from_this(), detach_future);

        robot_motion::moveInCartesianPath(mgi, approach_place_points[request->counter]);
        response->success = true;
        response->log = "Success";   
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PalletizationService>();
    //node->initializeMoveGroup();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
}