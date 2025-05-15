#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include "robot_motion.hpp"
#include "pattern_generation.hpp"
#include "palletizing_robot_interfaces/srv/palletization.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PalletizationService : public rclcpp::Node {
public:
    PalletizationService() : Node("palletization_service_node"), mgi(createMoveGroupNode(), "arm", nullptr, rclcpp::Duration(10, 0)){
        service_ = this->create_service<palletizing_robot_interfaces::srv::Palletization>(
            "Palletization",
            std::bind(&PalletizationService::palletization_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        attach_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client_ = this->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

        approach_pick.orientation.w = 1.0;
        approach_pick.position.x = 1.0;
        approach_pick.position.y = 0.0;
        approach_pick.position.z = 0.9;

        pick_pose.orientation.w = 1.0;
        pick_pose.position.x = 1.0;
        pick_pose.position.y = 0.0;
        pick_pose.position.z = 0.601;

        pg = std::make_unique<pattern_generation::PatternGenerator>(approach_pick, "Optimized");
        
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
    geometry_msgs::msg::Pose middle_point;
    geometry_msgs::msg::Pose pick_pose;
    std::vector<geometry_msgs::msg::Pose> approach_place_points;
    std::vector<geometry_msgs::msg::Pose> middle_points;
    std::vector<geometry_msgs::msg::Pose> place_pose_points;
    std::unique_ptr<pattern_generation::PatternGenerator> pg;
    // pattern_generation::PatternGenerator pg(approach_pick, "Optimized");
    //geometry_msgs::msg::Pose approach_place;
    //geometry_msgs::msg::Pose place_pose;
    std::shared_ptr<rclcpp::Node> createMoveGroupNode(){
        rclcpp::NodeOptions options;
        options.parameter_overrides().push_back({"use_sim_time", true});
        return std::make_shared<rclcpp::Node>("move_group_node", options);
    }


    void palletization_callback(
        const std::shared_ptr<palletizing_robot_interfaces::srv::Palletization::Request> request, 
        std::shared_ptr<palletizing_robot_interfaces::srv::Palletization::Response> response)
    {
        //approach_place.position.x = -0.45 + static_cast<double>(request->counter) * 0.3;
        //place_pose.position.x = -0.45 + static_cast<double>(request->counter) * 0.3;
        if(request->counter == 0){ robot_motion::moveInCartesianPath(mgi, pick_pose, approach_pick); }
        else{robot_motion::moveInCartesianPath(mgi, pick_pose, pg->getMiddlePoint(request->counter - 1), approach_pick);}
        //robot_motion::moveInCartesianPath(mgi, pick_pose);

        // Pick object
        rclcpp::sleep_for(std::chrono::milliseconds(500));
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
        
        //robot_motion::moveInCartesianPath(mgi, approach_pick); 
        //robot_motion::moveInCartesianPath(mgi, pg->getApproachPlacePoint(request->counter), pg->getMiddlePoint(request->counter));
        robot_motion::moveInCartesianPath(mgi, pg->getPlacePosePoint(request->counter), approach_pick, 
            pg->getMiddlePoint(request->counter), pg->getApproachPlacePoint(request->counter) );
        
        // Place object
        rclcpp::sleep_for(std::chrono::milliseconds(200));
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
 
        robot_motion::moveInCartesianPath(mgi, pg->getApproachPlacePoint(request->counter));
        response->log = "Success";   

        if(request->counter+1 == pg->getAmountPerLayer()){
            pg->generateNewLayer();
            // rclcpp::sleep_for(std::chrono::seconds(10)); 
        }
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