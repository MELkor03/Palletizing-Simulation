#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include "robot_motion.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_n_place_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pick_n_place_node");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set Poses
  geometry_msgs::msg::Pose approach_pick;
  approach_pick.orientation.w = 1.0;
  approach_pick.position.x = 1.0;
  approach_pick.position.y = 0.0;
  approach_pick.position.z = 1.01;

  geometry_msgs::msg::Pose pick_pose;
  pick_pose.orientation.w = 1.0;
  pick_pose.position.x = 1.0;
  pick_pose.position.y = 0.0;
  pick_pose.position.z = 0.701;

  geometry_msgs::msg::Pose approach_place;
  approach_place.orientation.z = -0.7068252;
  approach_place.orientation.w = 0.7073883;
  approach_place.position.x = 0.0;
  approach_place.position.y = -1.0;
  approach_place.position.z = 0.54;

  geometry_msgs::msg::Pose place_pose;
  place_pose.orientation.z = -0.7068252;
  place_pose.orientation.w = 0.7073883;
  place_pose.position.x = 0.0;
  place_pose.position.y = -1.0;
  place_pose.position.z = 0.246;

  // Approach object
  robot_motion::moveInCartesianPath(move_group_interface, approach_pick);

  // Move to pick up object
  robot_motion::moveInCartesianPath(move_group_interface, pick_pose);

  // Pick object
  rclcpp::sleep_for(std::chrono::seconds(1));
  move_group_interface.attachObject("Box_1", "wrist_3_link");
  auto attach_client = node->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");

  while (!attach_client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(node->get_logger(), "Waiting for service /ATTACHLINK...");
  }
  auto attach_request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
  attach_request->model1_name = "pr";
  attach_request->link1_name = "wrist_3_link";
  attach_request->model2_name = "small_box";
  attach_request->link2_name = "link";
  auto attach_future = attach_client->async_send_request(attach_request);
  if (rclcpp::spin_until_future_complete(node, attach_future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Object attached");
  } else {
      RCLCPP_ERROR(node->get_logger(), "Faild to attach object");
  }

  // Retreat
  robot_motion::moveInCartesianPath(move_group_interface, approach_pick);

  // Approach place position
  robot_motion::moveInCartesianPath(move_group_interface, approach_place);
  
  // Move to place
  robot_motion::moveInCartesianPath(move_group_interface, place_pose);

  // Place object
  rclcpp::sleep_for(std::chrono::seconds(1));
  move_group_interface.detachObject("Box_1");
  auto detach_client = node->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");
  while (!detach_client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(node->get_logger(), "Waiting for service /DETACHLINK...");
  }
  auto detach_request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
  detach_request->model1_name = "pr";
  detach_request->link1_name = "wrist_3_link";
  detach_request->model2_name = "small_box";
  detach_request->link2_name = "link";
  auto detach_future = detach_client->async_send_request(detach_request);
  if (rclcpp::spin_until_future_complete(node, detach_future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Obiect detached");
  } else {
      RCLCPP_ERROR(node->get_logger(), "Faild to detach object");
  }

  // Retreat
  robot_motion::moveInCartesianPath(move_group_interface, approach_place);

  // Return home
  robot_motion::moveToHome(move_group_interface);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}