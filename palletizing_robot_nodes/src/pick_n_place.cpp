#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <linkattacher_msgs/srv/attach_link.hpp>        // INCLUDE ROS2 SERVICE.
#include <linkattacher_msgs/srv/detach_link.hpp>        // INCLUDE ROS2 SERVICE.

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "moveit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_node");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Set a pick and place Pose
  auto const pick_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 1.0;
    msg.position.y = 0.0;
    msg.position.z = 0.701;
    return msg;
  }();

  auto const place_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.z = -0.7068252;
    msg.orientation.w = 0.7073883;
    msg.position.x = 0.0;
    msg.position.y = -1.0;
    msg.position.z = 0.245;
    return msg;
  }();

  move_group_interface.setPoseTarget(pick_pose);

  // Create a plan to that target pose
  auto const [success_1, plan_1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  move_group_interface.execute(plan_1);
  
  // Link attach clinet
  if(success_1){
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
  }

  move_group_interface.setStartStateToCurrentState();
  move_group_interface.setPoseTarget(place_pose);

  // Create a plan to that target pose
  auto const [success_2, plan_2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  move_group_interface.execute(plan_2);

  if(success_2){
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
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}