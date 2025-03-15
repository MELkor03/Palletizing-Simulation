#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "collision_environment.hpp"

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

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 1.0;
    msg.position.y = 0.0;
    msg.position.z = 0.701;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Collision Objects
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //collision_environment::generateCollisionEnviroment(planning_scene_interface);
  

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // psi, id, pose, dimensions
  //collision_environment::addMoveitBox(planning_scene_interface, "Box_1", {1, 0, 0.65}, {0.2, 0.3, 0.1});

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}