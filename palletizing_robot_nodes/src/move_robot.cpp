#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "collision_environment.hpp"
#include "robot_motion.hpp"

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
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // collision_environment::generateCollisionEnviroment(planning_scene_interface);
  
  geometry_msgs::msg::Pose target_pose_1;
  target_pose_1.orientation.w = 1.0;
  target_pose_1.position.x = -0.5;
  target_pose_1.position.y = -1.15;
  target_pose_1.position.z = 0.244;
  target_pose_1.orientation.z = -1.0;
  target_pose_1.orientation.w = 0.0;

  robot_motion::moveInCartesianPath(move_group_interface, target_pose_1);

  // psi, id, pose, dimensions
  //collision_environment::addMoveitBox(planning_scene_interface, "Box_1", {1, 0, 0.65}, {0.2, 0.3, 0.1});

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}