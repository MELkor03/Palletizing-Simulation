#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
    msg.position.z = 0.201;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Collision Objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(2);

  collision_objects[0].id = "floor";
  collision_objects[0].header.frame_id = "world";
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions = {4, 4, 0.02};
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = -0.01;
  collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

  collision_objects[1].id = "box";
  collision_objects[1].header.frame_id = "world";
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_objects[1].primitives[0].dimensions = {0.3, 0.4, 0.2};
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 1.0;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = 0.1;
  collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObjects(collision_objects);

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

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}