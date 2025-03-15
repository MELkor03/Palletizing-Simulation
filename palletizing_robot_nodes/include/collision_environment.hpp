#ifndef COLLISION_ENVIRONMENT_HPP
#define COLLISION_ENVIRONMENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace collision_environment {

void generateCollisionEnviroment(moveit::planning_interface::PlanningSceneInterface& psi){
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(4);

    // Floor
    collision_objects[0].id = "floor";
    collision_objects[0].header.frame_id = "world";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[0].primitives[0].dimensions = {6, 6, 0.02};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0.0;
    collision_objects[0].primitive_poses[0].position.z = -0.01;
    collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Euro pallet A
    collision_objects[1].id = "euro_pallet_A";
    collision_objects[1].header.frame_id = "world";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[0].dimensions = {1.2, 0.8, 0.144};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.0;
    collision_objects[1].primitive_poses[0].position.y = 1.0;
    collision_objects[1].primitive_poses[0].position.z = 0.072;
    collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Euro pallet B
    collision_objects[2].id = "euro_pallet_B";
    collision_objects[2].header.frame_id = "world";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[2].primitives[0].dimensions = {1.2, 0.8, 0.144};
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.0;
    collision_objects[2].primitive_poses[0].position.y = -1.0;
    collision_objects[2].primitive_poses[0].position.z = 0.072;
    collision_objects[2].operation = moveit_msgs::msg::CollisionObject::ADD;

    // Conveyor belt
    collision_objects[3].id = "conveyor_belt";
    collision_objects[3].header.frame_id = "world";
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[3].primitives[0].dimensions = {2, 0.6, 0.6};
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 1.7;
    collision_objects[3].primitive_poses[0].position.y = 0;
    collision_objects[3].primitive_poses[0].position.z = 0.3;
    collision_objects[3].operation = moveit_msgs::msg::CollisionObject::ADD;

    psi.applyCollisionObjects(collision_objects);
}

void addMoveitBox(moveit::planning_interface::PlanningSceneInterface& psi, const std::string& id, 
                  const std::array<double, 3>& position, const std::array<double, 3>& dimensions)
{
    moveit_msgs::msg::CollisionObject box;

    box.id = id;
    box.header.frame_id = "world";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {dimensions[0], dimensions[1], dimensions[2]};
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(pose);
    box.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(box);
}
    
} // namespace collision_enviroment

#endif // COLLISION_ENVIRONMENT_HPP