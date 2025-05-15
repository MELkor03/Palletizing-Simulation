#ifndef ROBOT_MOTION_HPP
#define ROBOT_MOTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>

namespace robot_motion {

void moveToPoint(moveit::planning_interface::MoveGroupInterface& mgi, const geometry_msgs::msg::Pose& target){
    mgi.setPoseTarget(target);
    
    auto const [success, plan] = [&mgi]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(mgi.plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    if(success) { mgi.execute(plan); }

    mgi.setStartStateToCurrentState();
}

void moveToHome(moveit::planning_interface::MoveGroupInterface& mgi){
    // important to have a defined "home" group state
    mgi.setNamedTarget("home");

    auto const [success, plan] = [&mgi]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(mgi.plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    if(success) { mgi.execute(plan); }

    mgi.setStartStateToCurrentState();
}

void moveInCartesianPath(moveit::planning_interface::MoveGroupInterface& mgi,
    const geometry_msgs::msg::Pose& target,
    const std::optional<geometry_msgs::msg::Pose>& extra_pose_1 = std::nullopt,
    const std::optional<geometry_msgs::msg::Pose>& extra_pose_2 = std::nullopt,
    const std::optional<geometry_msgs::msg::Pose>& extra_pose_3 = std::nullopt)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;

    if (extra_pose_1.has_value()) waypoints.push_back(extra_pose_1.value());
    if (extra_pose_2.has_value()) waypoints.push_back(extra_pose_2.value());
    if (extra_pose_3.has_value()) waypoints.push_back(extra_pose_3.value());

    waypoints.push_back(target);

    mgi.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;

    double success_fraction = mgi.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);

    RCLCPP_INFO(rclcpp::get_logger("move_group_interface"),
    "Computed %.2f%% of Cartesian path.", success_fraction * 100.0);

    if (!trajectory.joint_trajectory.points.empty()) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto result = mgi.execute(plan);
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger("move_group_interface"),
            "Execution stopped! Possibly due to a collision or invalid motion.");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "Trajectory executed successfully.");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("move_group_interface"),
        "Cartesian path incomplete. Only %.2f%% was planned.",
        success_fraction * 100.0);
    }

    mgi.setStartStateToCurrentState();
}

} // namespace robot_motion

#endif // ROBOT_MOTION_HPP