#ifndef ROBOT_MOTION_HPP
#define ROBOT_MOTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

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

void moveInCartesianPath(moveit::planning_interface::MoveGroupInterface& mgi, const geometry_msgs::msg::Pose& target){

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    
    double success = mgi.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if(success != -1) { mgi.execute(trajectory); }

    mgi.setStartStateToCurrentState();
}

} // namespace robot_motion

#endif // ROBOT_MOTION_HPP