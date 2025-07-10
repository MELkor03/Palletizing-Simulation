# Palletizing Simulation

This repository contains a palletizing robot simulation in ROS2 HUMBLE using MoveIt2 and Gazebo. It features the Universal Robots UR10e coupled with the Robotiq PowerPick10 vacuum gripper.

# Requirements

The simulation utilizes MoveIt2 and Gazebo. Additionally, it requires the UR robot driver and IFRA Cranfield's link attacher for Gazebo. Links to the required repositories:
 * <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble>
 * <https://github.com/IFRA-Cranfield/IFRA_LinkAttacher>

# Quick tutorial

To start the simulation, run the following command:
```
ros2 launch palletizing_robot_moveit_config gazebo_moveit.launch.py 
```

Wait for everything to initialize, then open a second terminal and launch:

```
ros2 launch palletizing_robot_nodes palletizing.launch.py 
```

# Current Issues

 * Sharp trajectory corners
