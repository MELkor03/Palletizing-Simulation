# Palletizing Simulation

This repository contains a palletizing robot simulation in ROS2 HUMBLE using MoveIt2 and Gazebo. It features the Universal Robots UR10e coupled with the Robotiq PowerPick10 vacuum gripper. Currently, the simulation is incomplete and only supports a basic pick-and-place application.

# Requirements

The simulation utilizes MoveIt2 and Gazebo. Additionally, it requires the UR robot driver and IFRA Cranfield's link attacher for Gazebo. Links to the required repositories:
 * <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble>
 * <https://github.com/IFRA-Cranfield/IFRA_LinkAttacher>

# Quick tutorial

To start the pick-and-place simulation, run the following command:
```
ros2 launch palletizing_robot_moveit_config gazebo_moveit.launch.py 
```

Wait for everything to initialize, then open a second terminal and run:

```
ros2 run palletizing_robot_nodes pick_n_place 
```

# Current Issues

 * Inconsistent trajectory generation
 * Spawning multiple objects over time
 * Camera integration
