# UR3 Smart Grasping System

Gazebo-based compliant grasping simulation for UR3 robot arm and Robotiq gripper.

## Current milestone

- ROS Noetic + Gazebo simulation compiled successfully.
- UR3 + gripper + table + cube scene launched successfully.
- Low-level compliant grasping script tested.
- Upper-level fault-tolerant state machine tested.
- Force/position tracking plots and ROS probe logs saved.

## Key ROS names

- Arm controller: scaled_pos_joint_traj_controller
- Gripper controller: gripper_controller
- Gripper joint: finger_joint
- Filtered wrench topic: /wrench/filtered

## Main scripts

- scripts/compliance_controller_example.py
- scripts/smart_grasp_state_machine.py

## Run commands

Start Gazebo:
source /opt/ros/noetic/setup.bash
source ~/ur3_sim/devel/setup.bash
roslaunch ur_gripper_gazebo ur3_cubes_example.launch grasp_plugin:=1

Start FT filter:
source /opt/ros/noetic/setup.bash
source ~/ur3_sim/devel/setup.bash
rosrun ur_control ft_filter.py -t wrench -ot wrench/filtered -z

Run state machine:
source /opt/ros/noetic/setup.bash
source ~/ur3_sim/devel/setup.bash
rosrun ur_control smart_grasp_state_machine.py _table_side:=0.02 _table_duration:=40 _target_force_z:=5 _max_retries:=1
