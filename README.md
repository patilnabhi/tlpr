# Two Link Planar Robot (TLPR)

## About

A ROS tutorial developed as part of *ME495 - Embedded Systems in Robotics* course in Northwestern University.

To launch (in a ROS workspace): 
```
roslaunch two_link_planar_robot display.launch
```

May also launch using various parameters:
```
roslaunch two_link_planar_robot display.launch show_rviz:=True circle_length:=75 rate_update:=50 rate_animate:=20
```

## Goal:

**To design a Two Link Planar Robot (TLPR) whose end effector traces and draws a circle**

## Project Objectives

* Develop an URDF for TLPR 
* Use **inverse kinematics** to determine the joint angles given following trajectory:

<img src="/bin/trajectory.jpg" align="middle" width="300"> 

* Move the end effector by publishing sensor_msgs/JointState` message
* Animate the trajectory by repeatedly using `lookupTransform` of `tf.TransformListener` and publishing a `visualization_msgs/Marker` to visualize path of the end effector

## Package Contents

This ROS package contains the following files:

* **display.launch:** A launch file to launch the following 2 nodes and a `rviz` configuration file (.rviz). The launch file also takes in below parameters:
	* frequency (rate) for publishing messages - two args: `rate_joint:=` and `rate_animate:=` (default=**50** and **=20** respectively)
	* adjustable time periods of circle drawn: `circle_length:=` (default=**75**)
	* option to show rviz: `show_rviz:=` (default=**True**) 

* **update_joint_state.py node:** This uses inverse kinematics to get the joint angles from the given trajectory and publishes the angles through joint_state_publisher to make the end effector move in circle <sup>[1]</sup>

* **animate_path.py node:** This animates the trajectory and visualizes the path of end effector

![TLPR gif](/bin/tlpr.gif)

[1] Inverse kinematics formula adapted from http://www.hessmer.org/uploads/RobotArm/Inverse%2520Kinematics%2520for%2520Robot%2520Arm.pdf





