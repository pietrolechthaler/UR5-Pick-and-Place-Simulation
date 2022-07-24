ur5
===

[![Build Status](https://travis-ci.org/utecrobotics/ur5.svg?branch=master)](https://travis-ci.org/utecrobotics/ur5)

ROS packages for the UR5 Robot with a Robotiq gripper

## Installation

This will assume that you already have a catkin workspace. Go to the source directory of the workspace
  ```
  $ roscd; cd ../src
  ```
Clone this and the gripper (robotiq) repositories
  ```
  $ git clone https://github.com/utecrobotics/ur5
  $ git clone https://github.com/utecrobotics/robotiq
  ```
Build using catkin_make
  ```
  $ cd ..
  $ catkin_make
  ```

## Visualization of UR5 in RViz

To visualize the model of the robot with a gripper, launch the following:
  ```
  $ roslaunch ur5_description display_with_gripper.launch
  ```
You can then use the sliders to change the joint values and the gripper values.

## Simulation in Gazebo

To simulate the robot launch the following:
  ```
  $ roslaunch ur5_gazebo ur5_cubes.launch
  ```
You should see something similar to the following picture:
![Image of UR5](ur5_images/ur5_cubes.png)

By default the simulation starts paused. Unpause the simulation. You can then send commands to the
joints or to the gripper.

The following is an example of an action client to change the gripper configuration. Open a new
terminal, and then execute:
  ```
  $ rosrun ur5_gazebo send_gripper.py --value 0.5
  ```
where the value is a float between 0.0 (closed) and 0.8 (open).

An example of sending joints values to the robot can be executed as follows:
  ```
  $ rosrun ur5_gazebo send_joints.py
  ```
To change the values of the joints, the file `send_joints.py` must be modified.
