<p align="center">
  <h2 align="center">UR5 Pick and Place Simulation in Ros/Gazebo</h2>

  
</p>
<br>

<img src="https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/main.png">

## Table of contents
- [Description](#description)
- [Requirement](#requirements)
- [Folder](#folder)
- [Setup](#setup)
- [Usage](#usage)
- [Contributors](#contributors)

### Description
This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a Xbox Kinect cam to detect eleven types of Lego Bricks, and publish its position and angolation. 

The goals of this project are:
- simulate the iteration of a UR5 robot with Lego bricks
- The robotic arm must be able to move a block from position A to B and construct a castle by assembling different bricks

<img src="https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/blob/main/intro.gif">

### Folder
```
UR5-Pick-and-Place-Simulation/catkin_ws/
├── levelManager
├── vision
├── motion_planning
├── gazebo_ros_link_attacher
├── robot
```
- `levelManager:` the task of this package is to launch the world and spawn the different bricks
- `vision:` the task of this package is to recognize the object type and orientation of the bricks
- `motion_planning:` the task is to move the robot and pick and place the lego
- `gazebo_ros_link_attacher:` A gazebo plugin definable from URDF to inform a client of a collision with an object
- `robot:` the task is to define the robot model with appropriate PID settings


### Requirements

For running each sample code:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Yolov5` https://github.com/ultralytics/yolov5
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/

### Setup

After installing the libraries needed to run the project. Clone this repo:
```
git clone https://github.com/pietrolechthaler/UR5-Pick-and-Place-Simulation/
```

Setup the project:
```
cd UR5-Pick-and-Place-Simulation/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
echo "source $PWD/devel/setup.bash" >> $HOME/.bashrc
```

Clone and install [YoloV5](https://github.com/ultralytics/yolov5):
```
cd ~
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip3 install -r requirements.txt
```
### Usage

Launch the world
```
roslaunch levelManager lego_world.launch
```
Choose the level (from 1 to 4):
```
rosrun levelManager levelManager.py -l [level]
```
Start the kinematics process
```
rosrun motion_planning motion_planning.py
```
Start the localization process
```
rosrun vision vision.py -show
```
- `-show` : show the results of the recognition and localization process with an image

### Contributors

| Name                 | Github                               |
|----------------------|--------------------------------------|
| Davice Cerpelloni    | https://github.com/davidecerpelloni  |
| Leonardo Collizzolli | https://github.com/leocolliz         |
| Pietro Lechthaler    | https://github.com/pietrolechthaler  |
| Stefano Rizzi        | https://github.com/StefanoRizzi      |
