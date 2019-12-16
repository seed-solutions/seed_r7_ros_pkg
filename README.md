# seed_r7_ros_pkg  
seed-noid meta package

## Continuous Integration Status
service    | Kinetic | Melodic |
---------- | ------- | ------- |
Travis     | ![Build Status](https://travis-ci.com/seed-solutions/seed_r7_ros_pkg.svg?branch=master) | ![Build Status](https://travis-ci.com/seed-solutions/seed_r7_ros_pkg.svg?branch=master)
## How to install
### 1. From Debian
```
  sudo apt-get update
  sudo apt-get install ros-{distro}-seed-r7-ros-pkg
```

### 2. From Source
In your catkin_ws/src,
```
  sudo apt-get install ros-{distro}-seed-smartactuator-sdk
  source /opt/ros/{distro}/setup.bash
  git clone https://github.com/seed-solutions/seed_r7_ros_pkg.git
  catkin build seed_r7_ros_pkg
  source {your-workspace}/devel/setup.bash
```
## How to run
### Bring up robot_control(with real robot and Simulation on rviz)

``roslaunch seed_r7_bringup seed_r7_bringup.launch`` 

## Without real robot (Simulation on rviz)

``roslaunch seed_r7_navigation wheel_with_dummy.launch``

``rosrun  rviz rviz ``

![sample](https://i.imgur.com/ffLGv19.png)
## To confirm a single axis movement of manipulator easily

``` 
sudo apt-get install ros-{distro}-rqt-joint-trajectory-controller 
source /opt/ros/{distro}/setup.bash 
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller 
```

![sample2](https://i.imgur.com/PHdqmfn.png)
![sample3](https://i.imgur.com/InP1J9Z.png)

http://wiki.ros.org/rqt_joint_trajectory_controller

### Sample code (you input only this command, so robot_controller and rviz, smach_viewer launched)

``roslaunch seed_r7_samples demo.launch``

In detail : https://github.com/seed-solutions/seed_r7_ros_pkg/blob/master/seed_r7_samples/README.md


## With real robot

### Enviroment Setting
1. When you connect your PC and robot main controller with USB FTDI, 

``rosrun seed_r7_bringup make_udev_install.py``

You need to set upper-udev-rules and lower-udev-rules.


### Make map with navigation

``roslaunch seed_r7_navigation wheel_with_making_map.launch``

### Save map

``roslaunch seed_r7_navigation map_saver.launch``

※Caution: Don't exit nodes of make_map before launch map_saver.


Map file is saved in 

``/home/{USER}/ros/{distro}/src/seed_r7_ros_pkg/seed_r7_navigation/config/map.pgm``

### Use saved map

``roslaunch seed_r7_navigation wheel_with_static_map.launch``

