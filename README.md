# seed_r7_ros_pkg  
seed-noid meta package

## Continuous Integration Status
service    | Master  |
---------- | ------- |
Travis     | [![Build Status](https://travis-ci.com/seed-solutions/seed_r7_ros_pkg.svg?branch=master)](https://travis-ci.com/seed-solutions/seed_r7_ros_pkg/) |

service    | Kinetic | Melodic |
---------- | ------- | ------- |
ROS Buildfarm     | [![Build Status](http://build.ros.org/job/Kbin_uX64__seed_r7_ros_pkg__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__seed_r7_ros_pkg__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/job/Mbin_uB64__seed_r7_ros_pkg__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__seed_r7_ros_pkg__ubuntu_bionic_amd64__binary/) |
## How to install
### 0. ROS
[Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)  

### 1. From Debian
```
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-seed-r7-ros-pkg
```

### 1. From Source
You need to clone and build some packages in your catkin workspace. 
Below is an example.
```
sudo apt-get install python-wstool python-catkin-tools
mkdir -p ~/ros/${ROS_DISTRO}
cd ~/ros/${ROS_DISTRO}
catkin init
catkin build
source ~/ros/${ROS_DISTRO}/devel/setup.bash
echo "source ~/ros/${ROS_DISTRO}/devel/setup.bash" >> ~/.bashrc
cd ~/ros/${ROS_DISTRO}/src
git clone https://github.com/seed-solutions/seed_smartactuator_sdk
git clone https://github.com/seed-solutions/seed_r7_ros_pkg.git
cd ~/ros/${ROS_DISTRO}
rosdep install -y -r --from-paths src --ignore-src
catkin build seed_r7_ros_pkg
source ~/.bashrc
```
### <a name = "udev_setting"> 2. Udev Setting (if you have a robot)   
If you still have not registered the robot's USB for your PC, you have to do it.
(When you register that, ``/etc/udev/rules.d/90-aero.rules`` need to be exist.)
Firstly, connect the robot's USB to your PC, and run the following command.    
```
rosrun seed_r7_bringup make_udev_install.py
```
Secondly, please disconnect and reconnect the USB.
When you run ``ls -l /dev/aero*``, the follwing message will appear if USB is recognized:
```
/dev/aero_upper -> ttyUSB0
/dev/aero_lower -> ttyUSB1
```
The number of ttyUSB may not be same, but there is no problem.

## How to launch
### Bring up ros_control ( without / with real robot )
If you use real robot, please refer to [Udev Setting](#udev_setting).
```
roslaunch seed_r7_bringup seed_r7_bringup.launch
rosrun rviz rviz -d `rospack find seed_r7_bringup`/rviz.rviz
``` 
In either case, you have to run this command. 
The robot model is not displayed on rviz at this time, because ``Fixed Frame`` is set as``map`` that does not exist. However, it will be displayed correctly in the next step.

### without real robot
```
roslaunch seed_r7_navigation wheel_with_dummy.launch
```
<img src="https://user-images.githubusercontent.com/12426780/74708739-af52d200-5260-11ea-86ba-9fe747c48414.png" width="80%">

### with real robot
1. make map and save it     
make map : ``roslaunch seed_r7_navigation wheel_with_making_map.launch``    
save map : ``roslaunch seed_r7_navigation map_saver.launch  #run after making map``    
**Do not kill nodes of making_map before starting map_saver.launch.**      
Map file is saved in ``~/ros/${ROS_DISTRO}/src/seed_r7_ros_pkg/seed_r7_navigation/config/map.pgm``    
2. use saved map    
``roslaunch seed_r7_navigation wheel_with_static_map.launch``

## How to use
### rqt_joint_trajectory_controller
It is easy to confirm a single axis movement by using [rqt_joint_trajectory_controller](http://wiki.ros.org/rqt_joint_trajectory_controller).
``` 
sudo apt-get install ros-${ROS_DISTRO}-rqt-joint-trajectory-controller 
source ~/.bashrc
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller 
```
<img src="https://user-images.githubusercontent.com/12426780/74709558-cc88a000-5262-11ea-96b9-a0069b1aff33.png" width="80%">

### MoveIt!
Please run the following command.
```
roslaunch seed_r7_samples demo.launch
```
**Do not connect the robot's USB on your computer!**
**The SEED-Mover start to move!**

The [ros_controller](http://wiki.ros.org/ros_control), [moveit](http://wiki.ros.org/moveit), [navigation](http://wiki.ros.org/navigation), [smach_viewer](http://wiki.ros.org/smach_viewer) and [rviz](http://wiki.ros.org/rviz) are launched, and [sample code](seed_r7_samples/scripts/demo.py) will be started. This code mainly includes [smach](http://wiki.ros.org/smach), [moveit_commander](http://wiki.ros.org/moveit_commander) and [actionlib](http://wiki.ros.org/actionlib).
![moveit](https://user-images.githubusercontent.com/12426780/68364074-455e6300-0170-11ea-82d4-ffb3afb18af1.gif)

If you want to make more complex programs, please refer to [Kinetic tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html), [Melodic tutorial](http://docs.ros.org/melodic/api/moveit_tutorials/html/index.html) or [MoveIt Manual by TORK](https://github.com/tork-a/tork_moveit_tutorial/releases/download/0.0.5/tork_moveit_tutorial-0.0.5.pdf)(in Japanese).    
**When using MoveIt! commands, be sure to run following launch file.**
```
roslaunch seed_r7_bringup moveit.launch 
```

### Grasp / Release
* Right Hand    
grasp : ``rosservice call /seed_r7_ros_controller/hand_control 0 grasp 100``    
release : ``rosservice call /seed_r7_ros_controller/hand_control 0 release 100
``    
* Left Hand   
grasp : ``rosservice call /seed_r7_ros_controller/hand_control 1 grasp 100``    
release : ``rosservice call /seed_r7_ros_controller/hand_control 1 release 100    
``

**The last number is current setting[%]**
