# seed_r7_ros_pkg
seed-noid meta package
## How to run
### Bring up robot_control(with real robot and Simulation on rviz)

``roslaunch seed_r7_bringup seed_r7_bringup.launch`` 

## Without real robot (Simulation on rviz)

``roslaunch seed_r7_navigation wheel_with_dummy.launch`` 



## Sample code (you input only this command, so robot_controller and rviz, smach_viewer launched)

``roslaunch seed_r7_samples demo.launch``

in detail : https://github.com/seed-solutions/seed_r7_ros_pkg/blob/master/seed_r7_samples/README.md


## With real robot

### Make map with navigation

``roslaunch seed_r7_navigation wheel_with_making_map.launch``

### Save map

``roslaunch seed_r7_navigation map_saver.launch``

※Caution: Don't exit nodes of make_map before launch map_saver.


Map file is saved in 

``/home/{USER}/ros/{distro}/src/seed_r7_ros_pkg/seed_r7_navigation/config/map.pgm``

### Use saved map

``roslaunch seed_r7_navigation wheel_with_static_map.launch``

