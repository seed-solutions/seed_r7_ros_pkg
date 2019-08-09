# seed_r7_ros_pkg
seed-noid meta package
## How to run
### To use bring up robot_control

``roslaunch seed_r7_bringup seed_r7_bringup.launch`` 

### To use make_map

``roslaunch seed_r7_navigation wheel_with_make_map.launch``

### To use map_save

``roslaunch seed_r7_navigation map_saver.launch``

※Caution: Don't exit nodes of make_map before launch map_saver.


Map file is saved in 

``/home/{USER}/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_navigation/config/map.pgm``

### To use saved map

``roslaunch seed_r7_navigation wheel_with_static_map.launch``

