cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_samples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  seed_r7_ros_controller
  moveit_core moveit_ros_planning moveit_ros_planning_interface
)

catkin_package()

link_directories(${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_definitions(-std=c++11)  # should be c++14 for melodic

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/demo.launch)
  find_package(rostest REQUIRED)
  add_rostest(test/demo.test)
endif()

# Executable
add_executable(moveit_sample_node src/moveit_sample.cpp)
target_link_libraries(moveit_sample_node ${catkin_LIBRARIES} )

add_executable(record_robot_info_node src/record_robot_info.cpp)
target_link_libraries(record_robot_info_node ${catkin_LIBRARIES} )

# Install
install(DIRECTORY config launch scripts 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/demo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS moveit_sample_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
