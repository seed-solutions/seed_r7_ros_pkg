cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_navigation)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rospy
)


catkin_python_setup()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_test_navigation.xml)
endif()

install(
    DIRECTORY config launch maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/dummy_scan.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

