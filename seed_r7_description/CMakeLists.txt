cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_description)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(tests/roslaunch_seed_r7_typef.xml)
endif()

install(DIRECTORY meshes urdf typef csv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
