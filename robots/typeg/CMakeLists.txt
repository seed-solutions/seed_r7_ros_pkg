cmake_minimum_required(VERSION 3.10)
project(typeg)

set(CMAKE_CXX_STANDARD 20)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  seed_ros_controller
)

catkin_package(
  CATKIN_DEPENDS roscpp seed_ros_controller
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
)

add_subdirectory(stroke_converter)