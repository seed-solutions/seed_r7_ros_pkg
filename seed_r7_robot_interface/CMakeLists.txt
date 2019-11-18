cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_robot_interface)


# Dependencies

# catkin ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  seed_r7_ros_controller
)
if(NOT catkin_LIBRARIES)
  message(FATAL "catkin required but not supported")
endif()

# ROS setups

catkin_package(
  CATKIN_DEPENDS roscpp seed_r7_ros_controller
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}_typeF
)

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)  # should be c++14 for melodic

add_library(${PROJECT_NAME}_typeF typef/src/stroke_converter.cpp)
target_link_libraries(${PROJECT_NAME}_typeF ${catkin_LIBRARIES})

# Install
install(
  DIRECTORY typef/config
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install
install(TARGETS ${PROJECT_NAME}_typeF
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
