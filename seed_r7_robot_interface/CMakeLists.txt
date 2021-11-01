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
  LIBRARIES ${PROJECT_NAME}_typef ${PROJECT_NAME}_typeg ${PROJECT_NAME}_typeg_arm ${PROJECT_NAME}_typeg2_arm
)

include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)  # should be c++14 for melodic

add_library(${PROJECT_NAME}_typef typef/src/stroke_converter.cpp)
target_link_libraries(${PROJECT_NAME}_typef ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_typeg typeg/src/stroke_converter.cpp)
target_link_libraries(${PROJECT_NAME}_typeg ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_typeg_arm typeg_arm/src/stroke_converter.cpp)
target_link_libraries(${PROJECT_NAME}_typeg_arm ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_typeg2_arm typeg2_arm/src/stroke_converter.cpp)
target_link_libraries(${PROJECT_NAME}_typeg2_arm ${catkin_LIBRARIES})

# Install
install(
  DIRECTORY typef
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "typef/src" EXCLUDE
)
install(
  DIRECTORY typeg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "typeg/src" EXCLUDE
)
install(
  DIRECTORY typeg_arm
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "typeg_arm/src" EXCLUDE
)
install(
  DIRECTORY typeg2_arm
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "typeg2_arm/src" EXCLUDE
)

# Install
install(TARGETS ${PROJECT_NAME}_typef ${PROJECT_NAME}_typeg
  ${PROJECT_NAME}_typeg_arm ${PROJECT_NAME}_typeg2_arm
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
