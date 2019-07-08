cmake_minimum_required(VERSION 2.8.3)
project(noid_ros_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs 
  geometry_msgs
  sensor_msgs 
  nav_msgs 
  control_msgs
  move_base_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  joint_limits_interface
  message_generation
  angles
  urdf
  noid_robot_interface
  seed_solutions_sdk

#  pluginlib
)

## Generate services in the 'srv' folder

add_service_files(
   FILES
   GraspControl.srv
   HandControl.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_interface
  CATKIN_DEPENDS
    roscpp
    std_msgs
    trajectory_msgs 
    sensor_msgs
    geometry_msgs 
    nav_msgs 
    control_msgs 
    move_base_msgs
    controller_manager
    control_toolbox
    hardware_interface
    transmission_interface
    joint_limits_interface
    angles
    urdf
    noid_robot_interface

    #    pluginlib
)

find_package(Eigen3 REQUIRED)
##find_package(Boost REQUIRED COMPONENTS thread)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# add c++ dependencies :
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCXX11_SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCXX11_SUPPORTED)
else()
  message(FATAL "c++11 required but not supported")
endif()

## Library


## Executable
add_executable(${PROJECT_NAME} src/noid_ros_controller.cpp src/noid_robot_hardware.cpp )
target_link_libraries(${PROJECT_NAME} noid_controllers ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(noid_hand_controller_node src/noid_hand_controller.cpp)
target_link_libraries(noid_hand_controller_node  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)




