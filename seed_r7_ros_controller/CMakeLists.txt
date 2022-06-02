cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_ros_controller)


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
  tf
  seed_r7_description
  seed_smartactuator_sdk
  pluginlib
)

# boost
find_package(Boost REQUIRED COMPONENTS thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

add_message_files(
   FILES
   MsSystemConfig.msg
   MsSystemConfigList.msg
)

# Generate services in the 'srv' folder
# Below service should be removed in future
add_service_files(
   FILES
   HandControl.srv
   LedControl.srv
   SetInitialPose.srv
   ResetRobotStatus.srv
   GetAeroCommand.srv
   CosmoCommand.srv
   CosmoSelectNumber.srv
   MsSystemConfigWrite.srv
   MsSystemConfigRead.srv
   MsConfigWrite.srv
   MsConfigRead.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES seed_r7_hand_controllers
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
    tf
    seed_r7_description
    seed_smartactuator_sdk
    pluginlib
)

link_directories(${catkin_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_definitions(-std=c++11)  # should be c++14 for melodic

# Library
add_library(seed_r7_hand_controllers src/seed_r7_hand_controller.cpp)
add_dependencies(seed_r7_hand_controllers ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(seed_r7_hand_controllers ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Executable
add_executable(${PROJECT_NAME} src/seed_r7_ros_controller.cpp
                               src/seed_r7_robot_hardware.cpp
                               src/seed_r7_mover_controller.cpp
                               src/seed_r7_upper_controller.cpp
                               src/seed_r7_lower_controller.cpp
                               src/stroke_converter_base.cpp
                               src/configurator.cpp
                               )
target_link_libraries(${PROJECT_NAME} seed_r7_hand_controllers
                                      ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# Install
install(TARGETS ${PROJECT_NAME} seed_r7_hand_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
