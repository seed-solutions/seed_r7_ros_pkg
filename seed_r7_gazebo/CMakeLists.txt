cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_gazebo)

if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
    add_compile_options(-std=c++17)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
)

catkin_package()

## Gazebo plugins
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories( ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mimic_plugin src/MimicPlugin.cpp)
target_link_libraries(mimic_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS mimic_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(gazebo_ros_planar_force_move src/gazebo_ros_planar_move.cpp)
target_link_libraries(gazebo_ros_planar_force_move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#install(TARGETS gazebo_ros_planar_force_move LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install
install(DIRECTORY config launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS gazebo_ros_planar_force_move mimic_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
