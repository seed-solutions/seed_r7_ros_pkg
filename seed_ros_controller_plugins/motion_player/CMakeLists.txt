cmake_minimum_required(VERSION 3.0.2)
project(motion_player)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

USE_QT(Widgets SerialPort Multimedia)

USE_ROS(roscpp 
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    seed_ros_controller
    message_generation
    genmsg
    actionlib_msgs
)


ADD_ROS_SERVICES(
    Play.action
    Load.srv
)

# これがないと、develに移動してくれない
# 外部からロードするときの情報
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    realtime_tools
    roscpp
)


ADD_FILES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_FILES)
ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

set(HEADER_DIRS ${HEADER_DIRS} ${PROJECT_SOURCE_DIR}/third_party)
set(HEADER_DIRS ${HEADER_DIRS} ${PROJECT_SOURCE_DIR}/third_party/picojson)

include_directories(include ${HEADER_DIRS})

add_library(${PROJECT_NAME}  ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS} pthread -lstdc++fs  pulse-simple pulse)
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g -Wall)
