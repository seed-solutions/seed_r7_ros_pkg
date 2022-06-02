cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_controller)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

USE_ROS(roscpp 
    controller_interface
    hardware_interface
    pluginlib
    realtime_tools
    seed_ros_controller
    aero_controller
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

include_directories(include ${HEADER_DIRS})

add_library(${PROJECT_NAME}  ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS} pthread -lstdc++fs )
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g -Wall)
