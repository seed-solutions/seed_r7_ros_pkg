cmake_minimum_required(VERSION 3.10)
project(seed_ros_controller)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

USE_QT(Widgets)

USE_ROS(roscpp 
        std_msgs
        tf_conversions
        controller_manager
        hardware_interface
        joint_limits_interface
        pluginlib
        message_generation
)

# これがないと、develに移動してくれない
# 外部からロードするときの情報
catkin_package(CATKIN_DEPENDS roscpp
  DEPENDS
  INCLUDE_DIRS src)

ADD_FILES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_FILES)
ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

include_directories(include ${HEADER_DIRS})
add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS} pthread -lstdc++fs )
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g -Wall)