cmake_minimum_required(VERSION 2.8.3)
project(noid_mover_navigation)

find_package(catkin REQUIRED)

catkin_package()

install(
    DIRECTORY config launch maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

