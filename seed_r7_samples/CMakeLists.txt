cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_samples)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/demo.launch)
  find_package(rostest REQUIRED)
  add_rostest(test/demo.test)
endif()

install(DIRECTORY config launch scripts 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



catkin_install_python(PROGRAMS scripts/demo.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
