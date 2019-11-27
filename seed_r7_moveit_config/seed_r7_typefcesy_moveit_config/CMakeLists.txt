cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_typefcesy_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
<<<<<<< HEAD
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
=======
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
>>>>>>> c0219c946923f983aeb3c9ac25f7eb61f51a6290
