cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_samples)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch scripts config
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	  USE_SOURCE_PERMISSIONS
 )

install(FILES rviz.rviz
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
