# Install script for directory: /home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/kinetic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "None")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/seed_r7_ros_controller/srv" TYPE FILE FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/seed_r7_ros_controller/cmake" TYPE FILE FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/catkin_generated/installspace/seed_r7_ros_controller-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/include/seed_r7_ros_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/share/roseus/ros/seed_r7_ros_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/share/common-lisp/ros/seed_r7_ros_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/share/gennodejs/ros/seed_r7_ros_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/lib/python2.7/dist-packages/seed_r7_ros_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/lib/python2.7/dist-packages/seed_r7_ros_controller")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/catkin_generated/installspace/seed_r7_ros_controller.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/seed_r7_ros_controller/cmake" TYPE FILE FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/catkin_generated/installspace/seed_r7_ros_controller-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/seed_r7_ros_controller/cmake" TYPE FILE FILES
    "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/catkin_generated/installspace/seed_r7_ros_controllerConfig.cmake"
    "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/catkin_generated/installspace/seed_r7_ros_controllerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/seed_r7_ros_controller" TYPE FILE FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller" TYPE EXECUTABLE FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/lib/seed_r7_ros_controller/seed_r7_ros_controller")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller"
         OLD_RPATH "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/lib:/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/seed_r7_ros_controller/seed_r7_ros_controller")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/devel/lib/libseed_r7_hand_controllers.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so"
         OLD_RPATH "/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libseed_r7_hand_controllers.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/seed_r7_ros_controller" TYPE DIRECTORY FILES "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/include/seed_r7_ros_controller/")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/obj-x86_64-linux-gnu/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
