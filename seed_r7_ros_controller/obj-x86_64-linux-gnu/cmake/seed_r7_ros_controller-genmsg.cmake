# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "seed_r7_ros_controller: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(seed_r7_ros_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" NAME_WE)
add_custom_target(_seed_r7_ros_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "seed_r7_ros_controller" "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(seed_r7_ros_controller
  "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seed_r7_ros_controller
)

### Generating Module File
_generate_module_cpp(seed_r7_ros_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seed_r7_ros_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(seed_r7_ros_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(seed_r7_ros_controller_generate_messages seed_r7_ros_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" NAME_WE)
add_dependencies(seed_r7_ros_controller_generate_messages_cpp _seed_r7_ros_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seed_r7_ros_controller_gencpp)
add_dependencies(seed_r7_ros_controller_gencpp seed_r7_ros_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seed_r7_ros_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(seed_r7_ros_controller
  "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/seed_r7_ros_controller
)

### Generating Module File
_generate_module_eus(seed_r7_ros_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/seed_r7_ros_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(seed_r7_ros_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(seed_r7_ros_controller_generate_messages seed_r7_ros_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" NAME_WE)
add_dependencies(seed_r7_ros_controller_generate_messages_eus _seed_r7_ros_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seed_r7_ros_controller_geneus)
add_dependencies(seed_r7_ros_controller_geneus seed_r7_ros_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seed_r7_ros_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(seed_r7_ros_controller
  "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seed_r7_ros_controller
)

### Generating Module File
_generate_module_lisp(seed_r7_ros_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seed_r7_ros_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(seed_r7_ros_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(seed_r7_ros_controller_generate_messages seed_r7_ros_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" NAME_WE)
add_dependencies(seed_r7_ros_controller_generate_messages_lisp _seed_r7_ros_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seed_r7_ros_controller_genlisp)
add_dependencies(seed_r7_ros_controller_genlisp seed_r7_ros_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seed_r7_ros_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(seed_r7_ros_controller
  "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/seed_r7_ros_controller
)

### Generating Module File
_generate_module_nodejs(seed_r7_ros_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/seed_r7_ros_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(seed_r7_ros_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(seed_r7_ros_controller_generate_messages seed_r7_ros_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" NAME_WE)
add_dependencies(seed_r7_ros_controller_generate_messages_nodejs _seed_r7_ros_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seed_r7_ros_controller_gennodejs)
add_dependencies(seed_r7_ros_controller_gennodejs seed_r7_ros_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seed_r7_ros_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(seed_r7_ros_controller
  "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seed_r7_ros_controller
)

### Generating Module File
_generate_module_py(seed_r7_ros_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seed_r7_ros_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(seed_r7_ros_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(seed_r7_ros_controller_generate_messages seed_r7_ros_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/seed/ros/kinetic/src/seed_r7_ros_pkg/seed_r7_ros_controller/srv/HandControl.srv" NAME_WE)
add_dependencies(seed_r7_ros_controller_generate_messages_py _seed_r7_ros_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(seed_r7_ros_controller_genpy)
add_dependencies(seed_r7_ros_controller_genpy seed_r7_ros_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS seed_r7_ros_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seed_r7_ros_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/seed_r7_ros_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(seed_r7_ros_controller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(seed_r7_ros_controller_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(seed_r7_ros_controller_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/seed_r7_ros_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/seed_r7_ros_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(seed_r7_ros_controller_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(seed_r7_ros_controller_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(seed_r7_ros_controller_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seed_r7_ros_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/seed_r7_ros_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(seed_r7_ros_controller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(seed_r7_ros_controller_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(seed_r7_ros_controller_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/seed_r7_ros_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/seed_r7_ros_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(seed_r7_ros_controller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(seed_r7_ros_controller_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(seed_r7_ros_controller_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seed_r7_ros_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seed_r7_ros_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/seed_r7_ros_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(seed_r7_ros_controller_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(seed_r7_ros_controller_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(seed_r7_ros_controller_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
