cmake_minimum_required(VERSION 3.0.2)
set(SETUPTOOLS_DEB_LAYOUT OFF CACHE BOOL "Build with old setup.py convention")
project(sl1m_ros)

## C++ 11
set (CMAKE_CXX_STANDARD 14)

## Find dependecies
set(CATKIN_COMPONENTS_DEPENDENCIES
  rospy
  visualization_msgs
  tf2_msgs
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_COMPONENTS_DEPENDENCIES} message_generation)
find_package(sl1m REQUIRED)

## Python package
catkin_python_setup()

## catkin specific configuration
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ${CATKIN_COMPONENTS_DEPENDENCIES} message_runtime
#   DEPENDS system_lib
)

## Install nodes
install(PROGRAMS
  src/nodes/sl1m_ros
  src/nodes/hpp_rbprm_server
  src/nodes/footstep_planner_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
