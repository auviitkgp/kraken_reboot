cmake_minimum_required(VERSION 2.8.3)
project(control_system)
find_package(catkin REQUIRED COMPONENTS
  tf
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  kraken_msgs
  message_generation
  resources
  roscpp
  rospy
  std_msgs
  nav_msgs
  msgs_stack
  resources
)
# find_package(Boost REQUIRED COMPONENTS system)

 generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} cfg/cpp
  LIBRARIES control_system
  CATKIN_DEPENDS actionlib tf actionlib_msgs dynamic_reconfigure kraken_msgs message_generation resources roscpp rospy std_msgs nav_msgs msgs_stack
  DEPENDS
)

include_directories(
  include
  cfg/cpp
  ${catkin_INCLUDE_DIRS}
)

add_executable(ControlServer_node src/main.cpp)
add_library(StateController src/StateController.cpp)
add_library(ControlServer src/ControlServer.cpp)
add_library(ControlParameters src/ControlParameters.cpp)
target_link_libraries(ControlServer_node ControlServer StateController ControlParameters ${catkin_LIBRARIES})
add_dependencies(ControlServer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
