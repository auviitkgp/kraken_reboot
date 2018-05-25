cmake_minimum_required(VERSION 2.8.3)
project(serial_comm)


find_package(catkin REQUIRED COMPONENTS
  kraken_msgs
  roscpp
  serial
  std_msgs
  resources
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_comm
#  CATKIN_DEPENDS kraken_msgs roscpp serial std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
