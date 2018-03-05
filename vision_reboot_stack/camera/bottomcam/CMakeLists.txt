cmake_minimum_required(VERSION 2.8.3)
project(bottomcam)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(bottomcam src/bottomcam.cpp)
target_link_libraries(bottomcam ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
