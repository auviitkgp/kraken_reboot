cmake_minimum_required(VERSION 2.8.3)
project(GMM_a)

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

add_executable(gmm_a src/gmm_a.cpp)
target_link_libraries(gmm_a  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(pub_a src/publisher_a.cpp)
target_link_libraries(pub_a  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(sub_a src/subscriber_a.cpp)
target_link_libraries(sub_a  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
