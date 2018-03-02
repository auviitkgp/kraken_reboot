cmake_minimum_required(VERSION 2.8.3)
project(GMM_r)

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

add_executable(gmm_r src/gmm_r.cpp)
target_link_libraries(gmm_r  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(pub_r src/publisher_r.cpp)
target_link_libraries(pub_r  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(sub_r src/subscriber_r.cpp)
target_link_libraries(sub_r  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_executable(kalman_r src/kalman_r.cpp)
target_link_libraries(kalman_r ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
