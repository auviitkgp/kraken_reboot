cmake_minimum_required(VERSION 2.8)

project(Xeno_kf-ebt C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -pedantic -O2 -Wno-long-long -fno-omit-frame-pointer -lpthread")

FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE (Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs sensor_msgs
)



link_directories ( ${OpenCV_LIB_DIR} )
MESSAGE(STATUS "OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")


MESSAGE(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR}/)


add_executable(KF-EBT_WebCam main_cam.cpp trackers/ASMS/colotracker.cpp trackers/ASMS/histogram.cpp trackers/ASMS/region.cpp trackers/kcf/piotr_fhog/gradientMex.cpp trackers/kcf/adjust.cpp trackers/kcf/kcf.cpp  kfebt.cpp trackers/tasms.cpp trackers/tkcf.cpp trackers/CBT/consensus/fastcluster/fastcluster.cpp trackers/CBT/consensus/common.cpp trackers/CBT/consensus/Consensus.cpp trackers/tcbt.cpp trackers/CBT/cbt.cpp trackers/CBT/consensus/coloravaliation.cpp trackers/GMM/gmm.cpp trackers/MOSSE/mosse.cpp trackers/NCC/ncc.cpp trackers/tncc.cpp trackers/tgmm.cpp trackers/tmosse.cpp trackers/tvdp.cpp kfebtracker.cpp)

target_link_libraries(KF-EBT_WebCam ${OpenCV_LIBS} pthread ${catkin_LIBRARIES})
