cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

generate_arduino_firmware(kraken
  SRCS BATTERY_ARDUINO.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD nano
  PORT /dev/ttyUSB0
  ARDLIBS Wire
)

#set(Wire_RECURSE False)
