project(ambition_vision2021)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(Threads REQUIRED)

find_package(OpenCV REQUIRED)

# find_package(MVS REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  roborts_msgs
)

catkin_package()

MESSAGE(@@@ "This is SOURCE dir " ${ambition_vision2021_SOURCE_DIR})

add_executable(ambition_vision2021
  main.cpp
  armor_detection/armor_detect.cpp
  buff_detection/buff_detect.cpp
  camera/camera_device.cpp
  camera/HIKvison.cpp
  camera/save_video.cpp
  common/serial/CRC_Check.cpp
  common/serial/serial_port.cpp
  common/solve_angle/solve_angle.cpp
  common/thread_control/thread_control.cpp
)

link_directories(
  /opt/MVS/lib/64
)

target_link_libraries(ambition_vision2021 PUBLIC
  # pthread
  ${OpenCV_LIBS}
  # ${MVS_LIBS}

FormatConversion
MediaProcess
MvUsb3vTL
GCBase_gcc421_v3_0
MvCameraControl
GenApi_gcc421_v3_0
NodeMapData_gcc421_v3_0
log4cpp_gcc421_v3_0
MVGigEVisionSDK
XmlParser_gcc421_v3_0
Log_gcc421_v3_0
MathParser_gcc421_v3_0
MVRender

  ${catkin_LIBRARIES}
)

include_directories(
  /opt/MVS/include
)

target_include_directories(ambition_vision2021 PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include

  ${catkin_INCLUDE_DIRS}
)