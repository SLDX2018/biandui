project(group_formation)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
# set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  nav_msgs
  geometry_msgs
  roborts_common
)

# find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  GroupFormationProtoSrc
  GroupFormationProtoHds
  ${ProtoFiles}
)

catkin_package()

add_executable(unit_control_node
  ${GroupFormationProtoHds}
  ${GroupFormationProtoSrc}
  unit_control.cpp
)

target_link_libraries(unit_control_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)


add_executable(group_control_node
  ${GroupFormationProtoHds}
  ${GroupFormationProtoSrc}
  group_control.cpp
)

target_link_libraries(group_control_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)
