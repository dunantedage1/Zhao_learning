cmake_minimum_required(VERSION 3.0.2)
project(complete_mission)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(complete_mission_node
  src/complete_mission.cpp
)

target_link_libraries(complete_mission_node ${catkin_LIBRARIES})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
