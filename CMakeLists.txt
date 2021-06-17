cmake_minimum_required(VERSION 3.0.2)
project(steve_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_gamepad src/teleop_gamepad.cpp)
target_link_libraries(teleop_gamepad ${catkin_LIBRARIES})
