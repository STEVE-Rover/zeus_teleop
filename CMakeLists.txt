cmake_minimum_required(VERSION 3.0.2)
project(steve_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

  add_message_files(
    FILES
    VelocityTargets.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_gamepad src/teleop_gamepad.cpp)
add_executable(zero_twist_publisher src/zero_twist_publisher.cpp)
target_link_libraries(teleop_gamepad ${catkin_LIBRARIES})
target_link_libraries(zero_twist_publisher ${catkin_LIBRARIES})
