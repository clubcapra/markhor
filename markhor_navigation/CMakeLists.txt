cmake_minimum_required(VERSION 3.0.2)
project(markhor_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  StartExploration.srv
  StopExploration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)