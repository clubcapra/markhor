cmake_minimum_required(VERSION 3.0.2)
project(markhor_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES SetReturnPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(

    # INCLUDE_DIRS include
    # LIBRARIES markhor_navigation
    # CATKIN_DEPENDS move_base my_odom_configuration_dep my_sensor_configuration_dep my_tf_configuration_dep
    # DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)

include_directories(

    # include
    # ${catkin_INCLUDE_DIRS}
)