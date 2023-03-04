cmake_minimum_required(VERSION 3.0.2)
project(markhor_2dnav)

find_package(catkin REQUIRED COMPONENTS
  move_base
  my_odom_configuration_dep
  my_sensor_configuration_dep
  my_tf_configuration_dep
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES markhor_2dnav
#  CATKIN_DEPENDS move_base my_odom_configuration_dep my_sensor_configuration_dep my_tf_configuration_dep
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)