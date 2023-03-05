cmake_minimum_required(VERSION 3.9)
project(markhor_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  sick_tim
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
