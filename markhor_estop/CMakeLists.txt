cmake_minimum_required(VERSION 3.0.2)
project(markhor_estop)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES markhor_estop
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(estop
  src/estop.cpp
  )

add_dependencies(estop
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(estop PRIVATE JetsonGPIO)

target_link_libraries(estop ${catkin_LIBRARIES})