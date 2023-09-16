cmake_minimum_required(VERSION 3.0.2)
project(tpv_joint_state_pub)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable( tpv_joint_state_pub 
                src/main.cpp
                )

add_dependencies(tpv_joint_state_pub
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(tpv_joint_state_pub ${catkin_LIBRARIES})
