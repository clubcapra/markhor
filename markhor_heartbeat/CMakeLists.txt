cmake_minimum_required(VERSION 3.0.2)
project(markhor_heartbeat)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp 
)
include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/heartbeat.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

