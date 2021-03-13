cmake_minimum_required(VERSION 3.9)
project(markhor_base LANGUAGES CXX)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  roslaunch
  hardware_interface
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES markhor_base
  CATKIN_DEPENDS 
    controller_manager 
    roscpp 
    hardware_interface
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
  src/markhor_base.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)

message("Your architecture is " ${CMAKE_LIBRARY_ARCHITECTURE})
message("Your path " ${PROJECT_SOURCE_DIR}/src)
if(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    message("Jetson TX2")
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCanutils.a)
else()
    message("x86-64")
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCanutils.a)
endif()

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)