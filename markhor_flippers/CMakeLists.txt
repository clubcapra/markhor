cmake_minimum_required(VERSION 3.9)
project(markhor_flippers LANGUAGES CXX)

add_compile_options(-std=c++14)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package (Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  roslaunch
  hardware_interface
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    controller_manager 
    roscpp 
    hardware_interface
)
include_directories(
    src
    include
    include/lib
    include/ctre
    ${catkin_INCLUDE_DIRS}
)

set(source src/markhor_flippers.cpp src/markhor_hw_interface_flippers.cpp)

add_executable(${PROJECT_NAME}_node ${source})

target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES}
    )

target_link_libraries(${PROJECT_NAME}_node
    Threads::Threads
    )

message("Your architecture is " ${CMAKE_LIBRARY_ARCHITECTURE})
message("Your path " ${PROJECT_SOURCE_DIR}/src)
if(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    message("Jetson TX2")
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/aarch64/libCTRE_PhoenixCore.a)
else()
    message("x86-64")
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${PROJECT_NAME}_node ${PROJECT_SOURCE_DIR}/include/lib/x86-64/libCTRE_PhoenixCore.a)
endif()

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
