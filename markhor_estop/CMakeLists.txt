project(markhor_estop)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        )


catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)


add_library(jetsonGPIO_estop include/JetsonXavierGPIO/jetsonGPIO.c include/JetsonXavierGPIO/jetsonGPIO.h)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(estop
        src/estop.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(estop
        ${catkin_EXPORTED_TARGETS}
        )

target_include_directories(estop PRIVATE JetsonXavierGPIO)

target_link_libraries(estop ${catkin_LIBRARIES})

target_link_libraries(estop jetsonGPIO_estop)
