project(capra_gpio_control)

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

add_library(jetsonGPIO_gpio_control include/JetsonXavierGPIO/jetsonGPIO.c include/JetsonXavierGPIO/jetsonGPIO.h)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(gpio_control
        src/gpio_control.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(gpio_control
        ${catkin_EXPORTED_TARGETS}
        )

target_include_directories(gpio_control PRIVATE JetsonXavierGPIO)

target_link_libraries(gpio_control ${catkin_LIBRARIES})

target_link_libraries(gpio_control jetsonGPIO_gpio_control)

