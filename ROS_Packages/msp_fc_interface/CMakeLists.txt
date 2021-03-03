cmake_minimum_required(VERSION 2.8.3)
project(msp_fc_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  radar_avoid_msgs
  rc_msgs
  message_generation
)

add_message_files(
    FILES
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS src
    CATKIN_DEPENDS roscpp message_runtime
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
    include
    include/arch/linux
    ${catkin_INCLUDE_DIRS}
)
add_library(UDP include/arch/linux/udp_socket.c)

add_executable(msp_fc_interface
    src/msp_node.cpp
    src/serial.cpp
    src/AP.cpp
    src/controller.cpp
    src/natnet.cpp)

target_link_libraries(msp_fc_interface
    UDP
    ${catkin_LIBRARIES})
