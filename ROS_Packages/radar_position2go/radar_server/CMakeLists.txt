cmake_minimum_required(VERSION 2.8.3)
project(radar_server)

find_package(catkin_simple REQUIRED COMPONENTS roscpp std_msgs)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(radar_server
  src/server.cpp
  src/server_node.cpp
)

# make the nodelet into a library
cs_add_library(radar_server_nodelet
  src/server_nodelet.cpp
  src/server.cpp
)

# link the executable to the necesarry libs
target_link_libraries(radar_server
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(radar_server_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_install()

# Install other support files for installation
#install(FILES radar_server_nodelet.xml radar_test.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
