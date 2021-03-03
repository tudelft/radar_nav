cmake_minimum_required(VERSION 2.8.3)
project(radar_ros_driver)

find_package(catkin_simple REQUIRED COMPONENTS roscpp std_msgs) #genmsg
catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -O3")

# make the executable
cs_add_executable(radar_ros_driver
    src/driver_node.cpp
    src/driver.cpp
    src/ofxRadar24Ghz.cpp
    src/EndpointTargetDetection.c 
    src/EndpointCalibration.c
    src/EndpointRadarAdcxmc.c
    src/EndpointRadarBase.c 
    src/EndpointRadarDoppler.c 
    src/EndpointRadarFmcw.c 
    src/EndpointRadarP2G.c
    src/EndpointRadarIndustrial.c 
    src/EndpointRadarErrorCodes.c
    src/Protocol.c
    src/COMPort_Unix.c
    src/COMPort_Windows.c
    src/Hungarian.cpp
)

# make the nodelet into a library
cs_add_library(radar_ros_driver_nodelet
    src/driver_nodelet.cpp
    src/driver.cpp
    src/ofxRadar24Ghz.cpp
    src/EndpointTargetDetection.c 
    src/EndpointCalibration.c
    src/EndpointRadarAdcxmc.c
    src/EndpointRadarBase.c 
    src/EndpointRadarDoppler.c 
    src/EndpointRadarFmcw.c 
    src/EndpointRadarP2G.c
    src/EndpointRadarIndustrial.c 
    src/EndpointRadarErrorCodes.c
    src/Protocol.c
    src/COMPort_Unix.c
    src/COMPort_Windows.c
    src/Hungarian.cpp 
)

# link the executable to the necesarry libs
target_link_libraries(radar_ros_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# link the executable to the necesarry libs
target_link_libraries(radar_ros_driver_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the nodelet library
cs_install()

# Install other support files for installation
install(FILES radar_ros_driver_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
