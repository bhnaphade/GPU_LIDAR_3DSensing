cmake_minimum_required(VERSION 2.8)
project(simple_grabber)
include_directories("/home/ubuntu/rplidar/sdk/sdk/include")
add_executable(lid_simple_grabber simple_grabber.cpp)
find_library(RPLIDAR_LIBRARY rplidar_sdk HINTS /home/ubuntu/rplidar/sdk/output/Linux/Release)
target_link_libraries(lid_simple_grabber PUBLIC ${RPLIDAR_LIBRARY} -lpthread)
