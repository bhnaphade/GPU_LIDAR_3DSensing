cmake_minimum_required(VERSION 2.8)
project(Resize)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(cv_resize image_resize.cpp)
target_link_libraries(cv_resize ${OpenCV_LIBS})

