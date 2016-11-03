cmake_minimum_required(VERSION 2.8)
project(Resize)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(cv_image_pyramid image_pyramid.cpp)
add_executable(cv_video_pyramid video_pyramid.cpp)
target_link_libraries(cv_image_pyramid ${OpenCV_LIBS})
target_link_libraries(cv_video_pyramid ${OpenCV_LIBS})

