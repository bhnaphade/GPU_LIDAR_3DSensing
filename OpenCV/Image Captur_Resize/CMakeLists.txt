cmake_minimum_required(VERSION 2.8)
project(Resize)
find_package(OpenCV REQUIRED)
add_executable(cv_resize resize.cpp)
target_link_libraries(cv_resize ${OpenCV_LIBS})

