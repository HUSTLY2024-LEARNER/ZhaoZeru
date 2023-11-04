cmake_minimum_required(VERSION 3.8)
project(camera-try)
set(SRC_LIST src/distorted.cpp)

find_package(OpenCV 4 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(camera-try ${SRC_LIST})
target_link_libraries(camera-try ${OpenCV_LIBS})

