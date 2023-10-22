cmake_minimum_required(VERSION 3.8)
project(segmentation)
set(SRC_LIST include/segment.h)

find_package(OpenCV 4 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
set(SRC_LIST ${SRC_LIST} src/main.cpp src/segment.cpp )

add_executable(segment ${SRC_LIST} )
target_link_libraries(segment ${OpenCV_LIBS})
