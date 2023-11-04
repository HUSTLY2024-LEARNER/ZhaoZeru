cmake_minimum_required(VERSION 3.8)
project(at_try)
set(SRC_LIST )

find_package(OpenCV 4 REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
set(SRC_LIST ${SRC_LIST} src/find.cpp)

add_executable(at_try ${SRC_LIST} )
target_link_libraries(at_try ${OpenCV_LIBS})
