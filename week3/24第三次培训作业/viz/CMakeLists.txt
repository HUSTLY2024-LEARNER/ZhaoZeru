cmake_minimum_required(VERSION 3.2)
project(vtkDisplay)

set(CMAKE_PREFIX_PATH "/home/zzr/opencv420/build/install/")
find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "   OpenCV Version: ${OpenCV_VERSION}")


set(CMAKE_CXX_STANDARD 14)


add_executable(vtkDisplay src/vtkDisplay.cpp )
target_link_libraries(vtkDisplay ${OpenCV_LIBS} )


