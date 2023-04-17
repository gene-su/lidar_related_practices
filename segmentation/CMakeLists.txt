cmake_minimum_required(VERSION 3.2)

project(segmentation)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-Wall")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (segmentation src/segmentation.cpp)
target_link_libraries (segmentation ${PCL_LIBRARIES})