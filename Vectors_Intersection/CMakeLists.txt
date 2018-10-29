cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(vector_intersection)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (vector_intersection vector_intersection.cpp)
target_link_libraries (vector_intersection ${PCL_LIBRARIES})
target_link_libraries (vector_intersection ${PCL_COMMONLIBRARIES})
target_link_libraries (vector_intersection ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries (vector_intersection ${OpenCV_LIBS})
