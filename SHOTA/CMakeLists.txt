cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(shotA)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (shotA shotA.cpp)
target_link_libraries (shotA ${PCL_LIBRARIES})
target_link_libraries (shotA ${PCL_COMMONLIBRARIES})
target_link_libraries (shotA ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries (shotA ${OpenCV_LIBS})
