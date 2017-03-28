cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation)

SET(CMAKE_C_COMPILER mpicc)
SET(CMAKE_CXX_COMPILER mpicxx)
include_directories(MPI_INCLUDE_PATH)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
