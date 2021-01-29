cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pc_encoder)

find_package(PCL 1.11 REQUIRED COMPONENTS common io features)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/python2.7)
include_directories(/usr/include/x86_64-linux-gnu/python2.7)
include_directories(lib)

add_executable(pc_encoder core/pc_encoder.cpp core/pc_utils.cpp core/pc_utils.hpp lib/nanoflann.hpp)
target_link_libraries(pc_encoder ${PCL_LIBRARIES})
