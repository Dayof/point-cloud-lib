cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pc_encoder)

find_package(PCL 1.11 REQUIRED COMPONENTS common io)
pkg_check_modules(LZ4 REQUIRED liblz4)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/usr/include/python2.7)
include_directories(/usr/include/x86_64-linux-gnu/python2.7)
include_directories(lib)
include_directories(lz4)
include_directories(flann)
include_directories(flann_cpp)

add_executable(pc_encoder core/pc_encoder.cpp core/pc_utils.cpp core/pc_utils.hpp lib/nanoflann.hpp)
target_link_libraries(pc_encoder ${PCL_LIBRARIES})
