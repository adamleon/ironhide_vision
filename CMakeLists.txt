cmake_minimum_required(VERSION 2.8.11)
project(ironhide_vision)

set(IRONHIDE_SOURCES src/*.cpp)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)

add_library(${PROJECT_NAME} src/ray_trace_cloud_loader.cpp)
