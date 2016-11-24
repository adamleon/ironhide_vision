cmake_minimum_required(VERSION 2.8.11)
project(ironhide_vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7.1 REQUIRED COMPONENTS
  segmentation
  filters
  surface
  visualization
  apps
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED src/ray_trace_cloud_loader.cpp)

install(TARGETS ironhide_vision DESTINATION /opt/ironhide_vision/lib)
install(FILES include/ironhide_vision.h DESTINATION /opt/ironhide_vision/include)
install(FILES FindIronhideVision.cmake DESTINATION ${CMAKE_ROOT}/Modules)
