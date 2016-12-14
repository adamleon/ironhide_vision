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

#add_library(ironhide_vision SHARED src/ray_trace_cloud_loader.cpp)

file(GLOB mesh_loader_SRC "src/MeshLoader/*.cpp")

message("MeshLoader: " ${mesh_loader_SRC})

add_executable(mesh_loader ${mesh_loader_SRC})

target_link_libraries(mesh_loader
  ${PCL_LIBRARIES} 
  ${VTK_LIBRARIES}
  )


#install(TARGETS ironhide_vision DESTINATION /usr/lib/ironhide_vision)
#install(FILES include/ray_trace_cloud_loader.h DESTINATION /usr/include/ironhide_vision)
#install(FILES include/ray_trace_cloud_exceptions.h DESTINATION /usr/include/ironhide_vision)
#install(FILES include/ray_trace_cloud.h DESTINATION /usr/include/ironhide_vision)
#install(FILES FindIronhideVision.cmake DESTINATION ${CMAKE_ROOT}/Modules)
