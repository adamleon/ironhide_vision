# Test .cmake file for a test project
# Delete this file at any time

find_package(PCL 1.7.1 REQUIRED COMPONENTS
  segmentation
  filters
  surface
  visualization
  apps
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(IRONHIDE_VISION_INCLUDE_DIR /usr/include ${PCL_INCLUDE_DIRS} ${VTK_DIR})
set(IRONHIDE_VISION_LIBRARY ironhide_vision)
set(IRONHIDE_VISION_LIBRARY_DIR /usr/lib/ironhide_vision)