set(IRONHIDE_VISION_ROOT
	"${IRONHIDE_VISION_ROOT}"
	CACHE
	PATH
	"/opt/ironhide_vision")

find_library(IRONHIDE_VISION_LIBRARY
	NAMES
	ironhidevisioncore
	PATHS
	"${IRONHIDE_VISION_ROOT}"
	PATH_SUFFIXES
	lib)

find_path(IRONHIDE_VISION_INCLUDE_DIR
	NAMES
	ironhide_vision/ray_trace_cloud_loader.h
	PATHS
	"${IRONHIDE_VISION_ROOT}"
	PATH_SUFFIXES
	include)


# handle the QUIETLY and REQUIRED arguments and set xxx_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ironhide-vision
	DEFAULT_MSG
	IRONHIDE_VISION_LIBRARY
	IRONHIDE_VISION_INCLUDE_DIR
	)


mark_as_advanced(IRONHIDE_VISION_LIBRARY
	IRONHIDE_VISION_INCLUDE_DIR
	)

if(IRONHIDE_VISION_FOUND)
	mark_as_advanced(IRONHIDE_VISION_ROOT)
endif()

