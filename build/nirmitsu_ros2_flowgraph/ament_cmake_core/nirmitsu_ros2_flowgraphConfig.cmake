# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nirmitsu_ros2_flowgraph_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nirmitsu_ros2_flowgraph_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nirmitsu_ros2_flowgraph_FOUND FALSE)
  elseif(NOT nirmitsu_ros2_flowgraph_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nirmitsu_ros2_flowgraph_FOUND FALSE)
  endif()
  return()
endif()
set(_nirmitsu_ros2_flowgraph_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nirmitsu_ros2_flowgraph_FIND_QUIETLY)
  message(STATUS "Found nirmitsu_ros2_flowgraph: 0.1.0 (${nirmitsu_ros2_flowgraph_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nirmitsu_ros2_flowgraph' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nirmitsu_ros2_flowgraph_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nirmitsu_ros2_flowgraph_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nirmitsu_ros2_flowgraph_DIR}/${_extra}")
endforeach()
