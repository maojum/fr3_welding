# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_weld_butt_seam_extracting_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED weld_butt_seam_extracting_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(weld_butt_seam_extracting_FOUND FALSE)
  elseif(NOT weld_butt_seam_extracting_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(weld_butt_seam_extracting_FOUND FALSE)
  endif()
  return()
endif()
set(_weld_butt_seam_extracting_CONFIG_INCLUDED TRUE)

# output package information
if(NOT weld_butt_seam_extracting_FIND_QUIETLY)
  message(STATUS "Found weld_butt_seam_extracting: 0.0.0 (${weld_butt_seam_extracting_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'weld_butt_seam_extracting' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT weld_butt_seam_extracting_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(weld_butt_seam_extracting_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${weld_butt_seam_extracting_DIR}/${_extra}")
endforeach()
