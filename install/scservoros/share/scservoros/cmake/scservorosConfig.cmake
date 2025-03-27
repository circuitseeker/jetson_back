# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_scservoros_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED scservoros_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(scservoros_FOUND FALSE)
  elseif(NOT scservoros_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(scservoros_FOUND FALSE)
  endif()
  return()
endif()
set(_scservoros_CONFIG_INCLUDED TRUE)

# output package information
if(NOT scservoros_FIND_QUIETLY)
  message(STATUS "Found scservoros: 0.0.1 (${scservoros_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'scservoros' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${scservoros_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(scservoros_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${scservoros_DIR}/${_extra}")
endforeach()
