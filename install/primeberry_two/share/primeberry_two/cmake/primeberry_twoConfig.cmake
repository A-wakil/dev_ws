# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_primeberry_two_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED primeberry_two_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(primeberry_two_FOUND FALSE)
  elseif(NOT primeberry_two_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(primeberry_two_FOUND FALSE)
  endif()
  return()
endif()
set(_primeberry_two_CONFIG_INCLUDED TRUE)

# output package information
if(NOT primeberry_two_FIND_QUIETLY)
  message(STATUS "Found primeberry_two: 0.0.0 (${primeberry_two_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'primeberry_two' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${primeberry_two_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(primeberry_two_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${primeberry_two_DIR}/${_extra}")
endforeach()
