# - Try to find Clothoids
# Once done this will define
#
#  CLOTHOIDS_FOUND - system has Clothoids
#  CLOTHOIDS_INCLUDE_DIR - the Clothoids include directory
#  CLOTHOIDS_LIBRARIES - Link these to use Clothoids
#
# Copyright (c) 2006, 2007 Carsten Niehaus, <cniehaus@gmx.de>
# Copyright (C) 2008 Marcus D. Hanwell <marcus@cryos.org>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (CLOTHOIDS_INCLUDE_DIR AND CLOTHOIDS_LIBRARIES)
  # in cache already
  set(CLOTHOIDS_FOUND TRUE)

else (CLOTHOIDS_INCLUDE_DIR AND CLOTHOIDS_LIBRARIES)

  # Extract the directory where *this* file has been installed (determined at cmake run-time)
  # Get the absolute path with no ../.. relative marks, to eliminate implicit linker warnings
  get_filename_component(CLOTHOIDS_CONFIG_PATH "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
  get_filename_component(CLOTHOIDS_INSTALL_PATH "${CLOTHOIDS_CONFIG_PATH}/../../" REALPATH)

  set(__CLOTHOIDS_INCLUDE_DIR "${CLOTHOIDS_INSTALL_PATH}/include")
  foreach(d ${__CLOTHOIDS_INCLUDE_DIR})
    get_filename_component(__d "${d}" REALPATH)
    if(NOT EXISTS "${__d}")
      message(WARNING "Clothoids: Include directory doesn't exist: '${d}'. Clothoids installation may be broken. Skip...")
    else()
      list(APPEND CLOTHOIDS_INCLUDE_DIR "${__d}")
    endif()
  endforeach()
  unset(__d)
  set(CLOTHOIDS_INCLUDE_DIR ${CLOTHOIDS_INCLUDE_DIR} CACHE INTERNAL "") 


  find_library(CLOTHOIDS_LIBRARIES NAMES Clothoids_linux
    PATHS
    HINTS "${CLOTHOIDS_INSTALL_PATH}/lib"
  )


  if (CLOTHOIDS_INCLUDE_DIR AND CLOTHOIDS_LIBRARIES)
    set(CLOTHOIDS_FOUND TRUE)
  endif(CLOTHOIDS_INCLUDE_DIR AND CLOTHOIDS_LIBRARIES)

  if (CLOTHOIDS_FOUND)
    message(STATUS "Found Clothoids Library: ${CLOTHOIDS_LIBRARIES}")
  else (CLOTHOIDS_FOUND)
    if (CLOTHOIDS_FIND_REQUIRED)
      message(FATAL_ERROR "Could NOT find Clothoids")
    endif (CLOTHOIDS_FIND_REQUIRED)
  endif (CLOTHOIDS_FOUND)

  mark_as_advanced(CLOTHOIDS_INCLUDE_DIR CLOTHOIDS_LIBRARIES)

endif (CLOTHOIDS_INCLUDE_DIR AND CLOTHOIDS_LIBRARIES)


