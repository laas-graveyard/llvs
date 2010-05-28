##############################################################################
#
# Copyright JRL, CNRS/AIST, 2010
# 
# Description:
# Try to find NMBT
# capabilities.
# Once run this will define: 
#
# NMBT_FOUND
# NMBT_CXX_FLAGS
# NMBT_LINK_FLAGS
#
# Authors:
# Claire Dune
#
#############################################################################
IF(NOT UNIX)
  MESSAGE("FindNMBT.cmake: only available for Unix.")
  SET(NMBT_FOUND FALSE)
ELSE(NOT UNIX)
# detection of the Libnmbt headers location
  FIND_PATH(LIBNMBT_INCLUDE_PATH 
    NAMES
    nmbtTracking.h
    PATHS
    $ENV{ROBOTPKG_BASE}/include/nmbt
    )
  #MESSAGE("LIBPNG_HEADER=${LIBPNG_INCLUDE_PATH}")

  # Detection of the Libpng library on Unix
  FIND_LIBRARY(LIBNMBT_LIBRARY
    NAMES
    libnmbt.a
    PATHS
     $ENV{ROBOTPKG_BASE}/lib
    )
  #MESSAGE("LIBPNG_LIBRARY=${LIBPNG_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBNMBT_LIBRARY
    LIBNMBT_INCLUDE_PATH
  )
  
  # Load the configuration file
  FIND_PATH(NMBT_DIR 
    NAMES 
    NMBTConfig.cmake
    PATHS 
    $ENV{ROBOTPKG_BASE}/lib
    )
 # MESSAGE("NMBT_DIR : ${NMBT_DIR}")
  include(${NMBT_DIR}/NMBTConfig.cmake)

## --------------------------------
  
IF(LIBNMBT_LIBRARY AND LIBNMBT_INCLUDE_PATH)
  SET(LIBNMBT_INCLUDE_DIR ${LIBNMBT_INCLUDE_PATH})
  SET(LIBNMBT_LIBRARIES  ${LIBNMBT_LIBRARY})
  SET(NMBT_FOUND TRUE)
ELSE(LIBNMBT_LIBRARY AND LIBNMBT_INCLUDE_PATH)
  SET(NMBT_FOUND FALSE)
ENDIF(LIBNMBT_LIBRARY AND LIBNMBT_INCLUDE_PATH)

  IF(NMBT_FOUND)
    SET(NMBT_CXX_FLAGS "-I${LIBNMBT_INCLUDE_PATH} -D__NMBT__")
    SET(NMBT_LD_FLAGS "-L${LIBNMBT_LIBRARY} -lcv" )
  MESSAGE(STATUS "NmbtTracker:  ${NMBT_FOUND}")
  ENDIF(NMBT_FOUND)


ENDIF(NOT UNIX)