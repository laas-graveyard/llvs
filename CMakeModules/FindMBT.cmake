##############################################################################
#
# Copyright JRL, CNRS/AIST, 2010
# 
# Description:
# Try to find NMBT
# capabilities.
# Once run this will define: 
#
# MBT_FOUND
# MBT_CXX_FLAGS
# MBT_LINK_FLAGS
#
# Authors:
# Claire Dune
#
#############################################################################
IF(NOT UNIX)
  MESSAGE("FindMBT.cmake: only available for Unix.")
  SET(MBT_FOUND FALSE)


ELSE(NOT UNIX)
# detection of the Libmbt headers location
  FIND_PATH(LIBMBT_INCLUDE_PATH 
    NAMES
    mbtTracking.h
    PATHS
    $ENV{ROBOTPKG_BASE}/include/nmbt
    )


  # Detection of the Libpng library on Unix
  FIND_LIBRARY(LIBMBT_LIBRARY
    NAMES
    libmbt.a
    PATHS
     $ENV{ROBOTPKG_BASE}/lib
    )
  #MESSAGE("LIBPNG_LIBRARY=${LIBPNG_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBMBT_LIBRARY
    LIBMBT_INCLUDE_PATH
  )
  
  # Load the configuration file
  FIND_PATH(MBT_DIR 
    NAMES 
    NMBTConfig.cmake
    PATHS 
    $ENV{ROBOTPKG_BASE}/lib
    )
 # MESSAGE("MBT_DIR : ${MBT_DIR}")
  include(${MBT_DIR}/MBTConfig.cmake)

## --------------------------------
  
IF(LIBMBT_LIBRARY AND LIBMBT_INCLUDE_PATH)
  SET(LIBMBT_INCLUDE_DIR ${LIBMBT_INCLUDE_PATH})
  SET(LIBMBT_LIBRARIES  ${LIBMBT_LIBRARY})
  SET(NMBT_FOUND TRUE)
ELSE(LIBMBT_LIBRARY AND LIBMBT_INCLUDE_PATH)
  SET(MBT_FOUND FALSE)
ENDIF(LIBMBT_LIBRARY AND LIBMBT_INCLUDE_PATH)

  IF(MBT_FOUND)
    SET(MBT_CXX_FLAGS "-I${LIBMBT_INCLUDE_PATH} -D__MBT__")
    SET(MBT_LD_FLAGS "-L${LIBMBT_LIBRARY} -lcv" )
  MESSAGE(STATUS "MbtTracker:  ${MBT_FOUND}")
  ENDIF(MBT_FOUND)


ENDIF(NOT UNIX)