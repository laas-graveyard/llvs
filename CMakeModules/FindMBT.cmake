MESSAGE("---- Find MBT ---- ")

# detection of the Libpng headers location
  FIND_PATH(MBT_INCLUDE_PATH 
    NAMES
    mbtRobustTracking.h
    PATHS
    $ENV{ROBOTPKG_BASE}/include/mbt
    )
 MESSAGE("MBT_INCLUDE_DIR=${MBT_INCLUDE_DIR}")

  # Detection of the Libmbtlibrary on Unix
 FIND_LIBRARY(MBT_LIBRARY
    NAMES
    libmbt.a
    PATHS
     $ENV{ROBOTPKG_BASE}/lib
    )
 MESSAGE("MBT_LIBRARY=${MBT_LIBRARY}")


  MARK_AS_ADVANCED(
    MBT_LINK_DIR 
    MBT_INCLUDE_DIR
  )
  
  # Load the configuration file
  FIND_PATH(MBT_DIR 
    NAMES 
    MBTConfig.cmake
    PATHS 
    $ENV{ROBOTPKG_BASE}/lib
    )

MESSAGE("MBT_DIR=${MBT_DIR}")
  include(${MBT_DIR}/MBTConfig.cmake)


## --------------------------------
  
IF(MBT_LIBRARY AND MBT_INCLUDE_DIR)
  SET(MBT_LINK_DIR "${MBT_DIR}")
  SET(MBT_FOUND TRUE)
  SET(MBT_LIB_NAME "mbt")
ELSE(MBT_LIBRARY AND MBT_INCLUDE_DIR)
  SET(MBT_FOUND FALSE)
ENDIF(MBT_LIBRARY AND MBT_INCLUDE_DIR)
