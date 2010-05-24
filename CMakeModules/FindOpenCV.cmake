##############################################################################
#
# Copyright JRL, CNRS/AIST, 2008
# 
# Description:
# Try to find OpenCV
# capabilities.
# Once run this will define: 
#
# OPENCV_FOUND
# OPENCV_CXX_FLAGS
# OPENCV_LINK_FLAGS
#
# Authors:
# Olivier Stasse
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindVVV.cmake: only available for Unix.")
  SET(OPENCV_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(OPENCV_HOME NAMES include/opencv/cv.h
    PATHS /usr/local/
    $ENV{ROBOTPKG_BASE}
    $ENV{HOME}/src )
    		
  # MESSAGE(STATUS "VW: ${VW_HOME}")    
  ## --------------------------------
     
  IF(OPENCV_HOME)
    SET(OPENCV_FOUND TRUE)
  ELSE(OPENCV_HOME})
    SET(OPENCV_FOUND FALSE)
  ENDIF(OPENCV_HOME)

  IF(OPENCV_FOUND)
#    IF(OMNIORB4_FOUND)
#      SET(OPENHRP_CXX_FLAGS "-DCOMMERCIAL -Wall -Wunused ")
#      SET(OPENHRP_LDD_FLAGS "")
#    ENDIF(OMNIORB4_FOUND)
  ENDIF(OPENCV_FOUND)
  
  set(OPENCV_CXX_FLAGS "-I${OPENCV_HOME}/include/opencv -D__OPENCV__")
  set(OPENCV_LD_FLAGS "-L${OPENCV_HOME}/lib -lcv" )
  MESSAGE(STATUS "OpenCV: ${OPENCV_HOME} ${OPENCV_FOUND}")
ENDIF(NOT UNIX)
