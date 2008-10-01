##############################################################################
#
# Copyright JRL, CNRS/AIST, 2008
# 
# Description:
# Try to find VW
# capabilities.
# Once run this will define: 
#
# VW_FOUND
# VW_CXX_FLAGS
# VW_LINK_FLAGS
#
# Authors:
# Olivier Stasse
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindVVV.cmake: only available for Unix.")
  SET(VW_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(VW_HOME NAMES include/VW/geomcompute.h
    PATHS /home/grxuser /home/grxuser/OS/HRP2VisionSystem
    $ENV{HOME}/src )
    		
  # MESSAGE(STATUS "VW: ${VW_HOME}")    
  ## --------------------------------
     
  IF(VW_HOME)
    SET(VW_FOUND TRUE)
  ELSE(VW_HOME})
    SET(VW_FOUND FALSE)
  ENDIF(VW_HOME)

  IF(VW_FOUND)
#    IF(OMNIORB4_FOUND)
#      SET(OPENHRP_CXX_FLAGS "-DCOMMERCIAL -Wall -Wunused ")
#      SET(OPENHRP_LDD_FLAGS "")
#    ENDIF(OMNIORB4_FOUND)
  ENDIF(VW_FOUND)
  
  set(VW_CXX_FLAGS "-I${VW_HOME}/include")
  set(VW_LD_FLAGS "-L${VW_HOME}/lib -lMonoSLAM -lGeometry -lImageProcessing -lmmx  -lSceneImproc -lScene -lintervals -lVWGL -lVW -lVNL -lGL -lGLU -lglut" )
  MESSAGE(STATUS "VW: ${VW_HOME} ${VW_FOUND}")
ENDIF(NOT UNIX)
