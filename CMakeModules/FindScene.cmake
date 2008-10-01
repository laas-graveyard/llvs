##############################################################################
#
# Copyright JRL, CNRS/AIST, 2008
# 
# Description:
# Try to find Scene/scene_single.h
# capabilities.
# Once run this will define: 
#
# SCENE_FOUND
# SCENE_CXX_FLAGS
# SCENE_LINK_FLAGS
# Authors:
# Olivier Stasse
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindOpenHRP.cmake: only available for Unix.")
  SET(SCENE_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(SCENE_HOME NAMES Scene/kalman.h
    PATHS /home/grxuser/OS/HRP2VisionSystem/include/nScene
    $ENV{HOME}/src )
    		
  # MESSAGE(STATUS "SCENE: ${SCENE_HOME}")    
  ## --------------------------------
     
  IF(SCENE_HOME)
    SET(SCENE_FOUND TRUE)
  ELSE(SCENE_HOME)
    SET(SCENE_FOUND FALSE)
  ENDIF(SCENE_HOME)

  IF(SCENE_FOUND)
    IF(SCENE_FOUND)
      SET(SCENE_CXX_FLAGS "-I${SCENE_HOME} ")
      SET(SCENE_LDD_FLAGS "")
    ENDIF(SCENE_FOUND)
  ENDIF(SCENE_FOUND)
  
  MESSAGE(STATUS "SCENE: ${SCENE_HOME} ${SCENE_FOUND}")
ENDIF(NOT UNIX)
