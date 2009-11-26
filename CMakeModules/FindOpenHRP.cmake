##############################################################################
#
# Copyright JRL, CNRS/AIST, 2008
# 
# Description:
# Try to find OpenHRP/Make.Rules
# capabilities.
# Once run this will define: 
#
# OpenHRP_FOUND
# OpenHRP_DIR
#
# Authors:
# Olivier Stasse
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindOpenHRP.cmake: only available for Unix.")
  SET(OPENHRP_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(OPENHRP_HOME NAMES Make.vars
    PATHS $ENV{OPENHRPHOME}
    $ENV{HOME}/src )
    		
  # MESSAGE(STATUS "OpenHRP: ${OPENHRP_HOME}")    
  ## --------------------------------
     
  IF(OPENHRP_HOME)
    SET(OPENHRP_FOUND TRUE)
  ELSE(OPENHRP_HOME)
    SET(OPENHRP_FOUND FALSE)
  ENDIF(OPENHRP_HOME)

  IF(OPENHRP_FOUND)
    IF(OMNIORB4_FOUND)
      SET(OPENHRP_CXX_FLAGS "-DCOMMERCIAL -Wall -Wunused ")
      SET(OPENHRP_LDD_FLAGS "")
    ENDIF(OMNIORB4_FOUND)
  ENDIF(OPENHRP_FOUND)
  
  SET(ROBOT HRP2JRL)
  SET(${openhrp_final_plugin_path} "${OPENHRP_HOME}/Controller/IOserver/robot/${ROBOT}/bin")
  
  # MESSAGE(STATUS "OpenHRP: ${OPENHRP_HOME}")
ENDIF(NOT UNIX)
