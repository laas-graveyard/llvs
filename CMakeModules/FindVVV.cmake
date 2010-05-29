##############################################################################
#
# Copyright JRL, CNRS/AIST, 2008
# 
# Description:
# Try to find VVV
# capabilities.
# Once run this will define: 
#
# VVV_FOUND
# VVV_CXX_FLAGS
# VVV_LINK_FLAGS
#
# Authors:
# Olivier Stasse
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindVVV.cmake: only available for Unix.")
  SET(VVV_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(VVV_HOME NAMES include/vvvconf.h
    PATHS /usr/local/VVV
    $ENV{HOME}/src )
    		
  # MESSAGE(STATUS "OpenHRP: ${OPENHRP_HOME}")    
  ## --------------------------------
     
  IF(VVV_HOME)
    SET(VVV_FOUND TRUE)
  ELSE(VVV_HOME})
    SET(VVV_FOUND FALSE)
  ENDIF(VVV_HOME)

  
  set(VVV_CXX_FLAGS "-I${VVV_HOME}/include")
  set(VVV_LD_FLAGS "-L${VVV_HOME}/lib   -ldisparitymap -lpebutil  -lisoluminance  -lvfgb  -lrange -lTUIeee1394++ -lTUTools++ -lraw1394  -lepbmtobrep  -lbrep -lvvvstereo -lstepone -lcalib -lscm -llcm  -lepbm -lbstereo -lstepfifteen -lvvvstd -lmatutil" )
  MESSAGE(STATUS "VVV: ${VVV_HOME} ${VVV_FOUND}")
ENDIF(NOT UNIX)
