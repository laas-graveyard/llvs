#############################################################################
#
# Copyright JRL, CNRS/AIST, 2009
#
# Description:
# Try to find VISP library
#
# Once run this will define: 
#
# NMBT_FOUND : flag to say if the file has been detected
# NMBT_INCLUDE_DIR : include dir of all the lib nmbt is using
# NMBT_LIBRARIES : names of the libraries
# NMBT_LINK_DIRECTORIES : path to the lib
# NMBT_SOURCE_DIR : path to the source
# 
# Authors:
# Claire Dune
#
#############################################################################

# search for the the file in
FIND_PATH(NMBT_DIR 
          NAMES 
	  	NMBTConfig.cmake 
    	  PATHS  
	  	 $ENV{ROBOTPKG_BASE}/share/cmake
           	 $ENV{ROBOTPKG_BASE}/lib 
    	)

# if NMBT has been found     
IF(NMBT_DIR)
    SET(NMBT_FOUND TRUE)
    INCLUDE(${NMBT_DIR}/NMBTConfig.cmake)
ELSE(MNBT_DIR)
    SET(NMBT_FOUND FALSE)
ENDIF(NMBT_DIR)

# Status message to check if everything is ok 
MESSAGE(STATUS "NMBT Found :  ${NMBT_FOUND}")
MESSAGE(STATUS "NMBT path  : ${NMBT_DIR}")


