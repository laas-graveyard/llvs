#############################################################################
#
# Copyright JRL, CNRS/AIST, 2009
#
# Description:
# Try to find VISP library
#
# Once run this will define: 
#
# VISP_FOUND
# VISP_INCLUDE_DIR
# VISP_LIBRARIES
# VISP_CXX_FLAGS
# VISP_LD_FLAGS
#
# Authors:
# Claire Dune
#
#############################################################################

# search for the the file in
FIND_PATH(VISP_DIR
	NAMES VISPConfig.cmake 
    	PATHS  $ENV{ROBOTPKG_BASE}/share/cmake
               $ENV{ROBOTPKG_BASE}/lib
    )

# if VISP has been found     
IF(VISP_DIR)
    SET(VISP_FOUND TRUE)
    INCLUDE(${VISP_DIR}/VISPConfig.cmake)
ELSE(VISP_DIR)
    MESSAGE("ERROR FindVISP.cmake>> ViSP library hasn't been found")
    SET(VISP_FOUND FALSE)
ENDIF(VISP_DIR)


# Status message to check if everything is ok 
MESSAGE(STATUS "ViSP Found : ${VISP_FOUND}")
MESSAGE(STATUS "ViSP path  : ${VISP_DIR}")

