##-----------------------------------------------------------------------------
## Copyright 2008, JRL, CNRS/AIST
## Author  : Olivier Stasse
## Creation: 09/09/2008
## 
## Logs:
## 2010/06/10 (Claire Dune) use append instead of set 
##                          and use path list instead
##                          of CXX flags
##
## 2010/06/29 (Clement Petit) Use generic JRL CMakeModule
##                            tools and LLVT.
##-----------------------------------------------------------------------------

#------------------------------------------------------------------------------
# Project information
#------------------------------------------------------------------------------
project (LLVS)
cmake_minimum_required (VERSION 2.6.4)
set (${PROJECT_NAME}_LIBRARY_NAME llvs-0)
set (${PROJECT_NAME}_BINARY_NAME LLVS_server)

#------------------------------------------------------------------------------
# Specify the cmake path and option
#------------------------------------------------------------------------------
# Dependent packages
find_package (
  Modules
  COMPONENTS
  image
  architecture
  middleware
  localization
  algo
  )

include(LLVT-toolbox)

#------------------------------------------------------------------------------
# Optional libraries
#------------------------------------------------------------------------------
# Note that for boost inclusion reason, BtlSlam must be first included.
ADD_PACKAGE_OPTION(HrpBtlSlam DESCRIPTION "Search for Bristol Visual SLAM for HRP2." DISABLED)
ADD_PACKAGE_OPTION(VISP DISABLED)
ADD_PACKAGE_OPTION(VW DISABLED)
ADD_PACKAGE_OPTION(Scene DESCRIPTION "Search for slam davison." DISABLED)
ADD_PACKAGE_OPTION(VVV DESCRIPTION "Search for aist vision vvv lib." DISABLED)
ADD_PACKAGE_OPTION(OmniORB4 DESCRIPTION "Search for corba." OUTPUT_VAR OMNIORB4)
ADD_PACKAGE_OPTION(MMX DESCRIPTION DISABLED)
ADD_PACKAGE_OPTION(DC1394_V2 DESCRIPTION "Search for firewire dc1394 library version 2."
  OUTPUT_VAR DC1394_V2)
ADD_PACKAGE_OPTION(KalmanFilter DISABLED)

#------------------------------------------------------------------------------
# Required libraries
#------------------------------------------------------------------------------
ADD_PACKAGES(
  OpenHRP
  OpenCV
  )

#------------------------------------------------------------------------------
# FIXME: Optional to required library transition
#------------------------------------------------------------------------------
# OpenCV had been for a long time an optional library and used LLVS_HAVE_OPENCV
# macro to enable/disable some code parts. Now the library became required for
# anything to work. All macros have to be removed from code. Then cmake
# compatibility lines below can be deleted.
set(LLVS_HAVE_OPENCV 1)

#------------------------------------------------------------------------------
# Propagation in sub dirs
#------------------------------------------------------------------------------
add_subdirectory(include)
add_subdirectory(src)

#------------------------------------------------------------------------------
# Unit tests
#------------------------------------------------------------------------------
option(COMPILE_UNIT_TESTS "Compile also unit tests" OFF)
if(COMPILE_UNIT_TESTS)
  add_subdirectory(unitTesting)
endif(COMPILE_UNIT_TESTS)

#------------------------------------------------------------------------------
# Add uninstall target
#------------------------------------------------------------------------------
include(install-toolbox)
ADD_UNINSTALL_TARGET()
ADD_PKGCONFIG_FILE()
