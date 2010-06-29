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
	# Project specific module files
	list (
		APPEND 
		CMAKE_MODULE_PATH 
		$ENV{ROBOTPKG_BASE}/share/cmake
	)

	# Dependent packages
	find_package (
		Modules
		COMPONENTS
		image
		architecture
		middleware
		localization
	)
	include(LLVT-toolbox)

#------------------------------------------------------------------------------
# Optional libraries
#------------------------------------------------------------------------------
	# Note that for boost inclusion reason, BtlSlam must be first included.
	ADD_LLV_OPTION(BtlSlam DESCRIPTION "Search for Bristol Visual SLAM." DISABLED)
	ADD_LLV_OPTION(VISP DESCRIPTION "Search for visual servoing plateform." DISABLED)
	ADD_LLV_OPTION(VW)
	ADD_LLV_OPTION(Scene DESCRIPTION "Search for slam davison." DISABLED)
	ADD_LLV_OPTION(VVV DESCRIPTION "Search for aist vision vvv lib.")
	ADD_LLV_OPTION(OmniORB4 DESCRIPTION "Search for corba." OUTPUT_VAR OMNIORB4)
	ADD_LLV_OPTION(OpenCV DESCRIPTION OUTPUT_VAR OPENCV)
	ADD_LLV_OPTION(MMX DESCRIPTION DISABLED)
	ADD_LLV_OPTION(NMBT DESCRIPTION "Search for model tracker new version." DISABLED)
	ADD_LLV_OPTION(MBT DESCRIPTION "Search for original model tracker version." DISABLED)
	ADD_LLV_OPTION(DC1394_V2 DESCRIPTION "Seach for firewire dc1394 library version 2."
		             OUTPUT_VAR DC1394_V2)

#------------------------------------------------------------------------------
# Required libraries
#------------------------------------------------------------------------------
	ADD_LLV_PACKAGES(
		OpenHRP
	)

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
