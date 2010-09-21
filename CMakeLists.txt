# Copyright (C) 2010 Claire Dune, Clement Petit, Thomas Moulard,
#		     Olivier Stasse, CNRS-AIST JRL, CNRS/AIST.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Require at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

INCLUDE(warning.cmake)

# Meta-information.
PROJECT(LLVS CXX) # FIXME: lowercase that.
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION "the Low Lever Vision Server")
SET(PROJECT_EMAIL "hpp@laas.fr")
SET(PROJECT_URL "https://jrlserver.muse.aist.go.jp/")


# For retro-compatibility.
SET(${PROJECT_NAME}_LIBRARY_NAME llvs)
SET(${PROJECT_NAME}_BINARY_NAME llvs-server)
#  OpenCV had been for a long time an optional library and used
#  LLVS_HAVE_OPENCV macro to enable/disable some code parts. Now the
#  library became required for anything to work. All macros have to be
#  removed from code. Then cmake compatibility lines below can be
#  deleted.
SET(LLVS_HAVE_OPENCV 1)


# Enable warnings.
CXX_WARNINGS()

# Be verbose by default.
SET(CMAKE_VERBOSE_MAKEFILE TRUE)


# Search for dependencies.
FIND_PACKAGE(
  Modules
  COMPONENTS
  image
  architecture
  middleware
  localization
  algo
  )

INCLUDE(LLVT-toolbox)
INCLUDE(install-toolbox)


# Optional libraries.
# Note that for boost inclusion reason, BtlSlam must be first included.
ADD_PACKAGE_OPTION(HrpBtlSlam DESCRIPTION
  "Search for Bristol Visual SLAM for HRP2." DISABLED)
ADD_PACKAGE_OPTION(VISP DISABLED)
ADD_PACKAGE_OPTION(VW DISABLED)
ADD_PACKAGE_OPTION(Scene DESCRIPTION "Search for slam davison." DISABLED)
ADD_PACKAGE_OPTION(VVV DESCRIPTION "Search for aist vision vvv lib." DISABLED)
ADD_PACKAGE_OPTION(OmniORB4 DESCRIPTION "Search for corba." OUTPUT_VAR OMNIORB4)
ADD_PACKAGE_OPTION(MMX DESCRIPTION DISABLED)
ADD_PACKAGE_OPTION(DC1394_V2 DESCRIPTION
  "Search for firewire dc1394 library version 2." OUTPUT_VAR DC1394_V2)
ADD_PACKAGE_OPTION(KalmanFilter DISABLED)

OPTION(COMPILE_UNIT_TESTS "Compile also unit tests" OFF)

# Required libraries.
ADD_PACKAGES(OpenHRP OpenCV)


# Add uninstall rule.
ADD_UNINSTALL_TARGET()

# Generate a pkg-config file.
ADD_PKGCONFIG_FILE()


# Include subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(scripts)
ADD_SUBDIRECTORY(src)
IF(COMPILE_UNIT_TESTS)
  ADD_SUBDIRECTORY(unitTesting)
ENDIF(COMPILE_UNIT_TESTS)
