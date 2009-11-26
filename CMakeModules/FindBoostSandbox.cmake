# - Find the Boost Sandbox includes and libraries.
# The following variables are set if BoostSandbox is found.  If BoostSandbox is not
# found, BoostSandbox_FOUND is set to false.
#  BoostSandbox_FOUND        - True when the BoostSandbox include directory is found.
#  BoostSandbox_INCLUDE_DIRS - the path to where the boost include files are.
#  BoostSandbox_LIBRARY_DIRS - The path to where the boost library files are.
#  BoostSandbox_LIB_DIAGNOSTIC_DEFINITIONS - Only set if using Windows.

# ----------------------------------------------------------------------------
# If you have installed BoostSandbox in a non-standard location or you have
# just staged the boost files using bjam then you have three
# options. In the following comments, it is assumed that <Your Path>
# points to the root directory of the include directory of BoostSandbox. e.g
# If you have put boost in C:\development\BoostSandbox then <Your Path> is
# "C:/development/BoostSandbox" and in this directory there will be two
# directories called "include" and "lib".
# 1) After CMake runs, set BoostSandbox_INCLUDE_DIR to <Your Path>/include/boost<-version>
# 2) Use CMAKE_INCLUDE_PATH to set a path to <Your Path>/include. This will allow FIND_PATH()
#    to locate BoostSandbox_INCLUDE_DIR by utilizing the PATH_SUFFIXES option. e.g.
#    SET(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "<Your Path>/include")
# 3) Set an environment variable called ${BOOST_ROOT} that points to the root of where you have
#    installed BoostSandbox, e.g. <Your Path>. It is assumed that there is at least a subdirectory called
#    include in this path.
#
# Note:
#  1) If you are just using the boost headers, then you do not need to use
#     BoostSandbox_LIBRARY_DIRS in your CMakeLists.txt file.
#  2) If BoostSandbox has not been installed, then when setting BoostSandbox_LIBRARY_DIRS
#     the script will look for /lib first and, if this fails, then for /stage/lib.
#
# Usage:
# In your CMakeLists.txt file do something like this:
# ...
# # BoostSandbox
# FIND_PACKAGE(BoostSandbox)
# ...
# INCLUDE_DIRECTORIES(${BoostSandbox_INCLUDE_DIRS})
# LINK_DIRECTORIES(${BoostSandbox_LIBRARY_DIRS})
#
# In Windows, we make the assumption that, if the BoostSandbox files are installed, the default directory
# will be C:\boost.

#
# TODO:
#
# 1) Automatically find the BoostSandbox library files and eliminate the need
#    to use Link Directories.
#

IF(WIN32)
  # In windows, automatic linking is performed, so you do not have to specify the libraries.
  # If you are linking to a dynamic runtime, then you can choose to link to either a static or a
  # dynamic BoostSandbox library, the default is to do a static link.  You can alter this for a specific
  # library "whatever" by defining BOOSTSANDBOX_WHATEVER_DYN_LINK to force BoostSandbox library "whatever" to
  # be linked dynamically.  Alternatively you can force all BoostSandbox libraries to dynamic link by
  # defining BOOSTSANDBOX_ALL_DYN_LINK.

  # This feature can be disabled for BoostSandbox library "whatever" by defining BOOSTSANDBOX_WHATEVER_NO_LIB,
  # or for all of BoostSandbox by defining BOOSTSANDBOX_ALL_NO_LIB.

  # If you want to observe which libraries are being linked against then defining
  # BOOSTSANDBOX_LIB_DIAGNOSTIC will cause the auto-linking code to emit a #pragma message each time
  # a library is selected for linking.
  SET(BoostSandbox_LIB_DIAGNOSTIC_DEFINITIONS "-DBOOSTSANDBOX_LIB_DIAGNOSTIC")
ENDIF(WIN32)


SET(BOOSTSANDBOX_INCLUDE_PATH_DESCRIPTION "directory containing the boost include files. E.g /usr/local/include/boost-sandbox or c:\\boost\\include\\boost-sandbox")

SET(BOOSTSANDBOX_DIR_MESSAGE "Set the BoostSandbox_INCLUDE_DIR cmake cache entry to the ${BOOSTSANDBOX_INCLUDE_PATH_DESCRIPTION}")

SET(BOOSTSANDBOX_DIR_SEARCH $ENV{BOOSTSANDBOX_ROOT})
IF(BOOSTSANDBOX_DIR_SEARCH)
  FILE(TO_CMAKE_PATH ${BOOSTSANDBOX_DIR_SEARCH} BOOSTSANDBOX_DIR_SEARCH)
  SET(BOOSTSANDBOX_DIR_SEARCH ${BOOSTSANDBOX_DIR_SEARCH}/include)
ENDIF(BOOSTSANDBOX_DIR_SEARCH)

IF(WIN32)
  SET(BOOSTSANDBOX_DIR_SEARCH
    ${BOOSTSANDBOX_DIR_SEARCH}
    C:/boost-sandbox/include
    D:/boost-sandbox/include
  )
ENDIF(WIN32)

# Add in some path suffixes. These will have to be updated whenever a new BoostSandbox version comes out.
SET(SUFFIX_FOR_PATH
 boost-sandbox
)

#
# Look for an installation.
#
FIND_PATH(BoostSandbox_INCLUDE_DIR NAMES boost/typeof/config.hpp PATH_SUFFIXES ${SUFFIX_FOR_PATH} PATHS

  # Look in other places.
  ${BOOSTSANDBOX_DIR_SEARCH}

  # Help the user find it if we cannot.
  DOC "The ${BOOSTSANDBOX_INCLUDE_PATH_DESCRIPTION}"
)

# Assume we didn't find it.
SET(BoostSandbox_FOUND 0)

# Now try to get the include and library path.
IF(BoostSandbox_INCLUDE_DIR)

  # Look for the boost library path.
  # Note that the user may not have installed any libraries
  # so it is quite possible the BoostSandbox_LIBRARY_PATH may not exist.
  SET(BoostSandbox_LIBRARY_DIR ${BoostSandbox_INCLUDE_DIR})

  IF("${BoostSandbox_LIBRARY_DIR}" MATCHES "boost-sandbox/boost-[0-9]+")
    GET_FILENAME_COMPONENT(BoostSandbox_LIBRARY_DIR ${BoostSandbox_LIBRARY_DIR} PATH)
  ENDIF ("${BoostSandbox_LIBRARY_DIR}" MATCHES "boost-sandbox/boost-[0-9]+")

  IF("${BoostSandbox_LIBRARY_DIR}" MATCHES "/include$")
    # Strip off the trailing "/include" in the path.
    GET_FILENAME_COMPONENT(BoostSandbox_LIBRARY_DIR ${BoostSandbox_LIBRARY_DIR} PATH)
  ENDIF("${BoostSandbox_LIBRARY_DIR}" MATCHES "/include$")

  IF(EXISTS "${BoostSandbox_LIBRARY_DIR}/lib")
    SET (BoostSandbox_LIBRARY_DIR ${BoostSandbox_LIBRARY_DIR}/lib)
  ELSE(EXISTS "${BoostSandbox_LIBRARY_DIR}/lib")
    IF(EXISTS "${BoostSandbox_LIBRARY_DIR}/stage/lib")
      SET(BoostSandbox_LIBRARY_DIR ${BoostSandbox_LIBRARY_DIR}/stage/lib)
    ELSE(EXISTS "${BoostSandbox_LIBRARY_DIR}/stage/lib")
      SET(BoostSandbox_LIBRARY_DIR "")
    ENDIF(EXISTS "${BoostSandbox_LIBRARY_DIR}/stage/lib")
  ENDIF(EXISTS "${BoostSandbox_LIBRARY_DIR}/lib")

  IF(EXISTS "${BoostSandbox_INCLUDE_DIR}")
    SET(BoostSandbox_INCLUDE_DIRS ${BoostSandbox_INCLUDE_DIR})
    # We have found boost. It is possible that the user has not
    # compiled any libraries so we set BoostSandbox_FOUND to be true here.
    SET(BoostSandbox_FOUND 1)
  ENDIF(EXISTS "${BoostSandbox_INCLUDE_DIR}")

  IF(BoostSandbox_LIBRARY_DIR AND EXISTS "${BoostSandbox_LIBRARY_DIR}")
    SET(BoostSandbox_LIBRARY_DIRS ${BoostSandbox_LIBRARY_DIR})
  ENDIF(BoostSandbox_LIBRARY_DIR AND EXISTS "${BoostSandbox_LIBRARY_DIR}")
ENDIF(BoostSandbox_INCLUDE_DIR)

IF(NOT BoostSandbox_FOUND)
  IF(NOT BoostSandbox_FIND_QUIETLY)
    MESSAGE(STATUS "BoostSandbox was not found. ${BOOSTSANDBOX_DIR_MESSAGE}")
  ELSE(NOT BoostSandbox_FIND_QUIETLY)
    IF(BoostSandbox_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "BoostSandbox was not found. ${BOOSTSANDBOX_DIR_MESSAGE}")
    ENDIF(BoostSandbox_FIND_REQUIRED)
  ENDIF(NOT BoostSandbox_FIND_QUIETLY)
ENDIF(NOT BoostSandbox_FOUND)

