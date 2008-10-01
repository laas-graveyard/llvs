# - robotpkg-config module for CMake
#
# Defines the following macros:
#
# ROBOTPKGCONFIG(package includedir libdir linkflags cflags)
#
# Calling ROBOTPKGCONFIG will fill the desired information into the 4 given arguments,
# e.g. ROBOTPKGCONFIG(walkGenJrl WALKGENJRL_INCLUDE_DIR WALKGENJRL_LINK_DIR WALKGENJRL_LINK_FLAGS WALKGENJRL_CFLAGS)
# if pkg-config was NOT found or the specified software package doesn't exist, the
# variable will be empty when the function returns, otherwise they will contain the respective information
#


  SET(malfilename "MatrixAbstractLayer.pc")
  FIND_PATH(PKG_ROBOTPACKAGE_DIR NAMES ${malfilename} PATHS ${ROBOTPACKAGE_DIR}/lib/pkgconfig )

  #MESSAGE(STATUS "Robot package directory: ${PKG_ROBOTPACKAGE_DIR} ${ROBOTPACKAGE_DIR} ${malfilename}")
  FIND_PROGRAM(ROBOT_PKGCONFIG_EXECUTABLE NAMES pkg-config PATHS ${PKG_ROBOTPACKAGE_DIR})
  SET(ENV{PKG_CONFIG_PATH} "${PKG_ROBOTPACKAGE_DIR}")

  MACRO(ROBOTPKGCONFIG _package _include_DIR _link_DIR _link_FLAGS _cflags)
  # reset the variables at the beginning
    SET(${_include_DIR})
    SET(${_link_DIR})
    SET(${_link_FLAGS})
    SET(${_cflags})
    SET(${_package}_FOUND "1")

    # if pkg-config has been found
    IF(ROBOT_PKGCONFIG_EXECUTABLE)

      #MESSAGE(STATUS " PKG_CONFIG_PATH: $ENV{PKG_CONFIG_PATH}")
      #MESSAGE(STATUS " ${ROBOT_PKGCONFIG_EXECUTABLE} ARGS ${_package}")	
      EXEC_PROGRAM(${ROBOT_PKGCONFIG_EXECUTABLE} ARGS ${_package} --exists RETURN_VALUE _return_VALUE OUTPUT_VARIABLE _pkgconfigDevNull )

      # and if the package of interest also exists for pkg-config, then get the information
      IF(NOT _return_VALUE)
        # MESSAGE(STATUS "${_package} exists !")	
        EXEC_PROGRAM(${ROBOT_PKGCONFIG_EXECUTABLE} ARGS  ${_package} --variable=includedir 
          OUTPUT_VARIABLE ${_include_DIR} )
        # MESSAGE(STATUS "${${_include_DIR}} _include_DIR !")		
        string(REGEX REPLACE "[\r\n]" " " ${_include_DIR} "${${_include_DIR}}")


        EXEC_PROGRAM(${ROBOT_PKGCONFIG_EXECUTABLE} ARGS  ${_package} --variable=libdir 
          OUTPUT_VARIABLE ${_link_DIR} )
        string(REGEX REPLACE "[\r\n]" " " ${_link_DIR} "${${_link_DIR}}")

        EXEC_PROGRAM(${ROBOT_PKGCONFIG_EXECUTABLE} ARGS  ${_package} --libs 
          OUTPUT_VARIABLE ${_link_FLAGS} )
        string(REGEX REPLACE "[\r\n]" " " ${_link_FLAGS} "${${_link_FLAGS}}")

        EXEC_PROGRAM(${ROBOT_PKGCONFIG_EXECUTABLE} ARGS  ${_package} --cflags 
          OUTPUT_VARIABLE ${_cflags} )
        string(REGEX REPLACE "[\r\n]" " " ${_cflags} "${${_cflags}}")

      ENDIF(NOT _return_VALUE)

    ENDIF(ROBOT_PKGCONFIG_EXECUTABLE)

  ENDMACRO(ROBOTPKGCONFIG _include_DIR _link_DIR _link_FLAGS _cflags)

  MARK_AS_ADVANCED(ROBOT_PKGCONFIG_EXECUTABLE)

