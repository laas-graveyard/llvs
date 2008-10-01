# Olivier Stasse, JRL, CNRS/AIST
# Creation: 02/07/2008
# History:
#
# Copyright CNRS/AIST

MACRO(ADD_OPENHRP_PLUGIN PLUGINNAME PLUGINNAME_SRCS PLUGINCORBAIDL WORKINGDIRIDL PLUGINLINKS CORBAPLUGINDEPS)

GET_FILENAME_COMPONENT(PluginBaseName ${PLUGINNAME} NAME)
GET_FILENAME_COMPONENT(PluginBasePath ${PLUGINNAME} PATH)
GET_FILENAME_COMPONENT(PluginBaseNameIDL ${PLUGINCORBAIDL} NAME_WE)

SET(name_ofplugincorbaIDL ${PLUGINCORBAIDL})
#MESSAGE(STATUS ":name_ofplugincorbaIDL:" ${name_ofplugincorbaIDL})

SET(all_sourcefiles_for_plugin ${PLUGINNAME_SRCS})

# Build the path of the target
SET(${plugin_final_position} "${openhrp_final_plugin_path}/${PLUGINNAME}")

IF (NOT ${PLUGINNAME_SRCS})
   SET(PLUGINNAME_SRCS "${PluginBasePath}/${PluginBaseName}.cpp")
ENDIF (NOT ${PLUGINNAME_SRCS})

#MESSAGE(STATUS "Sources files for ${PLUGINNAME} : ${PLUGINNAME_SRCS} ")

SET(IDL_INCLUDE_DIR ${OPENHRP_HOME}/Common/corba )

IF   ( EXISTS "${name_ofplugincorbaIDL}" )


  SET(plugincorbaidl_CPP "${WORKINGDIRIDL}/${PluginBaseNameIDL}SK.cc")
  SET(plugincorbaidl_Header "${WORKINGDIRIDL}/${PluginBaseNameIDL}.hh")
  IDLFILERULE(${PLUGINCORBAIDL} ${plugincorbaidl_CPP} ${plugincorbaidl_Header} ${WORKINGDIRIDL}
              ${IDL_INCLUDE_DIR})

  SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${plugincorbaidl_CPP})

  SET(common_CPP "${WORKINGDIRIDL}/commonSK.cc")
  SET(common_Header "${WORKINGDIRIDL}/common.hh" )	
  
  IDLFILERULE(${OPENHRP_HOME}/Common/corba/common.idl 
               ${common_CPP}
               ${common_Header} ${WORKINGDIRIDL} 
	       ${IDL_INCLUDE_DIR})

  SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${common_CPP})
ELSE ( EXISTS "${name_ofplugincorbaIDL}" )
  MESSAGE(STATUS "${name_ofplugincorbaIDL} empty")
ENDIF( EXISTS "${name_ofplugincorbaIDL}" )

FOREACH( corbaplugin ${CORBAPLUGINDEPS})

  GET_FILENAME_COMPONENT(corbaplugin_basename ${corbaplugin} NAME_WE)
  SET(corbaplugin_CPP "${WORKINGDIRIDL}/${corbaplugin_basename}SK.cc")
  SET(corbaplugin_Header "${WORKINGDIRIDL}/${corbaplugin_basename}.hh" )	
  IDLFILERULE(${corbaplugin}
              ${corbaplugin_CPP}
              ${corbaplugin_Header} ${WORKINGDIRIDL}  
              ${IDL_INCLUDE_DIR})
  SET(all_sourcefiles_for_plugin ${all_sourcefiles_for_plugin} ${corbaplugin_CPP})

ENDFOREACH(corbaplugin)
#MESSAGE(STATUS "all_sourcefiles_for_plugin : ${all_sourcefiles_for_plugin}")
#MESSAGE(STATUS "PluginName : ${PLUGINNAME}")
ADD_LIBRARY(${PluginBaseName} ${all_sourcefiles_for_plugin})

SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} ${OPENHRP_CXX_FLAGS}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/IOserver/robot/${ROBOT}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/IOserver/include")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Common")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/common")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${OPENHRP_HOME}/Controller/IOserver/sys/plugin")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${Boost_INCLUDE_DIRS}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${WORKINGDIRIDL}")
SET(openhrp_plugin_cflags "${openhrp_plugin_cflags} -I${SOT_CXX_FLAGS}")

SET(openhrp_plugin_path "${OPENHRP_HOME}/Controller/IOserver/robot/${ROBOT}/bin")
SET(openhrp_plugin_filenamepath "${openhrp_plugin_path}/${PluginBaseName}.so")

#MESSAGE(STATUS "openhrp_plugin_filenamepath ${openhrp_plugin_filenamepath}")
GET_TARGET_PROPERTY(PluginFinalFileNamePrefix ${PluginBaseName} PREFIX)
GET_TARGET_PROPERTY(PluginFinalFileNameSuffix ${PluginBaseName} SUFFIX)	              
#MESSAGE(STATUS "Prefix: ${PluginFinalFileNamePrefix} Suffix: ${PluginFinalFileNameSuffix}")

SET_TARGET_PROPERTIES(${PluginBaseName}
			PROPERTIES	
		        COMPILE_FLAGS "${omniORB4_cflags} ${openhrp_plugin_cflags}"
			LINK_FLAGS "${omniORB4_link_FLAGS} ${PLUGINLINKS}"
			PREFIX ""
			SUFFIX ".so"
			LIBRARY_OUTPUT_DIRECTORY ${openhrp_plugin_path})

#SET(AlwaysCopyPlugin "${PluginBaseName}AlwaysCopyPlugin")
#ADD_CUSTOM_TARGET( ${AlwaysCopyPlugin} ALL
#	           ${CMAKE_COMMAND} -E copy ${LIBRARY_OUTPUT_PATH}/${PluginBaseName}.so ${openhrp_plugin_filenamepath}
#		   DEPENDS ${PluginBaseName}
#		   )

ENDMACRO(ADD_OPENHRP_PLUGIN)