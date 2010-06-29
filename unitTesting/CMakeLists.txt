#
# Authors:
# Claire Dune 
#
#############################################################################

# Build process propagation in the sub directories
# If you add/remove a directory, modify here
SUBDIRS(
  ModelTracker
  ViSP
  PointTracker
  Asser3D
)

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)

#############################################################################
# ADD_UNIT_TEST(<source1> <source2> ...)
#
# Useful macro to serialize test creation
# <source> variables corresponds to the 
# list of all the sources to build binaries.
# The generated binary name comes by removing 
# the .cpp extension to the source name.

macro(ADD_UNIT_TEST)

	# rule for binary build
	foreach(source ${ARGN})

		# Compute the name of the binary to create
		get_filename_component(binary ${source} NAME_WE)

		# Create the binary
		ADD_LLV_CLIENT(${binary} SOURCES ${source} 
  	               LINK_WITH ${${PROJECT_NAME}_LIBRARY_NAME})
	 
		# Add test
		add_test(${binary} ${binary})

	endforeach(source)

	# customize clean target 
	set_directory_properties(PROPERTIES 
		ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
	)

endmacro(ADD_UNIT_TEST)
