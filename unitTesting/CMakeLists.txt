
#
# Authors:
# Claire Dune 
#
#############################################################################

# Set SRC_SUBDIRS variable to all the sub directories we want to parse during
# the build process. 
#
# If you add/remove a directory, modify here
SET (SRC_SUBDIRS
  ModelTracker
)

# Build process propagation in the sub directories
SUBDIRS(${SRC_SUBDIRS})

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)

