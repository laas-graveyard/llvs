# Copyright (C) 2010 Claire Dune, Thomas Moulard, CNRS-AIST JRL, CNRS/AIST.
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

SUBDIRS(ModelTracker ViSP PointTracker Asser3D)


#############################################################################
# ADD_UNIT_TEST(<source1> <source2> ...)
#
# Useful macro to serialize test creation
# <source> variables corresponds to the
# list of all the sources to build binaries.
# The generated binary name comes by removing
# the .cpp extension to the source name.
MACRO(ADD_UNIT_TEST)
  FOREACH(source ${ARGN})
    # Compute the name of the binary to create.
    GET_FILENAME_COMPONENT(binary ${source} NAME_WE)

    ADD_LLV_CLIENT(${binary} SOURCES ${source}
      LINK_WITH ${${PROJECT_NAME}_LIBRARY_NAME})

    ADD_TEST(${binary} ${binary})
  ENDFOREACH(source)
ENDMACRO(ADD_UNIT_TEST)
