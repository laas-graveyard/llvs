# Copyright (C) 2010 Thomas Moulard, CNRS-AIST JRL, CNRS/AIST.
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

MACRO(CXX_WARNINGS)
  INCLUDE(CheckCXXCompilerFlag)
  # Add warnings.
  SET(FLAGS -Wall -Wcast-align -Wcast-qual
            -Wformat -Wwrite-strings -Wconversion
	    -Wmissing-declarations)
  FOREACH(FLAG ${FLAGS})
    CHECK_CXX_COMPILER_FLAG(${FLAG} R${FLAG})
    IF(${R${FLAG}})
      SET(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
    ENDIF(${R${FLAG}})
  ENDFOREACH(FLAG ${FLAGS})
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")
ENDMACRO(CXX_WARNINGS)
