# Copyright 2010, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of llvs.
# llvs is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# llvs is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# llvs. If not, see <http://www.gnu.org/licenses/>.

# MANPAGE
# -------
#
# Generate a pod man page from a template, then
# generate the man page and compress it.
# This macro also adds the installation rules.
#
MACRO(MANPAGE NAME)
  FIND_PROGRAM(POD2MAN pod2man)
  CONFIGURE_FILE(${NAME}.pod.in ${NAME}.pod @ONLY)

  ADD_CUSTOM_COMMAND(
    OUTPUT ${NAME}.1
    COMMAND ${POD2MAN} --section=1
            --center="LOCAL USER COMMANDS"
	    --release ${PROJECT_NAME} ${NAME}.pod
	    > ${NAME}.1
    DEPENDS ${BUILD_DIR}/${NAME}.pod)

  ADD_CUSTOM_COMMAND(
    OUTPUT ${NAME}.1.gz
    COMMAND gzip -c ${NAME}.1 > ${NAME}.1.gz
    DEPENDS ${BUILD_DIR}/${NAME}.1)
  INSTALL(FILES ${BUILD_DIR}/${NAME}.1.gz DESTINATION share/man/man1)
ENDMACRO(MANPAGE)

SET(BUILD_DIR ${CMAKE_BINARY_DIR}/doc)

SET(DOCDIR share/doc/${PROJECT_NAME})

# Add `doc' target.
ADD_CUSTOM_TARGET(
  doc ALL
  DEPENDS llvs-server.1.gz modfirewire.1.gz)

# Man page generation.
MANPAGE(llvs-server)

MANPAGE(modfirewire)
