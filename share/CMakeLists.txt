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

# Vision profiles list.
SET(VISION_PROFILES_DATA
  llvs/profiles/default.vsp
  llvs/profiles/default-wide.vsp
  llvs/profiles/flea2.vsp
  llvs/profiles/hrp2014.vsp
  llvs/profiles/unibrain2.vsp
  llvs/profiles/unibrain-double.vsp
  llvs/profiles/unibrain-single.vsp
  llvs/profiles/unibrain.vsp
  )

# Install vision profiles.
INSTALL(
  FILES ${VISION_PROFILES_DATA}
  DESTINATION share/llvs/profiles
  PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
  )
