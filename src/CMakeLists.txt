#
# Copyright (c) 2016 CNRS
# Authors: Joseph Mirabel
#
#
# This file is part of hpp-quadcopter
# hpp-quadcopter is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-quadcopter is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-quadcopter  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

ADD_EXECUTABLE (hpp-quadcopter-server
  flat-path.cc
  main.cc
)

# Use CXX 11
set_property(TARGET hpp-quadcopter-server PROPERTY CXX_STANDARD 11)
set_property(TARGET hpp-quadcopter-server PROPERTY CXX_STANDARD_REQUIRED ON)

# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-quadcopter-server hpp-corbaserver)

# Install executable
INSTALL (TARGETS hpp-quadcopter-server DESTINATION ${CMAKE_INSTALL_BINDIR})
