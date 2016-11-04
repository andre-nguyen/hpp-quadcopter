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

SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-quadcopter)
SET(PROJECT_DESCRIPTION
  "Tools for quadcopter motion planning with the humanoid path planner platform."
)
SET(PROJECT_URL "")

FINDPYTHON()

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

ADD_REQUIRED_DEPENDENCY("hpp-core >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-corbaserver >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-util")

ADD_REQUIRED_DEPENDENCY("libglog")

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
  include/hpp/quadcopter/fwd.hh
  include/hpp/quadcopter/flat-path.hh
  include/hpp/quadcopter/steering-method.hh
  include/hpp/quadcopter/configuration-shooter.hh
  )

ADD_SUBDIRECTORY(src)

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  data/package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  data/urdf/quadcopter.urdf
  data/urdf/scene.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  data/srdf/quadcopter.srdf
  data/srdf/scene.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
  data/meshes/quadrotor_base.dae
  data/meshes/scene.stl
  data/meshes/scene1.stl
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )
install (FILES
  src/hpp/quadcopter/robot.py
  src/hpp/quadcopter/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/quadcopter)

install(FILES
  script/quadcopter.py
  DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html/script
  )

SETUP_PROJECT_FINALIZE()
