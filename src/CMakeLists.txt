# Copyright 2010, François Bleibel, Olivier Stasse, JRL, CNRS/AIST,
#                 Florent Lamiraux (CNRS/LAAS)
#
# This file is part of sot-dyninv.
# sot-dyninv is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-dyninv is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-dyninv. If not, see <http://www.gnu.org/licenses/>.

INCLUDE(../cmake/python.cmake)

# prefix and suffix each element of list by ${prefix}elemnt${suffix}
macro(ADDPREFIX newlist prefix list_name)
    # create empty list - necessary?
    SET(${newlist})

     # prefix and suffix elements
    foreach(l ${${list_name}})
      list(APPEND ${newlist} ${prefix}${l} )
    endforeach()

endmacro(ADDPREFIX)

ADDPREFIX(${PROJECT_NAME}_ABSOLUTE_HEADERS  "${CMAKE_SOURCE_DIR}/"  ${PROJECT_NAME}_HEADERS)

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  dg-imu-attitude-estimation.cc
  dg-imu-model-free-flex-estimation.cc
  dg-imu-model-base-flex-estimation.cc
  input_reconstructor.cc
  calibrate.cc
  stack-of-contacts.cc
  odometry.cc
  estimator-interface.cc
  from-local-to-global-frame.cc
  filter.cc
  moving-frame-transformation.cc
  position-state-reconstructor.cc
  drift-from-mocap.cc
  stop-watch.cpp
  ${${PROJECT_NAME}_ABSOLUTE_HEADERS}
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# provide path to dependency libraries
LINK_DIRECTORIES(${STATE_OBSERVATION_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_DEPENDENCIES(${LIBRARY_NAME} state-observation sot-core dynamic-graph dynamic-graph-python)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} state-observation sot-core dynamic-graph dynamic-graph-python)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} state-observation)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)


INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${LIBRARY_NAME})

SET(NEW_ENTITY_CLASS
  )

DYNAMIC_GRAPH_PYTHON_MODULE("sot/application/state_observation" ${LIBRARY_NAME} wrap)


###
# PYTHON PART
###

#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

INCLUDE(../cmake/python.cmake)
FINDPYTHON ()

INSTALL(FILES
  dynamic_graph/sot/application/state_observation/initializations/__init__.py
  dynamic_graph/sot/application/state_observation/initializations/hrp2_model_free_flex_estimator.py
  dynamic_graph/sot/application/state_observation/initializations/hrp2_model_base_flex_estimator.py
  dynamic_graph/sot/application/state_observation/initializations/hrp2_model_base_flex_estimator_imu_force.py
  dynamic_graph/sot/application/state_observation/initializations/hrp2_model_base_flex_estimator_imu_force_encoders.py
  dynamic_graph/sot/application/state_observation/initializations/hrp2_mocap_to_sot.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/application/state_observation/initializations
  )
