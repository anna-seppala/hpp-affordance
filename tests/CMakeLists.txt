# Copyright 2016 CNRS-LAAS
#
# Author: Anna Seppala
#
# This file is part of hpp-affordance
# hpp-affordance is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-affordance is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-affordance  If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

# Make Boost.Test generates the main function in test cases.
#ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
macro(add_testcase NAME GENERATED)
	IF (${GENERATED} STREQUAL TRUE)
	add_executable(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME.cc)
	  ELSE()
    add_executable(${NAME} ${NAME}.cc)
  ENDIF()
  add_test(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-fcl)
	PKG_CONFIG_USE_DEPENDENCY(${NAME} eigen3)

	target_link_libraries(${NAME}
    ${Boost_LIBRARIES}
    ${PROJECT_NAME}
		hpp-affordance
    )
endmacro(add_testcase)

# add_affordance_test(test-create-mesh test-create-mesh.cc)
# add_affordance_test(test-create-affordance test-main3.cc)

add_testcase (test-operations FALSE)
add_testcase (test-one-triangle1 FALSE)
add_testcase (test-one-triangle2 FALSE)
add_testcase (test-oriented-triangles FALSE)
