# Copyright 2009-2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0
# This file was automatically generated
cmake_minimum_required(VERSION 3.5)

project(embree-testing)

# allowing EMBREE_TESTING_INTENSITY to be overriden
unset(TESTING_INTENSITY_SAVE CACHE)
set(TESTING_INTENSITY_SAVE ${EMBREE_TESTING_INTENSITY})
unset(EMBREE_TESTING_INTENSITY CACHE)
include("../lib/cmake/embree-4.3.3/embree-config.cmake")
if (DEFINED TESTING_INTENSITY_SAVE)
  set(EMBREE_TESTING_INTENSITY ${TESTING_INTENSITY_SAVE})
endif()


SET(EMBREE_TESTING_PACKAGE_TEST_PROJECT ON)
SET(EMBREE_INSTALL_BINDIR bin)
SET(BUILD_TESTING ON)
include(test.cmake)


if (EMBREE_TESTING_FILTER_TESTS_FILE)
  if (EXISTS "${EMBREE_TESTING_FILTER_TESTS_FILE}")
    file(READ "${EMBREE_TESTING_FILTER_TESTS_FILE}" lines)
    string(REGEX REPLACE "\n" ";" testnames "${lines}")
    list(APPEND EMBREE_TESTING_FILTER_TESTNAMES ${testnames})
  else()
    message(FATAL_ERROR "EMBREE_TESTING_FILTER_TESTS_FILE" was specified as '${EMBREE_TESTING_FILTER_TESTS_FILE}' but file could not be opened.)
  endif()
endif()

message("filter:                            \"\"")
message("BUILD_TESTING:                     \"${BUILD_TESTING}\"")
message("EMBREE_TESTING_INSTALL_TESTS:      \"${EMBREE_TESTING_INSTALL_TESTS}\"")
message("EMBREE_SYCL_SUPPORT:               \"${EMBREE_SYCL_SUPPORT}\"")
message("EMBREE_TESTING_INTENSITY:          \"${EMBREE_TESTING_INTENSITY}\"")
message("EMBREE_TESTING_ONLY_SYCL_TESTS:    \"${EMBREE_TESTING_ONLY_SYCL_TESTS}\"")
message("EMBREE_GEOMETRY_TRIANGLE:          \"${EMBREE_GEOMETRY_TRIANGLE}\"")
message("EMBREE_GEOMETRY_QUAD:              \"${EMBREE_GEOMETRY_QUAD}\"")
message("EMBREE_GEOMETRY_GRID:              \"${EMBREE_GEOMETRY_GRID}\"")
message("EMBREE_GEOMETRY_CURVE:             \"${EMBREE_GEOMETRY_CURVE}\"")
message("EMBREE_GEOMETRY_SUBDIVISION:       \"${EMBREE_GEOMETRY_SUBDIVISION}\"")
message("EMBREE_GEOMETRY_USER:              \"${EMBREE_GEOMETRY_USER}\"")
message("EMBREE_GEOMETRY_INSTANCE:          \"${EMBREE_GEOMETRY_INSTANCE}\"")
message("EMBREE_GEOMETRY_INSTANCE_ARRAY:    \"${EMBREE_GEOMETRY_INSTANCE_ARRAY}\"")

include(embree-addtests.cmake)
