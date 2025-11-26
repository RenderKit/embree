## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

FILE(GLOB PRIMITIVE_TESTS "${PROJECT_SOURCE_DIR}/tests/primitives/*.xml")
SET(SHADERS eyelight;uv;Ng;primID;occlusion)

FOREACH(xml ${PRIMITIVE_TESTS})
  FOREACH(shader ${SHADERS})
    GET_FILENAME_COMPONENT(FN ${xml} NAME_WE)
    SET(testname "prim_${FN}_${shader}")
    SET(compare_threshold "55")

    # TODO: APPLE with AVX2 enabled have a sphere cap intersection problem with linear curves
    IF ((testname MATCHES "prim_curves_linear_flat.*") AND ((testname MATCHES ".*uv") OR testname MATCHES ".*Ng"))
      IF (APPLE AND (EMBREE_ISA_AVX2 OR EMBREE_ISA_AVX512))
        SET(compare_threshold "200")
      ENDIF()
    ENDIF()

    # TODO: linear_flat_mblur and linear_flat_far_mblur tests have different results on sycl - issue: 2531
    IF ((testname MATCHES "prim_curves_linear_flat.*mblur.*") AND ((testname MATCHES ".*uv") OR (testname MATCHES ".*Ng") OR (testname MATCHES ".*eyelight")))
      IF (EMBREE_SYCL_SUPPORT)
        SET(compare_threshold "700")
      ENDIF()
    ENDIF()

    # For unkown hardware on public CI
    IF (testname MATCHES "prim_curves_bezier_normal_oriented_mblur_occlusion")
      SET(compare_threshold "60")
    ENDIF()

    ADD_EMBREE_TEST_ECS(${testname} embree_viewer XML ${xml} 
      REFERENCE_SUFFIX _${shader}
      ARGS
        --shader ${shader} 
        --time 0.5 
        --compare-threshold ${compare_threshold}
    )
  ENDFOREACH()
ENDFOREACH()


IF (EMBREE_TESTING_INSTALL_TESTS)
  # test resources
  INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/primitives" DESTINATION "${CMAKE_INSTALL_TESTDIR}/tests" COMPONENT testing PATTERN "*.py" EXCLUDE)
  INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/models" DESTINATION "${CMAKE_INSTALL_TESTDIR}/tests" COMPONENT testing PATTERN "*.py" EXCLUDE)

  # generated file for creating the testing project in install destination
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/testproject-CMakeLists.txt"
    "# Copyright 2009-2021 Intel Corporation\n"
    "# SPDX-License-Identifier: Apache-2.0\n"
    "# This file was automatically generated\n"
    "cmake_minimum_required(VERSION 3.10)\n"
    "\n"
    "project(embree-testing)\n"
    "\n"
    "# allowing EMBREE_TESTING_INTENSITY to be overriden\n"
    "unset(TESTING_INTENSITY_SAVE CACHE)\n"
    "set(TESTING_INTENSITY_SAVE \${EMBREE_TESTING_INTENSITY})\n"
    "unset(EMBREE_TESTING_INTENSITY CACHE)\n"
    "include(\"../${CMAKE_INSTALL_LIBDIR}/cmake/embree-${EMBREE_VERSION}/embree-config.cmake\")\n"
    "if (DEFINED TESTING_INTENSITY_SAVE)\n"
    "  set(EMBREE_TESTING_INTENSITY \${TESTING_INTENSITY_SAVE})\n"
    "endif()\n"
    "\n"
    "\n"
    "SET(EMBREE_TESTING_PACKAGE_TEST_PROJECT ON)\n"
    "SET(EMBREE_INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR})\n"
    "SET(BUILD_TESTING ON)\n"
    "include(test.cmake)\n"
    "\n"
    "\n"
    "if (EMBREE_TESTING_FILTER_TESTS_FILE)\n"
    "  if (EXISTS \"\${EMBREE_TESTING_FILTER_TESTS_FILE}\")\n"
    "    file(READ \"\${EMBREE_TESTING_FILTER_TESTS_FILE}\" lines)\n"
    "    string(REGEX REPLACE \"\\n\" \";\" testnames \"\${lines}\")\n"
    "    list(APPEND EMBREE_TESTING_FILTER_TESTNAMES \${testnames})\n"
    "  else()\n"
    "    message(FATAL_ERROR \"EMBREE_TESTING_FILTER_TESTS_FILE\" was specified as '\${EMBREE_TESTING_FILTER_TESTS_FILE}' but file could not be opened.)\n"
    "  endif()\n"
    "endif()\n"
    "\n"
    "message(\"filter:                            \\\"${EMBREE_TESTING_FILTER_TESTNAMES}\\\"\")\n"
    "message(\"BUILD_TESTING:                     \\\"\${BUILD_TESTING}\\\"\")\n"
    "message(\"EMBREE_TESTING_INSTALL_TESTS:      \\\"\${EMBREE_TESTING_INSTALL_TESTS}\\\"\")\n"
    "message(\"EMBREE_SYCL_SUPPORT:               \\\"\${EMBREE_SYCL_SUPPORT}\\\"\")\n"
    "message(\"EMBREE_TESTING_INTENSITY:          \\\"\${EMBREE_TESTING_INTENSITY}\\\"\")\n"
    "message(\"EMBREE_TESTING_ONLY_SYCL_TESTS:    \\\"\${EMBREE_TESTING_ONLY_SYCL_TESTS}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_TRIANGLE:          \\\"\${EMBREE_GEOMETRY_TRIANGLE}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_QUAD:              \\\"\${EMBREE_GEOMETRY_QUAD}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_GRID:              \\\"\${EMBREE_GEOMETRY_GRID}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_CURVE:             \\\"\${EMBREE_GEOMETRY_CURVE}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_SUBDIVISION:       \\\"\${EMBREE_GEOMETRY_SUBDIVISION}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_USER:              \\\"\${EMBREE_GEOMETRY_USER}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_INSTANCE:          \\\"\${EMBREE_GEOMETRY_INSTANCE}\\\"\")\n"
    "message(\"EMBREE_GEOMETRY_INSTANCE_ARRAY:    \\\"\${EMBREE_GEOMETRY_INSTANCE_ARRAY}\\\"\")\n"
    "\n"
    "include(embree-addtests.cmake)\n"
  )
  INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/testproject-CMakeLists.txt"
            DESTINATION "${CMAKE_INSTALL_TESTDIR}"
            RENAME "CMakeLists.txt"
            COMPONENT testing)

  FILE(GLOB packfiles "${PROJECT_SOURCE_DIR}/tests/gta-configs/*")
  foreach(f ${packfiles})
    INSTALL(FILES "${f}"
              DESTINATION "${CMAKE_INSTALL_TESTDIR}"
              COMPONENT testing)
  endforeach()

  # generated file adding tests
  INSTALL(FILES "${EMBREE_INSTALL_CTESTTESTFILE}"
            DESTINATION "${CMAKE_INSTALL_TESTDIR}"
            COMPONENT testing)

  # needed for adding tests in install destination
  INSTALL(FILES "${PROJECT_SOURCE_DIR}/common/cmake/test.cmake"
            DESTINATION "${CMAKE_INSTALL_TESTDIR}"
            COMPONENT testing)

ENDIF()
