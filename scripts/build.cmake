## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

message("CTEST_BUILD_OPTIONS = ${CTEST_BUILD_OPTIONS}")

include("${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_test_common.cmake")

# build using as many processes as we have processors
include(ProcessorCount)
ProcessorCount(numProcessors)
if(numProcessors EQUAL 0)
  SET(numProcessors 1)
endif()

# set build name
set(CTEST_CMAKE_GENERATOR "Unix Makefiles")
IF (WIN32)
  set(CTEST_BUILD_COMMAND "${CMAKE_COMMAND} --build . --config ${CTEST_CONFIGURATION_TYPE} -- /m /t:rebuild ")
ELSE()
  set(CTEST_BUILD_COMMAND "make -j ${numProcessors}")
ENDIF()

###################
# execute the test
###################

set(CTEST_CONFIGURE_COMMAND "${CMAKE_COMMAND} ${CTEST_BUILD_OPTIONS} ..")

# start the test
IF (NOT WIN32)
  ctest_empty_binary_directory(${CTEST_BINARY_DIRECTORY})
ENDIF()

ctest_start("Continuous")

ctest_configure(RETURN_VALUE retval)
message("test.cmake: ctest_configure return value = ${retval}")
IF (NOT retval EQUAL 0)
  message(FATAL_ERROR "test.cmake: configure failed")
ENDIF()

ctest_build(RETURN_VALUE retval)
message("test.cmake: ctest_build return value = ${retval}")
IF (NOT retval EQUAL 0)
  message(FATAL_ERROR "test.cmake: build failed")
ENDIF()
