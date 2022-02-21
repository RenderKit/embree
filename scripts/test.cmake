## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

message("CTEST_BUILD_OPTIONS = ${CTEST_BUILD_OPTIONS}")

# project and repository to clone from
SET(CTEST_PROJECT_NAME "Embree")
SET(TEST_REPOSITORY "https://github.com/embree/embree.git")

# set directories
set(CTEST_SOURCE_DIRECTORY "${CTEST_SCRIPT_DIRECTORY}/..")
set(CTEST_BINARY_DIRECTORY "${CTEST_SCRIPT_DIRECTORY}/../build")
SET(TEST_MODELS_PARENT_DIRECTORY $ENV{HOME})
SET(TEST_MODELS_DIRECTORY ${TEST_MODELS_PARENT_DIRECTORY}/embree-models)

# fix slashes under windows
IF (WIN32)
  STRING(REPLACE "\\" "/" CTEST_SOURCE_DIRECTORY "${CTEST_SOURCE_DIRECTORY}")
  STRING(REPLACE "\\" "/" CTEST_BINARY_DIRECTORY "${CTEST_BINARY_DIRECTORY}")
  STRING(REPLACE "\\" "/" TEST_MODELS_PARENT_DIRECTORY "${TEST_MODELS_PARENT_DIRECTORY}")
  STRING(REPLACE "\\" "/" TEST_MODELS_DIRECTORY "${TEST_MODELS_DIRECTORY}")
ENDIF()

MESSAGE("CTEST_SOURCE_DIRECTORY = ${CTEST_SOURCE_DIRECTORY}")
MESSAGE("CTEST_BINARY_DIRECTORY = ${CTEST_BINARY_DIRECTORY}")
MESSAGE("TEST_MODELS_DIRECTORY = ${TEST_MODELS_DIRECTORY}")
MESSAGE("http_proxy = $ENV{http_proxy}")
MESSAGE("https_proxy = $ENV{https_proxy}")
MESSAGE("no_proxy = $ENV{no_proxy}")
MESSAGE("PATH = $ENV{PATH}")

# update external model repository
FIND_PROGRAM(CTEST_GIT_COMMAND NAMES git)

MACRO(check_return_code)
  IF (NOT "${retcode}" STREQUAL "0")
    MESSAGE(FATAL_ERROR "error executing process")
  ENDIF()
ENDMACRO()

# macro that updates the test models
MACRO(update_test_models)
  IF(NOT EXISTS "${TEST_MODELS_DIRECTORY}")
    MESSAGE("cloning test models ...")
    EXECUTE_PROCESS(
      COMMAND ${CTEST_GIT_COMMAND} "clone" "git@vis-gitolite:embree-models" embree-models
      WORKING_DIRECTORY ${TEST_MODELS_PARENT_DIRECTORY}
      RESULT_VARIABLE retcode)
    check_return_code()
  ELSE()
    MESSAGE("updating test models ...")
    EXECUTE_PROCESS(
      COMMAND ${CTEST_GIT_COMMAND} "fetch"
      WORKING_DIRECTORY ${TEST_MODELS_DIRECTORY}
      RESULT_VARIABLE retcode)
    check_return_code()
  ENDIF()
  IF (NOT TEST_MODELS_HASH)
    MESSAGE(FATAL_ERROR "no TEST_MODELS_HASH set")
  ENDIF()
  MESSAGE("checking out test models: ${TEST_MODELS_HASH}")
  EXECUTE_PROCESS(
    COMMAND ${CTEST_GIT_COMMAND} "checkout" "-f" ${TEST_MODELS_HASH}
    WORKING_DIRECTORY ${TEST_MODELS_DIRECTORY}
    RESULT_VARIABLE retcode)
  check_return_code()
ENDMACRO()

##################################
# configure and build            #
##################################
MACRO(build)

  include(ProcessorCount)
  ProcessorCount(numProcessors)
  if(numProcessors EQUAL 0)
    SET(numProcessors 1)
  endif()

  if (${THREADS} EQUAL 0)
    SET(THREADS ${numProcessors})
  endif()

  set(CTEST_CMAKE_GENERATOR "Unix Makefiles")
  IF (WIN32)
    set(CTEST_BUILD_COMMAND "${CMAKE_COMMAND} --build . -j ${THREADS} --config ${CTEST_CONFIGURATION_TYPE} ${BUILD_SUFFIX}")
  ELSE()
    set(CTEST_BUILD_COMMAND "make -j ${THREADS}")
  ENDIF()

  IF (NOT WIN32)
    ctest_empty_binary_directory(${CTEST_BINARY_DIRECTORY})
  ENDIF()

  ctest_start("Continuous")

  set(CTEST_CONFIGURE_COMMAND "${CMAKE_COMMAND} ${CTEST_BUILD_OPTIONS} ..")
  ctest_configure(RETURN_VALUE retval)
  IF (NOT retval EQUAL 0)
    message(FATAL_ERROR "test.cmake: configure failed")
  ENDIF()

  ctest_build(RETURN_VALUE retval)
  message("test.cmake: ctest_build return value = ${retval}")
  IF (NOT retval EQUAL 0)
    message(FATAL_ERROR "test.cmake: build failed")
  ENDIF()
ENDMACRO()

##################################
# configure and execute the test #
##################################
MACRO(test)
  ctest_start("Continuous")

  IF (EMBREE_UPDATE_MODELS AND EMBREE_TESTING_INTENSITY GREATER 0)
    update_test_models()
  ENDIF()

  set(CTEST_CONFIGURE_COMMAND "${CMAKE_COMMAND} -DBUILD_TESTING:BOOL=ON -DEMBREE_TESTING_MODEL_DIR:PATH=${TEST_MODELS_DIRECTORY} -DEMBREE_TESTING_INTENSITY=${EMBREE_TESTING_INTENSITY} ..")
  ctest_configure()

  IF (EMBREE_TESTING_INTENSITY GREATER 0 OR EMBREE_TESTING_KLOCWORK)
    ctest_test(RETURN_VALUE retval)
    message("test.cmake: ctest_test return value = ${retval}")
    IF (NOT retval EQUAL 0)
      message(FATAL_ERROR "test.cmake: some tests failed")
    ENDIF()
  ENDIF()
ENDMACRO()

# increase default output sizes for test outputs
IF (NOT DEFINED CTEST_CUSTOM_MAXIMUM_PASSED_TEST_OUTPUT_SIZE)
  SET(CTEST_CUSTOM_MAXIMUM_PASSED_TEST_OUTPUT_SIZE 100000)
ENDIF()
IF(NOT DEFINED CTEST_CUSTOM_MAXIMUM_FAILED_TEST_OUTPUT_SIZE)
  SET(CTEST_CUSTOM_MAXIMUM_FAILED_TEST_OUTPUT_SIZE 800000)
ENDIF()

IF (${STAGE} STREQUAL "build")
  build()
ELSEIF (${STAGE} STREQUAL "test")
  test()
ELSE ()
  message("unknown stage ${STAGE}. Should be \"build\" or \"test\"")
ENDIF()

