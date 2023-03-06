## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(CTest)

IF (WIN32)
    IF("${CMAKE_CXX_COMPILER_ID}" MATCHES "MSVC")
      SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
    ELSE()
      SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    ENDIF()
ELSE()
    SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
ENDIF()

FIND_PATH(EMBREE_TESTING_MODEL_DIR
  NAMES test-models-intensity2.txt
  PATHS "${PROJECT_SOURCE_DIR}/models"
  DOC "Path to the folder containing the Embree models for regression testing."
  NO_DEFAULT_PATHS)

SET(EMBREE_TESTING_INTENSITY 1 CACHE STRING "Intensity of testing (0 = no testing, 1 = verify and tutorials, 2 = light testing, 3 = intensive testing, 4 = very intensive testing.")
SET_PROPERTY(CACHE EMBREE_TESTING_INTENSITY PROPERTY STRINGS 0 1 2 3 4)
SET(EMBREE_TESTING_ONLY_SYCL_TESTS OFF CACHE BOOL "Run only tests with the sycl support.")
SET(EMBREE_TESTING_MEMCHECK OFF CACHE BOOL "Turns on memory checking for some tests.")
SET(EMBREE_TESTING_BENCHMARK OFF CACHE BOOL "Turns benchmarking on.")
SET(EMBREE_TESTING_BENCHMARK_DATABASE "${PROJECT_BINARY_DIR}" CACHE PATH "Path to database for benchmarking.")
SET(EMBREE_TESTING_PACKAGE OFF CACHE BOOL "Packages release as test.")
SET(EMBREE_TESTING_KLOCWORK OFF CACHE BOOL "Runs Kocwork as test.")
SET(EMBREE_TESTING_SDE OFF CACHE STRING "Uses SDE to run tests for specified CPU.")
SET_PROPERTY(CACHE EMBREE_TESTING_SDE PROPERTY STRINGS OFF pnr nhm wsm snb ivb hsw bdw knl skl skx cnl)

# checks if the current cmake configuration is compatible with the requirements declared in the optionsfile
MACRO (EMBREE_TEST_CHECK_OPTIONS optionsfile out)  

  # if no optionsfile is found return true (no requirements defined)
  SET(${out} 1)

  IF(EXISTS "${optionsfile}")
    FILE(READ "${optionsfile}" lines)
    STRING(REGEX REPLACE "\n" ";" lines "${lines}")

    FOREACH (line ${lines})

      # parse line into list
      string(REGEX MATCHALL "([^\ ]+\ |[^\ ]+$)" tokens "${line}")
      LIST(LENGTH tokens token_count)
      IF (NOT ${token_count} EQUAL 3)
        message(FATAL_ERROR "illegal .embree_opitons in ${optionsfile} at line ${line}")
      ENDIF()

      # we require every line to follow the scheme "option [==/!=] value"
      LIST(GET tokens 0 option)
      LIST(GET tokens 1 comp)
      LIST(GET tokens 2 value)

      STRING(STRIP ${option} option)
      STRING(STRIP ${comp} comp)
      STRING(STRIP ${value} value)

      IF ("${comp}" STREQUAL "==")
        IF (NOT "${${option}}" STREQUAL "${value}")
          SET(${out} 0)
        ENDIF()
      ELSE()
        IF ("${${option}}" STREQUAL "${value}")
          SET(${out} 0)
        ENDIF()
      ENDIF()

      IF (${out} EQUAL 0)
        BREAK()
      ENDIF()

    ENDFOREACH()
  ENDIF()

ENDMACRO()

MACRO (ADD_EMBREE_GENERIC_CPP_TEST testname executable args)  
  IF (BUILD_TESTING)
    ADD_TEST(NAME ${testname}
             WORKING_DIRECTORY "${MY_PROJECT_BINARY_DIR}"
             COMMAND ${executable} ${args})
  ENDIF()
ENDMACRO()

MACRO (ADD_EMBREE_GENERIC_ISPC_TEST testname executable args)  
  IF (BUILD_TESTING AND EMBREE_ISPC_SUPPORT AND EMBREE_RAY_PACKETS)
    ADD_TEST(NAME ${testname}_ispc
             WORKING_DIRECTORY "${MY_PROJECT_BINARY_DIR}"
             COMMAND ${executable}_ispc ${args})
  ENDIF()       
ENDMACRO()

MACRO (ADD_EMBREE_GENERIC_SYCL_TEST testname executable args)  
  IF (BUILD_TESTING AND EMBREE_SYCL_SUPPORT)
    ADD_TEST(NAME ${testname}_sycl
             WORKING_DIRECTORY ${MY_PROJECT_BINARY_DIR}
             COMMAND ${executable}_sycl ${args})
    SET_TESTS_PROPERTIES(${testname}_sycl PROPERTIES TIMEOUT 50)
  ENDIF()
ENDMACRO()

MACRO (ADD_EMBREE_TEST testname executable args)
  # Check if we should dump the output of this tests
  LIST(FIND EMBREE_TESTING_DUMPTESTS ${testname} dumptest)
  if (NOT(EMBREE_TESTING_DUMPFORMAT))
    SET(EMBREE_TESTING_DUMPFORMAT "tga")
  endif()
  set(DUMP_ARG "")
  IF(NOT(${dumptest} EQUAL -1))
    file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/tests/dump")
    set(argsx "${args} -o ${PROJECT_SOURCE_DIR}/tests/dump/${testname}.${EMBREE_TESTING_DUMPFORMAT}")
  ELSE()
    set(argsx ${args})
  ENDIF()

  # By default add tests for all executables
  SET (extra_args ${ARGN})
  LIST(APPEND extra_args cpp)
  LIST(APPEND extra_args ispc)
  LIST(APPEND extra_args sycl)
  LIST(REMOVE_DUPLICATES extra_args)

  # Exclude exe with '!'
  LIST(FIND extra_args "!cpp" p)
  if (NOT (${p} EQUAL -1))
    LIST(REMOVE_ITEM extra_args "!cpp")
    LIST(REMOVE_ITEM extra_args "cpp")
  endif()

  LIST(FIND extra_args "!ispc" p)
  if (NOT (${p} EQUAL -1))
    LIST(REMOVE_ITEM extra_args "!ispc")
    LIST(REMOVE_ITEM extra_args "ispc")
  endif()

  LIST(FIND extra_args "!sycl" p)
  if (NOT (${p} EQUAL -1))
    LIST(REMOVE_ITEM extra_args "!sycl")
    LIST(REMOVE_ITEM extra_args "sycl")
  endif()

  # Add the tests
  FOREACH(loopvar ${extra_args})
    if((NOT ${EMBREE_SYCL_SUPPORT}) OR (NOT ${EMBREE_TESTING_ONLY_SYCL_TESTS}))
      if(${loopvar} STREQUAL "cpp")
        ADD_EMBREE_GENERIC_CPP_TEST(${testname} ${executable} ${argsx})
      ENDIF()
      if(${loopvar} STREQUAL "ispc")
        ADD_EMBREE_GENERIC_ISPC_TEST(${testname} ${executable} ${argsx})
      ENDIF()
    endif()
    if(${loopvar} STREQUAL "sycl")
      ADD_EMBREE_GENERIC_SYCL_TEST(${testname} ${executable} ${argsx}) 
    ENDIF()
  ENDFOREACH()
ENDMACRO()

MACRO (ADD_EMBREE_TUTORIAL_TEST name args)
  ADD_EMBREE_TEST(${name} embree_${name} "--compare ${PROJECT_SOURCE_DIR}/tests/models/reference/${name}.exr ${args}" ${ARGN})
ENDMACRO()

MACRO (ADD_EMBREE_PRIMITIVE_TEST xml shader)
  EMBREE_TEST_CHECK_OPTIONS("${xml}.embree_options" out)
  IF (${out} EQUAL 1)

    GET_FILENAME_COMPONENT(FN ${xml} NAME_WE)
    SET(testname "prim_${FN}_${shader}")
    SET(compare_threshold "55")

    # TODO: APPLE with AVX2 enabled have a sphere cap intersection problem with linear curves
    IF ((testname MATCHES "prim_curves_linear_flat.*") AND ((testname MATCHES ".*uv") OR testname MATCHES ".*Ng"))
      IF (APPLE AND (EMBREE_ISA_AVX2 OR EMBREE_ISA_AVX512))
        SET(compare_threshold "200")
      ENDIF()
    ENDIF()

    SET(args "-i ${xml} --shader ${shader} --time 0.5 --compare ${PROJECT_SOURCE_DIR}/tests/primitives/reference/${testname}.exr --compare-threshold ${compare_threshold}")
    ADD_EMBREE_TEST(${testname} embree_viewer ${args})

  ENDIF()
ENDMACRO()

MACRO (ADD_EMBREE_MODEL_TEST testname executable model reference args)
  EMBREE_TEST_CHECK_OPTIONS("${PROJECT_SOURCE_DIR}/tests/models/${model}.embree_options" out)
  IF (${out} EQUAL 1)

    SET(extra_args ${ARGN})
    IF ("${testname}" MATCHES ".*subdiv.*")  # skip subdiv models for SYCL mode
      LIST(APPEND extra_args !sycl)
    ENDIF()

    SET(argsx "-c ${PROJECT_SOURCE_DIR}/tests/models/${model} --compare ${PROJECT_SOURCE_DIR}/tests/models/reference/${reference}.exr ${args}")
    ADD_EMBREE_TEST(${testname} ${executable} ${argsx} ${extra_args})

  ENDIF()
ENDMACRO()

MACRO (ADD_EMBREE_MODELS_TEST model_list_file testname executable reference args)
  IF (BUILD_TESTING)  

    SET(full_model_list_file "${PROJECT_SOURCE_DIR}/tests/models/${model_list_file}")
    
    IF(NOT EXISTS "${full_model_list_file}")
      MESSAGE(FATAL_ERROR "File ${PROJECT_SOURCE_DIR}/tests/models/${model_list_file} does not exist!")
    ENDIF()

    FILE(READ "${full_model_list_file}" models)
    STRING(REGEX REPLACE "\n" ";" models "${models}")
    
    FOREACH (model ${models})
      IF (model MATCHES "^#")
        CONTINUE()
      ENDIF()
      IF (model MATCHES ".*mblur.*" OR model MATCHES ".*motion_blur.*")  # skip mblur models if motion blur is not enabled
        IF (NOT EMBREE_SYCL_MBLUR)
          CONTINUE()
        ENDIF()
      ENDIF()
      STRING(REGEX REPLACE " .*" "" modelname "${model}")
      STRING(REGEX REPLACE "/" "_" modelname "${modelname}")
      STRING(REGEX REPLACE ".ecs" "" modelname "${modelname}")

      ADD_EMBREE_MODEL_TEST(${testname}_${modelname} ${executable} ${model} ${reference}_${modelname} "${args}")
    ENDFOREACH()
  ENDIF()
ENDMACRO()

















# add klocwork test
IF (EMBREE_TESTING_KLOCWORK)
  ADD_TEST(NAME Klocwork-Build WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}" COMMAND "${PROJECT_SOURCE_DIR}/scripts/klocwork_build.sh")
  ADD_TEST(NAME Klocwork-Check WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}" COMMAND "${PROJECT_SOURCE_DIR}/scripts/klocwork_check.sh")
  SET_TESTS_PROPERTIES(Klocwork-Build PROPERTIES TIMEOUT 2400)
  SET_TESTS_PROPERTIES(Klocwork-Check PROPERTIES TIMEOUT 300)
ENDIF()

# add valgrind test
IF (EMBREE_TESTING_MEMCHECK)
  find_program( EMBREE_MEMORYCHECK_COMMAND valgrind )
  set( EMBREE_MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full --show-leak-kinds=definite --errors-for-leak-kinds=definite --error-exitcode=1" )
  IF (NOT EMBREE_MEMORYCHECK_COMMAND)
    MESSAGE(FATAL_ERROR "valgrind not found")
  ENDIF()
  FUNCTION(ADD_MEMCHECK_TEST name binary)
    set(memcheck_command "${EMBREE_MEMORYCHECK_COMMAND} ${EMBREE_MEMORYCHECK_COMMAND_OPTIONS}")
    separate_arguments(memcheck_command)
    add_test(NAME ${name} COMMAND ${memcheck_command} "${MY_PROJECT_BINARY_DIR}/${binary}" ${ARGN})
  ENDFUNCTION()
ENDIF()
