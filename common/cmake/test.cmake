## ======================================================================== ##
## Copyright 2009-2018 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

INCLUDE(CTest)

IF (WIN32)
    SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
ELSE()
    SET(MY_PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")
ENDIF()

FIND_PATH(EMBREE_TESTING_MODEL_DIR
  NAMES test-models-intensity2.txt
  PATHS "${PROJECT_SOURCE_DIR}/models"
  DOC "Path to the folder containing the Embree models for regression testing."
  NO_DEFAULT_PATHS)

SET(EMBREE_TESTING_INTENSITY 1 CACHE INT "Intensity of testing (0 = no testing, 1 = verify and tutorials, 2 = light testing, 3 = intensive testing.")
SET(EMBREE_TESTING_MEMCHECK OFF CACHE BOOL "Turns on memory checking for some tests.")
SET(EMBREE_TESTING_BENCHMARK OFF CACHE BOOL "Turns benchmarking on.")
SET(EMBREE_TESTING_BENCHMARK_DATABASE "${PROJECT_BINARY_DIR}" CACHE PATH "Path to database for benchmarking.")
SET(EMBREE_TESTING_PACKAGE OFF CACHE BOOL "Packages release as test.")
SET(EMBREE_TESTING_KLOCWORK OFF CACHE BOOL "Runs Kocwork as test.")
SET(EMBREE_TESTING_SDE OFF CACHE STRING "Uses SDE to run tests for specified CPU.")
SET_PROPERTY(CACHE EMBREE_TESTING_SDE PROPERTY STRINGS OFF pnr nhm wsm snb ivb hsw bdw knl skl skx cnl)

SET(EMBREE_MODEL_DIR "none")
IF (EMBREE_TESTING_MODEL_DIR)
  SET(EMBREE_MODEL_DIR ${EMBREE_TESTING_MODEL_DIR})
ENDIF()

MACRO (ADD_EMBREE_NORMAL_TEST name reference executable args)  
  IF (BUILD_TESTING)  
    ADD_TEST(NAME ${name}
             WORKING_DIRECTORY ${MY_PROJECT_BINARY_DIR}
             COMMAND python ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                     --name ${name}
                     --reference ${reference}
                     --modeldir ${EMBREE_MODEL_DIR}
                     --model default
                     --sde ${EMBREE_TESTING_SDE}
                     --execute ${MY_PROJECT_BINARY_DIR}/${executable} ${args})
  ENDIF()
  
  IF (EMBREE_ISPC_SUPPORT AND EMBREE_RAY_PACKETS)
    IF (BUILD_TESTING)  
      ADD_TEST(NAME ${name}_ispc
               WORKING_DIRECTORY ${MY_PROJECT_BINARY_DIR}
               COMMAND python ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                       --name ${name}_ispc
                       --reference ${reference}
                       --modeldir ${EMBREE_MODEL_DIR}
                       --model default
                       --sde ${EMBREE_TESTING_SDE}
                       --execute ${MY_PROJECT_BINARY_DIR}/${executable}_ispc ${args})
    ENDIF()
  ENDIF()
ENDMACRO()

MACRO (ADD_EMBREE_TEST name)
  ADD_EMBREE_NORMAL_TEST(${name} ${name} ${name} "")
ENDMACRO()

MACRO (ADD_EMBREE_TEST2 name exe args)
  ADD_EMBREE_MODEL_TEST(${name} ${exe} ${exe} "${args}" "default")
ENDMACRO()

MACRO (ADD_EMBREE_MODEL_TEST name reference executable args model)
  IF (BUILD_TESTING)  
    ADD_TEST(NAME ${name}
             WORKING_DIRECTORY ${MY_PROJECT_BINARY_DIR}
             COMMAND python ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                     --name ${name}
                     --reference ${reference}
                     --modeldir ${EMBREE_MODEL_DIR}
                     --model ${model}
                     --sde ${EMBREE_TESTING_SDE}
                     --execute ${MY_PROJECT_BINARY_DIR}/${executable} ${args})
  ENDIF()
  
  IF (EMBREE_ISPC_SUPPORT AND EMBREE_RAY_PACKETS)
    IF (BUILD_TESTING)  
      ADD_TEST(NAME ${name}_ispc
               WORKING_DIRECTORY ${MY_PROJECT_BINARY_DIR}
               COMMAND python ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                       --name ${name}_ispc
                       --reference ${reference}
                       --modeldir ${EMBREE_MODEL_DIR}
                       --model ${model}
                       --sde ${EMBREE_TESTING_SDE}
                       --execute ${MY_PROJECT_BINARY_DIR}/${executable}_ispc ${args})
    ENDIF()
  ENDIF()
ENDMACRO()
  
MACRO (ADD_EMBREE_MODELS_TEST model_list_file name reference executable)
  IF (BUILD_TESTING)  

    SET(full_model_list_file ${EMBREE_TESTING_MODEL_DIR}/${model_list_file})
    
    IF(NOT EXISTS "${full_model_list_file}")
      MESSAGE(FATAL_ERROR "File ${EMBREE_TESTING_MODEL_DIR}/${model_list_file} does not exist!")
    ENDIF()

    FILE(READ "${full_model_list_file}" models)
    STRING(REGEX REPLACE "\n" ";" models "${models}")
    
    FOREACH (model ${models})
      STRING(REGEX REPLACE "/" "_" modelname "${model}")
      STRING(REGEX REPLACE ".ecs" "" modelname "${modelname}")
      ADD_EMBREE_MODEL_TEST(${name}_${modelname} ${reference}_${modelname} ${executable} "${ARGN}" ${model})
    ENDFOREACH()
  ENDIF()
ENDMACRO()

# add klocwork test
IF (EMBREE_TESTING_KLOCWORK)
  ADD_TEST(NAME Klocwork WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PROJECT_SOURCE_DIR}/scripts/klocwork.sh)
  SET_TESTS_PROPERTIES(Klocwork PROPERTIES TIMEOUT 2400)
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
    add_test(NAME ${name} COMMAND ${memcheck_command} ${MY_PROJECT_BINARY_DIR}/${binary} ${ARGN})
  ENDFUNCTION()
ENDIF()
