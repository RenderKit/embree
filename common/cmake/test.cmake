## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
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

IF(NOT BUILD_TESTING)
  SET(BUILD_TESTING OFF CACHE BOOL "Build the testing tree.")
ENDIF()

INCLUDE(CTest)

IF (BUILD_TESTING)

  CMAKE_HOST_SYSTEM_INFORMATION(RESULT memory QUERY TOTAL_PHYSICAL_MEMORY)
  
  SET(BUILD_TESTING_MODEL_DIR "${PROJECT_SOURCE_DIR}/models" CACHE FILEPATH "Path to the folder containing the Embree models for regression testing.")
  SET(BUILD_TESTING_INTENSIVE OFF CACHE BOOL "Turns on intensive testing.")

  IF(   NOT EXISTS "${BUILD_TESTING_MODEL_DIR}/embree-models-subdiv.txt"
     OR NOT EXISTS "${BUILD_TESTING_MODEL_DIR}/embree-models-small-win32.txt"
     OR NOT EXISTS "${BUILD_TESTING_MODEL_DIR}/embree-models-small-x64.txt"
     OR NOT EXISTS "${BUILD_TESTING_MODEL_DIR}/embree-models-large.txt")
    MESSAGE(FATAL_ERROR "Invalid Embree testing model repository. Either disable BUILD_TESTING or properly set BUILD_TESTING_MODEL_DIR.")
  ENDIF()
  
  FILE(READ "${BUILD_TESTING_MODEL_DIR}/embree-models-subdiv.txt" models_subdiv)
  STRING(REGEX REPLACE "\n" ";" models_subdiv "${models_subdiv}")
  
  FILE(READ "${BUILD_TESTING_MODEL_DIR}/embree-models-small-win32.txt" models_small_win32)
  STRING(REGEX REPLACE "\n" ";" models_small_win32 "${models_small_win32}")
  
  FILE(READ "${BUILD_TESTING_MODEL_DIR}/embree-models-small-x64.txt" models_small_x64)
  STRING(REGEX REPLACE "\n" ";" models_small_x64 "${models_small_x64}")
  
  FILE(READ "${BUILD_TESTING_MODEL_DIR}/embree-models-large.txt" models_large)
  STRING(REGEX REPLACE "\n" ";" models_large "${models_large}")

  IF (BUILD_TESTING_INTENSIVE)
    SET(models ${models_small_x64})
    IF (${memory} GREATER 10000) 
      SET(models ${models} ${models_large})
    ENDIF()
  ELSE()
    SET(models ${models_small_win32})
  ENDIF()
  
  MACRO (ADD_EMBREE_MODEL_TEST name reference executable args model)
  
    ADD_TEST(NAME ${name}
             WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
             COMMAND ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                     --name ${name}
                     --modeldir ${BUILD_TESTING_MODEL_DIR}
                     --reference ${reference}
                     --model ${model}
                     --execute ${PROJECT_BINARY_DIR}/${executable} ${args})
                     
    IF (ENABLE_ISPC_SUPPORT AND RTCORE_RAY_PACKETS)
      ADD_TEST(NAME ${name}_ispc
               WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
               COMMAND ${PROJECT_SOURCE_DIR}/scripts/invoke_test.py
                       --name ${name}_ispc
                       --modeldir ${BUILD_TESTING_MODEL_DIR}
                       --reference ${reference}
                       --model ${model}
                       --execute ${PROJECT_BINARY_DIR}/${executable}_ispc ${args})
    ENDIF()
  ENDMACRO()
  
  MACRO (ADD_EMBREE_MODELS_TEST name reference executable)
    FOREACH (model ${models})
      STRING(REGEX REPLACE "/" "_" modelname "${model}")
      STRING(REGEX REPLACE ".ecs" "" modelname "${modelname}")
      ADD_EMBREE_MODEL_TEST(${name}_${modelname} ${reference}_${modelname} ${executable} "${ARGN}" ${model})
    ENDFOREACH()
  ENDMACRO()

  MACRO (ADD_EMBREE_SUBDIV_MODELS_TEST name reference executable)
    FOREACH (model ${models_subdiv})
      STRING(REGEX REPLACE "/" "_" modelname "${model}")
      STRING(REGEX REPLACE ".ecs" "" modelname "${modelname}")
      ADD_EMBREE_MODEL_TEST(${name}_${modelname} ${reference}_${modelname} ${executable} "${ARGN}" ${model})
    ENDFOREACH()
  ENDMACRO()

ELSE()

  MACRO (ADD_EMBREE_MODEL_TEST name reference executable args model)
  ENDMACRO()

  MACRO (ADD_EMBREE_SUBDIV_MODELS_TEST name reference executable)
  ENDMACRO()

  MACRO (ADD_EMBREE_MODELS_TEST name)
  ENDMACRO()

ENDIF()

MACRO (ADD_EMBREE_TEST name)
  ADD_EMBREE_MODEL_TEST(${name} ${name} ${name} "" "default")
ENDMACRO()
