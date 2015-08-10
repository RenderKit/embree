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

# ##################################################################
# add macro INCLUDE_DIRECTORIES_ISPC() that allows to specify search
# paths for ISPC sources
# ##################################################################
SET(ISPC_INCLUDE_DIR "")
MACRO (INCLUDE_DIRECTORIES_ISPC)
  SET(ISPC_INCLUDE_DIR ${ISPC_INCLUDE_DIR} ${ARGN})
ENDMACRO ()

IF (ENABLE_ISPC_SUPPORT)

SET(ISPC_VERSION_REQUIRED "1.8.2")

IF (NOT ISPC_EXECUTABLE)
  # try sibling folder as hint for path of ISPC
  IF (APPLE)
    SET(ISPC_DIR_SUFFIX "osx")
  ELSEIF(WIN32)
    SET(ISPC_DIR_SUFFIX "windows")
  ELSE()
    SET(ISPC_DIR_SUFFIX "linux")
  ENDIF()
  SET(ISPC_DIR_HINT ${PROJECT_SOURCE_DIR}/../ispc-v${ISPC_VERSION_REQUIRED}-${ISPC_DIR_SUFFIX})

  FIND_PROGRAM(ISPC_EXECUTABLE ispc PATHS ${ISPC_DIR_HINT} DOC "Path to the ISPC executable.")
  IF (NOT ISPC_EXECUTABLE)
    MESSAGE(FATAL_ERROR "Intel SPMD Compiler (ISPC) not found. Disable ENABLE_ISPC_SUPPORT or install ISPC.")
  ELSE()
    MESSAGE(STATUS "Found Intel SPMD Compiler (ISPC): ${ISPC_EXECUTABLE}")
  ENDIF()
ENDIF()

IF(NOT ISPC_VERSION)
  EXECUTE_PROCESS(COMMAND ${ISPC_EXECUTABLE} --version OUTPUT_VARIABLE ISPC_OUTPUT)
  STRING(REGEX MATCH " ([0-9]+[.][0-9]+[.][0-9]+)(dev|knl)? " DUMMY "${ISPC_OUTPUT}")
  SET(ISPC_VERSION ${CMAKE_MATCH_1})

  IF (ISPC_VERSION VERSION_LESS ISPC_VERSION_REQUIRED)
    MESSAGE(FATAL_ERROR "Need at least version ${ISPC_VERSION_REQUIRED} of Intel SPMD Compiler (ISPC).")
  ENDIF()

  SET(ISPC_VERSION ${ISPC_VERSION} CACHE STRING "ISPC Version")
  MARK_AS_ADVANCED(ISPC_VERSION)
  MARK_AS_ADVANCED(ISPC_EXECUTABLE)
ENDIF()

GET_FILENAME_COMPONENT(ISPC_DIR ${ISPC_EXECUTABLE} PATH)

SET(EMBREE_ISPC_ADDRESSING 32 CACHE INT "32vs64 bit addressing in ispc")
MARK_AS_ADVANCED(EMBREE_ISPC_ADDRESSING)

MACRO (ISPC_COMPILE)
  SET(ISPC_ADDITIONAL_ARGS "")

  IF (NOT __XEON__)
    SET(ISPC_TARGET_EXT .cpp)
    SET(ISPC_TARGET_ARGS generic-16)
    SET(ISPC_ADDITIONAL_ARGS ${ISPC_ADDITIONAL_ARGS} --opt=force-aligned-memory --emit-c++ --c++-include-file=${ISPC_DIR}/examples/intrinsics/knc.h -D__XEON_PHI__)
  ELSE()
    SET(ISPC_TARGET_EXT ${CMAKE_CXX_OUTPUT_EXTENSION})
    STRING(REPLACE ";" "," ISPC_TARGET_ARGS "${ISPC_TARGETS}")
  ENDIF()

  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ISPC_ARCHITECTURE "x86-64")
  ELSE()
    SET(ISPC_ARCHITECTURE "x86")
  ENDIF()

  SET(ISPC_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ISPC_TARGET_DIR})

  IF(ISPC_INCLUDE_DIR)
    STRING(REPLACE ";" ";-I;" ISPC_INCLUDE_DIR_PARMS "${ISPC_INCLUDE_DIR}")
    SET(ISPC_INCLUDE_DIR_PARMS "-I" ${ISPC_INCLUDE_DIR_PARMS})
  ENDIF()

  IF (WIN32 OR "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    SET(ISPC_OPT_FLAGS -O3)
  ELSE()
    SET(ISPC_OPT_FLAGS -O2 -g)
  ENDIF()

  IF (WIN32)
    SET(ISPC_ADDITIONAL_ARGS ${ISPC_ADDITIONAL_ARGS} --dllexport)
  ELSE()
    SET(ISPC_ADDITIONAL_ARGS ${ISPC_ADDITIONAL_ARGS} --pic)
  ENDIF()

  SET(ISPC_OBJECTS "")

  FOREACH(src ${ARGN})
    GET_FILENAME_COMPONENT(fname ${src} NAME_WE)
    GET_FILENAME_COMPONENT(dir ${src} PATH)

    SET(outdir "${ISPC_TARGET_DIR}/${dir}")
    SET(input ${CMAKE_CURRENT_SOURCE_DIR}/${src})

    SET(deps "")
    IF (EXISTS ${outdir}/${fname}.dev.idep)
      FILE(READ ${outdir}/${fname}.dev.idep contents)
      STRING(REPLACE " " ";"     contents "${contents}")
      STRING(REPLACE ";" "\\\\;" contents "${contents}")
      STRING(REPLACE "\n" ";"    contents "${contents}")
      FOREACH(dep ${contents})
        IF (EXISTS ${dep})
          SET(deps ${deps} ${dep})
        ENDIF (EXISTS ${dep})
      ENDFOREACH(dep ${contents})
    ENDIF ()

    SET(results "${outdir}/${fname}.dev${ISPC_TARGET_EXT}")

    # if we have multiple targets add additional object files
    IF (__XEON__)
      LIST(LENGTH ISPC_TARGETS NUM_TARGETS)
      IF (NUM_TARGETS GREATER 1)
        FOREACH(target ${ISPC_TARGETS})
          SET(results ${results} "${outdir}/${fname}.dev_${target}${ISPC_TARGET_EXT}")
        ENDFOREACH()
      ENDIF()
    ENDIF()

    ADD_CUSTOM_COMMAND(
      OUTPUT ${results} ${ISPC_TARGET_DIR}/${fname}_ispc.h
      COMMAND ${CMAKE_COMMAND} -E make_directory ${outdir}
      COMMAND ${ISPC_EXECUTABLE}
      -I ${CMAKE_CURRENT_SOURCE_DIR}
      ${ISPC_INCLUDE_DIR_PARMS}
      --arch=${ISPC_ARCHITECTURE}
      --addressing=${EMBREE_ISPC_ADDRESSING}
      ${ISPC_OPT_FLAGS}
      --target=${ISPC_TARGET_ARGS}
      --woff
      --opt=fast-math
      ${ISPC_ADDITIONAL_ARGS}
      -h ${ISPC_TARGET_DIR}/${fname}_ispc.h
      -MMM  ${outdir}/${fname}.dev.idep
      -o ${outdir}/${fname}.dev${ISPC_TARGET_EXT}
      ${input}
      DEPENDS ${input} ${deps}
      COMMENT "Building ISPC object ${outdir}/${fname}.dev${ISPC_TARGET_EXT}"
    )

    SET(ISPC_OBJECTS ${ISPC_OBJECTS} ${results})
  ENDFOREACH()
ENDMACRO()

MACRO (ADD_ISPC_EXECUTABLE name)
   SET(ISPC_SOURCES "")
   SET(OTHER_SOURCES "")
   FOREACH(src ${ARGN})
    GET_FILENAME_COMPONENT(ext ${src} EXT)
    IF (ext STREQUAL ".ispc")
      SET(ISPC_SOURCES ${ISPC_SOURCES} ${src})
    ELSE ()
      SET(OTHER_SOURCES ${OTHER_SOURCES} ${src})
    ENDIF ()
  ENDFOREACH()
  ISPC_COMPILE(${ISPC_SOURCES})
  ADD_EXECUTABLE(${name} ${ISPC_OBJECTS} ${OTHER_SOURCES})

  IF (NOT __XEON__)
    FOREACH(src ${ISPC_OBJECTS})
      SET_SOURCE_FILES_PROPERTIES( ${src} PROPERTIES COMPILE_FLAGS -std=gnu++98 )
    ENDFOREACH()
  ENDIF()
ENDMACRO()

MACRO (ADD_ISPC_LIBRARY name type)
   SET(ISPC_SOURCES "")
   SET(OTHER_SOURCES "")
   FOREACH(src ${ARGN})
    GET_FILENAME_COMPONENT(ext ${src} EXT)
    IF (ext STREQUAL ".ispc")
      SET(ISPC_SOURCES ${ISPC_SOURCES} ${src})
    ELSE ()
      SET(OTHER_SOURCES ${OTHER_SOURCES} ${src})
    ENDIF ()
  ENDFOREACH()
  ISPC_COMPILE(${ISPC_SOURCES})
  ADD_LIBRARY(${name} ${type} ${ISPC_OBJECTS} ${OTHER_SOURCES})

  IF (NOT __XEON__)
    FOREACH(src ${ISPC_OBJECTS})
      SET_SOURCE_FILES_PROPERTIES( ${src} PROPERTIES COMPILE_FLAGS -std=gnu++98 )
    ENDFOREACH()
  ENDIF()
ENDMACRO()

ELSE (ENABLE_ISPC_SUPPORT)

MACRO (ADD_ISPC_LIBRARY name type)
   SET(ISPC_SOURCES "")
   SET(OTHER_SOURCES "")
   FOREACH(src ${ARGN})
    GET_FILENAME_COMPONENT(ext ${src} EXT)
    IF (ext STREQUAL ".ispc")
      SET(ISPC_SOURCES ${ISPC_SOURCES} ${src})
    ELSE ()
      SET(OTHER_SOURCES ${OTHER_SOURCES} ${src})
    ENDIF ()
  ENDFOREACH()
  ADD_LIBRARY(${name} ${type} ${OTHER_SOURCES})
ENDMACRO()

ENDIF (ENABLE_ISPC_SUPPORT)
