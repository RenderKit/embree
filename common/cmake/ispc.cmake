## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# ##################################################################
# add macro INCLUDE_DIRECTORIES_ISPC() that allows to specify search
# paths for ISPC sources
# ##################################################################
SET(ISPC_INCLUDE_DIR "")
MACRO (INCLUDE_DIRECTORIES_ISPC)
  SET(ISPC_INCLUDE_DIR ${ISPC_INCLUDE_DIR} ${ARGN})
ENDMACRO ()

IF (EMBREE_ISPC_SUPPORT)

# ISPC versions to look for, in descending order (newest first)
SET(ISPC_VERSION_WORKING "1.9.1" "1.9.0" "1.8.3" "1.8.2")
LIST(GET ISPC_VERSION_WORKING -1 ISPC_VERSION_REQUIRED)

IF (NOT EMBREE_ISPC_EXECUTABLE)
  # try sibling folder as hint for path of ISPC
  IF (APPLE)
    SET(ISPC_DIR_SUFFIX "osx")
  ELSEIF(WIN32)
    SET(ISPC_DIR_SUFFIX "windows")
    IF (MSVC14)
      LIST(APPEND ISPC_DIR_SUFFIX "windows-vs2015")
    ELSE()
      LIST(APPEND ISPC_DIR_SUFFIX "windows-vs2013")
    ENDIF()
  ELSE()
    SET(ISPC_DIR_SUFFIX "linux")
  ENDIF()
  FOREACH(ver ${ISPC_VERSION_WORKING})
    FOREACH(suffix ${ISPC_DIR_SUFFIX})
      LIST(APPEND ISPC_DIR_HINT "${PROJECT_SOURCE_DIR}/../ispc-v${ver}-${suffix}")
    ENDFOREACH()
  ENDFOREACH()

  FIND_PROGRAM(EMBREE_ISPC_EXECUTABLE ispc PATHS ${ISPC_DIR_HINT} DOC "Path to the ISPC executable.")
  IF (NOT EMBREE_ISPC_EXECUTABLE)
    MESSAGE(FATAL_ERROR "Intel SPMD Compiler (ISPC) not found. Disable EMBREE_ISPC_SUPPORT or install ISPC.")
  ELSE()
    MESSAGE(STATUS "Found Intel SPMD Compiler (ISPC): ${EMBREE_ISPC_EXECUTABLE}")
  ENDIF()
ENDIF()

# check ISPC version
EXECUTE_PROCESS(COMMAND ${EMBREE_ISPC_EXECUTABLE} --version
                OUTPUT_VARIABLE ISPC_OUTPUT
                RESULT_VARIABLE ISPC_RESULT)

IF (NOT ${ISPC_RESULT} STREQUAL "0")
  MESSAGE(FATAL_ERROR "Error executing ISPC executable '${EMBREE_ISPC_EXECUTABLE}': ${ISPC_RESULT}")
ENDIF()

STRING(REGEX MATCH "([0-9]+[.][0-9]+[.][0-9]+)" DUMMY "${ISPC_OUTPUT}")
SET(ISPC_VERSION ${CMAKE_MATCH_1})

IF (ISPC_VERSION VERSION_LESS ISPC_VERSION_REQUIRED)
  MESSAGE(FATAL_ERROR "ISPC ${ISPC_VERSION} is too old. You need at least ISPC ${ISPC_VERSION_REQUIRED}.")
ENDIF()

GET_FILENAME_COMPONENT(ISPC_DIR ${EMBREE_ISPC_EXECUTABLE} PATH)

SET(EMBREE_ISPC_ADDRESSING 32 CACHE STRING "32vs64 bit addressing in ispc")
SET_PROPERTY(CACHE EMBREE_ISPC_ADDRESSING PROPERTY STRINGS 32 64)
MARK_AS_ADVANCED(EMBREE_ISPC_ADDRESSING)

MACRO (ISPC_COMPILE)
  SET(ISPC_ADDITIONAL_ARGS "")

  SET(ISPC_TARGET_EXT ${CMAKE_CXX_OUTPUT_EXTENSION})
  STRING(REPLACE ";" "," ISPC_TARGET_ARGS "${ISPC_TARGETS}")

  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64|aarch64")
      SET(ISPC_ARCHITECTURE "aarch64")
    ELSE()
      SET(ISPC_ARCHITECTURE "x86-64")
    ENDIF()
  ELSE()
    SET(ISPC_ARCHITECTURE "x86")
  ENDIF()

  SET(ISPC_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}")

  IF(ISPC_INCLUDE_DIR)
    STRING(REPLACE ";" ";-I;" ISPC_INCLUDE_DIR_PARMS "${ISPC_INCLUDE_DIR}")
    SET(ISPC_INCLUDE_DIR_PARMS "-I" ${ISPC_INCLUDE_DIR_PARMS})
  ENDIF()

  IF (WIN32 OR "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    SET(ISPC_OPT_FLAGS -O3)
  ELSE()
    SET(ISPC_OPT_FLAGS -O2)
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
    SET(input "${CMAKE_CURRENT_SOURCE_DIR}/${src}")

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
    LIST(LENGTH ISPC_TARGETS NUM_TARGETS)
    IF (NUM_TARGETS GREATER 1)
      FOREACH(target ${ISPC_TARGETS})
        IF (${target} STREQUAL "avx512skx-i32x16")
          SET(target "avx512skx")
        ENDIF()
        SET(results ${results} "${outdir}/${fname}.dev_${target}${ISPC_TARGET_EXT}")
      ENDFOREACH()
    ENDIF()

    ADD_CUSTOM_COMMAND(
      OUTPUT ${results} "${ISPC_TARGET_DIR}/${fname}_ispc.h"
      COMMAND ${CMAKE_COMMAND} -E make_directory ${outdir}
      COMMAND ${EMBREE_ISPC_EXECUTABLE}
      -I "${CMAKE_CURRENT_SOURCE_DIR}"
      ${ISPC_INCLUDE_DIR_PARMS}
      ${ISPC_DEFINITIONS}
      --arch=${ISPC_ARCHITECTURE}
      --addressing=${EMBREE_ISPC_ADDRESSING}
      ${ISPC_OPT_FLAGS}
      --target=${ISPC_TARGET_ARGS}
      --woff
      --opt=fast-math
      ${ISPC_ADDITIONAL_ARGS}
      -h "${ISPC_TARGET_DIR}/${fname}_ispc.h"
      -MMM  ${outdir}/${fname}.dev.idep
      -o ${outdir}/${fname}.dev${ISPC_TARGET_EXT}
      ${input}
      DEPENDS ${input} ${deps}
      COMMENT "Building ISPC object ${outdir}/${fname}.dev${ISPC_TARGET_EXT}"
    )

    SET(ISPC_OBJECTS ${ISPC_OBJECTS} ${results})
  ENDFOREACH()
ENDMACRO()

MACRO (ADD_EMBREE_ISPC_EXECUTABLE name)
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
ENDMACRO()

ELSE (EMBREE_ISPC_SUPPORT)

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

ENDIF (EMBREE_ISPC_SUPPORT)
