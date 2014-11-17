## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
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

SET (ISPC_INCLUDE_DIR "")
MACRO (INCLUDE_DIRECTORIES_ISPC)
  SET (ISPC_INCLUDE_DIR ${ISPC_INCLUDE_DIR} ${ARGN})
ENDMACRO ()

IF (NOT ISPC_EXECUTABLE)
  FIND_PROGRAM(ISPC_EXECUTABLE ispc)
  MARK_AS_ADVANCED(ISPC_EXECUTABLE)
  IF (NOT ISPC_EXECUTABLE)
    MESSAGE(FATAL_ERROR "Intel SPMD Compiler (ISPC) not found.")
  ELSE()
    MESSAGE(STATUS "Found Intel SPMD Compiler (ISPC): ${ISPC_EXECUTABLE}")
  ENDIF()
ENDIF()
GET_FILENAME_COMPONENT(ISPC_DIR ${ISPC_EXECUTABLE} DIRECTORY)

SET(EMBREE_ISPC_ADDRESSING 32 CACHE INT "32vs64 bit addressing in ispc")
MARK_AS_ADVANCED(EMBREE_ISPC_ADDRESSING)


MACRO (ispc_compile targets)
  IF (__XEON__)
    SET (ISPC_TARGET_EXT ${CMAKE_CXX_OUTPUT_EXTENSION})
  ELSE()
    SET (ISPC_TARGET_EXT .cpp)
    SET (ISPC_TARGET_ALIGNED_MEMORY --opt=force-aligned-memory)
  ENDIF()

  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ISPC_ARCHITECTURE "x86-64")
  ELSE()
    SET(ISPC_ARCHITECTURE "x86")
  ENDIF()

  SET(ISPC_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR})
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${ISPC_TARGET_DIR})

  IF(ISPC_INCLUDE_DIR)
    STRING(REGEX REPLACE ";" ";-I;" ISPC_INCLUDE_DIR_PARMS "${ISPC_INCLUDE_DIR}")
    SET(ISPC_INCLUDE_DIR_PARMS "-I" ${ISPC_INCLUDE_DIR_PARMS})
  ENDIF()

  IF (__XEON__)
    STRING(REGEX REPLACE "," ";" target_list "${targets}")
    SET(ISPC_TARGETS ${targets})
  ELSE()
    SET(ISPC_TARGETS generic-16 --emit-c++ -D__XEON_PHI__ --c++-include-file=${ISPC_DIR}/examples/intrinsics/knc.h)
  ENDIF()

  SET(ISPC_OBJECTS "")

  FOREACH(src ${ARGN})
    GET_FILENAME_COMPONENT(fname ${src} NAME_WE)
    GET_FILENAME_COMPONENT(dir ${src} DIRECTORY)

    IF("${dir}" STREQUAL "")
      SET(outdir ${ISPC_TARGET_DIR})
    ELSE("${dir}" STREQUAL "")
      SET(outdir ${ISPC_TARGET_DIR}/${dir})
    ENDIF("${dir}" STREQUAL "")
    SET(outdirh ${ISPC_TARGET_DIR})

    SET(deps "")
    IF (EXISTS ${outdir}/${fname}.dev.idep)
      FILE(READ ${outdir}/${fname}.dev.idep contents)
      STRING(REGEX REPLACE " " ";"     contents "${contents}")
      STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
      STRING(REGEX REPLACE "\n" ";"    contents "${contents}")
      FOREACH(dep ${contents})
        IF (EXISTS ${dep})
          SET(deps ${deps} ${dep})
        ENDIF (EXISTS ${dep})
      ENDFOREACH(dep ${contents})
    ENDIF ()

    SET(results "${outdir}/${fname}.dev${ISPC_TARGET_EXT}")

    # if we have multiple targets add additional object files
    IF (__XEON__)
      IF (${targets} MATCHES ".*,.*")
        FOREACH(target ${target_list})
          SET(results ${results} "${outdir}/${fname}.dev_${target}${ISPC_TARGET_EXT}")
        ENDFOREACH()
      ENDIF()
    ENDIF()

    IF (WIN32)
      SET(ISPC_ARGS "")
    ELSE (WIN32)
      SET(ISPC_ARGS --pic)
    ENDIF (WIN32)

    ADD_CUSTOM_COMMAND(
      OUTPUT ${results} ${outdirh}/${fname}_ispc.h
      COMMAND ${CMAKE_COMMAND} -E make_directory ${outdir}
      COMMAND ${ISPC_EXECUTABLE}
      -I ${CMAKE_CURRENT_SOURCE_DIR}
      ${ISPC_INCLUDE_DIR_PARMS}
      --arch=${ISPC_ARCHITECTURE}
      --addressing=${EMBREE_ISPC_ADDRESSING}
      ${ISPC_ARGS}
      -O3
      --target=${ISPC_TARGETS}
      --woff
#      --wno-perf
      --opt=fast-math
      ${ISPC_TARGET_ALIGNED_MEMORY}
#      --opt=force-aligned-memory
      -h ${outdirh}/${fname}_ispc.h
      -MMM  ${outdir}/${fname}.dev.idep
      -o ${outdir}/${fname}.dev${ISPC_TARGET_EXT}
      ${CMAKE_CURRENT_SOURCE_DIR}/${src}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${src} ${deps}
      COMMENT "Building with Intel SPMD Compiler (ISPC): ${CMAKE_CURRENT_SOURCE_DIR}/${src}"
    )

    SET(ISPC_OBJECTS ${ISPC_OBJECTS} ${results})

  ENDFOREACH()

ENDMACRO()

MACRO (add_ispc_executable name)
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
  ISPC_COMPILE (${ISPC_TARGETS} ${ISPC_SOURCES})
  ADD_EXECUTABLE(${name} ${ISPC_OBJECTS} ${OTHER_SOURCES})

  IF (NOT __XEON__)
    FOREACH(src ${ISPC_OBJECTS})
      SET_SOURCE_FILES_PROPERTIES( ${src} PROPERTIES COMPILE_FLAGS -std=gnu++98 )
    ENDFOREACH()
  ENDIF()

#  SET_TARGET_PROPERTIES(${name} PROPERTIES LINKER_LANGUAGE C)
ENDMACRO()

MACRO (add_ispc_library name type)
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
  ISPC_COMPILE (${ISPC_TARGETS} ${ISPC_SOURCES})
  ADD_LIBRARY(${name} ${type} ${ISPC_OBJECTS} ${OTHER_SOURCES})

  IF (NOT __XEON__)
    FOREACH(src ${ISPC_OBJECTS})
      SET_SOURCE_FILES_PROPERTIES( ${src} PROPERTIES COMPILE_FLAGS -std=gnu++98 )
    ENDFOREACH()
  ENDIF()
#  SET_TARGET_PROPERTIES(${name} PROPERTIES LINKER_LANGUAGE C)
ENDMACRO()
