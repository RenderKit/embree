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

PROJECT(embree)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY)
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
MARK_AS_ADVANCED(CLEAR CMAKE_VERBOSE_MAKEFILE)

MARK_AS_ADVANCED(CMAKE_OSX_ARCHITECTURES)
MARK_AS_ADVANCED(CMAKE_OSX_DEPLOYMENT_TARGET)
MARK_AS_ADVANCED(CMAKE_OSX_SYSROOT)
MARK_AS_ADVANCED(GLUT_cocoa_LIBRARY)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/common/cmake ${CMAKE_MODULE_PATH})

##############################################################
# Embree configuration
##############################################################

SET(RTCORE_RAY_MASK OFF CACHE BOOL "Enables ray mask support.")
IF (RTCORE_RAY_MASK)
 ADD_DEFINITIONS(-D__USE_RAY_MASK__)
ENDIF()

SET(RTCORE_STAT_COUNTERS OFF CACHE BOOL "Enables statistic counters.")
IF (RTCORE_STAT_COUNTERS)
 ADD_DEFINITIONS(-D__USE_STAT_COUNTERS__)
ENDIF()

SET(RTCORE_BACKFACE_CULLING OFF CACHE BOOL "Enables backface culling.")
IF (RTCORE_BACKFACE_CULLING)
 ADD_DEFINITIONS(-D__BACKFACE_CULLING__)
ENDIF()

IF (XEON_PHI_ISA)
#  UNSET(RTCORE_INTERSECTION_FILTER CACHE)
  SET(RTCORE_INTERSECTION_FILTER OFF CACHE BOOL "Enables intersection filter callback.")
ELSE ()
  SET(RTCORE_INTERSECTION_FILTER ON CACHE BOOL "Enables intersection filter callback.")
ENDIF()

IF (RTCORE_INTERSECTION_FILTER)
 ADD_DEFINITIONS(-D__INTERSECTION_FILTER__)
ENDIF()

SET(RTCORE_BUFFER_STRIDE ON CACHE BOOL "Enables buffer strides.")
IF (RTCORE_BUFFER_STRIDE)
 ADD_DEFINITIONS(-D__BUFFER_STRIDE__)
ENDIF()

SET(RTCORE_SPINLOCKS OFF CACHE BOOL "Use spinning locks.")

IF (RTCORE_SPINLOCKS)
 ADD_DEFINITIONS(-D__SPINLOCKS__)
ENDIF()

SET(RTCORE_TASKLOGGER OFF CACHE BOOL "Allows creating scheduling diagram of tasks.")
IF (RTCORE_TASKLOGGER)
 ADD_DEFINITIONS(-D__LOG_TASKS__)
ENDIF()

SET(RTCORE_EXPORT_ALL_SYMBOLS OFF CACHE BOOL "Lets Embree shared library export all symbols.")
IF (RTCORE_EXPORT_ALL_SYMBOLS)
 ADD_DEFINITIONS(-D__EXPORT_ALL_SYMBOLS__)
ENDIF()

SET(RTCORE_ENABLE_RAYSTREAM_LOGGER OFF CACHE BOOL "Enable ray stream logger")
IF (RTCORE_ENABLE_RAYSTREAM_LOGGER)
 ADD_DEFINITIONS(-D__ENABLE_RAYSTREAM_LOGGER__)
ENDIF()

##############################################################
# ISA configuration
##############################################################

SET(XEON_ISA "AVX2" CACHE STRING "Select highest ISA to support (SSE2, SSE3, SSSE3, SSE4.1, SSE4.2, AVX, AVX-I, AVX2")
SET(XEON_PHI_ISA OFF CACHE BOOL "Generate code path for Xeon Phi")

IF (XEON_ISA STREQUAL "SSE2")
  SET(ISA  1)
ENDIF ()

IF (XEON_ISA STREQUAL "SSE3")
  SET(ISA  2)
ENDIF ()

IF (XEON_ISA STREQUAL "SSSE3")
  SET(ISA  3)
ENDIF ()

IF (XEON_ISA STREQUAL "SSE4.1")
  SET(ISA  4)
ENDIF ()

IF (XEON_ISA STREQUAL "SSE4.2")
  SET(ISA  5)
ENDIF ()

IF (XEON_ISA STREQUAL "AVX")
  SET(ISA  6)
ENDIF ()

IF (XEON_ISA STREQUAL "AVX-I")
  SET(ISA  7)
ENDIF ()

IF (XEON_ISA STREQUAL "AVX2")
  SET(ISA  8)
ENDIF ()

IF (XEON_ISA STREQUAL "AVX512")
  SET(ISA  9)
ENDIF ()

SET(TARGET_SSE2  OFF)
SET(TARGET_SSE3  OFF)
SET(TARGET_SSSE3  OFF)
SET(TARGET_SSE41  OFF)
SET(TARGET_SSE42  OFF)
SET(TARGET_AVX  OFF)
SET(TARGET_AVXI  OFF)
SET(TARGET_AVX2  OFF)
SET(TARGET_AVX512 OFF)

IF (ISA GREATER 0)
  SET(TARGET_SSE2  ON)
  SET(ISPC_TARGETS "sse2")
ENDIF ()

IF (ISA GREATER 1)
  SET(TARGET_SSE3  ON)
ENDIF ()

IF (ISA GREATER 2)
  SET(TARGET_SSSE3  ON)
ENDIF ()

IF (ISA GREATER 3)
  SET(TARGET_SSE41  ON)
ENDIF ()

IF (ISA GREATER 4)
  SET(TARGET_SSE42  ON)
  SET(ISPC_TARGETS ${ISPC_TARGETS} "sse4")
ENDIF ()

IF (ISA GREATER 5)
  SET(TARGET_AVX  ON)
  SET(ISPC_TARGETS ${ISPC_TARGETS} "avx")
ENDIF ()

IF (ISA GREATER 6)
  SET(TARGET_AVXI  ON)
ENDIF ()

IF (ISA GREATER 7)
  SET(TARGET_AVX2  ON)
  SET(ISPC_TARGETS ${ISPC_TARGETS} "avx2")
ENDIF ()

IF (ISA GREATER 8)
  SET(TARGET_AVX512  ON)
ENDIF ()

SET(ISPC_TARGETS "${ISPC_TARGETS}")
STRING(REGEX REPLACE ";" "," ISPC_TARGETS "${ISPC_TARGETS}")

##############################################################
# Compiler
##############################################################

SET(COMPILER "GCC" CACHE STRING "Set to GCC, CLANG, or ICC")

IF (COMPILER STREQUAL "GCC") 
  INCLUDE (gcc)
ELSEIF (COMPILER STREQUAL "CLANG") 
  INCLUDE (clang)
ELSEIF (COMPILER STREQUAL "ICC") 
  INCLUDE (icc)
ELSE ()
  MESSAGE(FATAL_ERROR "Unknown compiler specified: " ${COMPILER})
ENDIF ()

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type: Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

SET(CMAKE_VERBOSE_MAKEFILE false CACHE BOOL "Enables verbose mode.")

INCLUDE (ispc)

##############################################################
# Search paths
##############################################################
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/common)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES_ISPC(${PROJECT_SOURCE_DIR})
LINK_DIRECTORIES(${EXECUTABLE_OUTPUT_PATH})

##############################################################
# Output paths
##############################################################

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

##############################################################
# Directories to compile
##############################################################

ADD_SUBDIRECTORY(common)
ADD_SUBDIRECTORY(kernels)
ADD_SUBDIRECTORY(tutorials)

SET (__XEON__ 1)
ADD_SUBDIRECTORY(tests)

IF (XEON_PHI_ISA)
  SET (__XEON__ 0)
  ADD_SUBDIRECTORY(tests tests_xeonphi)
ENDIF ()

##############################################################
# Install Headers
##############################################################

INSTALL(DIRECTORY include/embree2 DESTINATION include FILES_MATCHING PATTERN "*.h")
INSTALL(DIRECTORY include/embree2 DESTINATION include FILES_MATCHING PATTERN "*.isph")

# ##################################################################
# Create Documentation. Call via 'make doc'
# ##################################################################

ADD_CUSTOM_TARGET(doc
  make -C ${CMAKE_CURRENT_SOURCE_DIR}/doc
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc 
  COMMENT "Generating documentation" VERBATIM)

