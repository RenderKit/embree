## ======================================================================== ##
## Copyright 2009-2013 Intel Corporation                                    ##
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

SET(RTCORE_EXIT_ON_ERROR ON CACHE BOOL "Exits the application if an error occurs.")
IF (RTCORE_EXIT_ON_ERROR)
 ADD_DEFINITIONS(-D__EXIT_ON_ERROR__)
ENDIF()

SET(RTCORE_ENABLE_RAY_MASK OFF CACHE BOOL "Enables ray mask support.")
IF (RTCORE_ENABLE_RAY_MASK)
 ADD_DEFINITIONS(-D__USE_RAY_MASK__)
ENDIF()

SET(RTCORE_ENABLE_STAT_COUNTERS OFF CACHE BOOL "Enables statistic counters.")
IF (RTCORE_ENABLE_STAT_COUNTERS)
 ADD_DEFINITIONS(-D__USE_STAT_COUNTERS__)
ENDIF()

SET(RTCORE_BACKFACE_CULLING OFF CACHE BOOL "Enables backface culling.")
IF (RTCORE_BACKFACE_CULLING)
 ADD_DEFINITIONS(-D__BACKFACE_CULLING__)
ENDIF()

SET(RTCORE_FIX_RAYS OFF CACHE BOOL "Makes traversal algorithms NAN and INF safe")
IF (RTCORE_FIX_RAYS)
 ADD_DEFINITIONS(-D__FIX_RAYS__)
ENDIF()

SET(RTCORE_ENABLE_SPINLOCKS ON CACHE BOOL "Use spinning locks.")
IF (RTCORE_ENABLE_SPINLOCKS)
 ADD_DEFINITIONS(-D__SPINLOCKS__)
ENDIF()

SET(RTCORE_ENABLE_TASKLOGGER OFF CACHE BOOL "Allows creating scheduling diagram of tasks.")
IF (RTCORE_ENABLE_TASKLOGGER)
 ADD_DEFINITIONS(-D__LOG_TASKS__)
ENDIF()

##############################################################
# ISA configuration
##############################################################

SET(TARGET_SSE41 ON CACHE BOOL "Generate code path for SSE41")
SET(TARGET_AVX   ON CACHE BOOL "Generate code path for AVX")
SET(TARGET_AVX2  ON CACHE BOOL "Generate code path for AVX2")
SET(TARGET_XEON_PHI OFF CACHE BOOL "Generate code path for Xeon Phi")

SET(ISPC_TARGETS "sse2")

IF (TARGET_SSE41 STREQUAL "ON")
  SET(ISPC_TARGETS ${ISPC_TARGETS} "sse4")
ENDIF()

IF (TARGET_AVX STREQUAL "ON")
  SET(ISPC_TARGETS ${ISPC_TARGETS} "avx")
ENDIF()

IF (TARGET_AVX2 STREQUAL "ON")
  SET(ISPC_TARGETS ${ISPC_TARGETS} "avx2")
ENDIF()

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

IF (TARGET_XEON_PHI)
  SET (__XEON__ 0)
  ADD_SUBDIRECTORY(tests tests_xeonphi)
ENDIF ()

##############################################################
# Install Headers
##############################################################

INSTALL(DIRECTORY embree2 DESTINATION include FILES_MATCHING PATTERN "*.h")
INSTALL(DIRECTORY embree2 DESTINATION include FILES_MATCHING PATTERN "*.isph")
