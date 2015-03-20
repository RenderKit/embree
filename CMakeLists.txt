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

PROJECT(embree)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(EMBREE_VERSION_MAJOR 2)
SET(EMBREE_VERSION_MINOR 5)
SET(EMBREE_VERSION_PATCH 0)
SET(EMBREE_VERSION_NOTE " (devel)")

CONFIGURE_FILE(
  "${PROJECT_SOURCE_DIR}/kernels/version.h.in"
  "${PROJECT_BINARY_DIR}/version.h"
)
INCLUDE_DIRECTORIES("${PROJECT_BINARY_DIR}")

IF(COMMAND cmake_policy)
  if (POLICY CMP0003)
    cmake_policy(SET CMP0003 NEW)
  endif()
  if (POLICY CMP0042)
    cmake_policy(SET CMP0042 OLD)
  endif()
ENDIF(COMMAND cmake_policy)

MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY)
MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

MARK_AS_ADVANCED(CMAKE_OSX_ARCHITECTURES)
MARK_AS_ADVANCED(CMAKE_OSX_DEPLOYMENT_TARGET)
MARK_AS_ADVANCED(CMAKE_OSX_SYSROOT)
MARK_AS_ADVANCED(GLUT_cocoa_LIBRARY)

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/common/cmake ${CMAKE_MODULE_PATH})

##############################################################
# Embree configuration
##############################################################

OPTION(ENABLE_STATIC_LIB "Build Embree as a static library." OFF)
IF (ENABLE_STATIC_LIB)
  SET(EMBREE_LIB_TYPE STATIC)
  ADD_DEFINITIONS(-DENABLE_STATIC_LIB)
ELSE()
  SET(EMBREE_LIB_TYPE SHARED)
ENDIF()

OPTION(ENABLE_ISPC_SUPPORT "Build Embree with support for ISPC applications." ON)
OPTION(ENABLE_TUTORIALS    "Enable to build Embree tutorials" ON)

# only support Xeon Phi on Linux
IF (NOT WIN32 AND NOT APPLE)
  OPTION(ENABLE_XEON_PHI_SUPPORT "Generates code path for Xeon Phi.")
ELSE()
  SET(ENABLE_XEON_PHI_SUPPORT OFF CACHE INTERNAL "Xeon Phi only supported on Linux.")
ENDIF()

IF (ENABLE_XEON_PHI_SUPPORT)
  SET(ENABLE_ISPC_SUPPORT ON CACHE BOOL "Build Embree with support for ISPC applications." FORCE)
ENDIF ()

OPTION(RTCORE_RAY_MASK "Enables ray mask support.")
OPTION(RTCORE_STAT_COUNTERS "Enables statistic counters.")
OPTION(RTCORE_BACKFACE_CULLING "Enables backface culling.")
OPTION(RTCORE_INTERSECTION_FILTER "Enables intersection filter callback." ON)
OPTION(RTCORE_BUFFER_STRIDE "Enables buffer strides." ON)
OPTION(RTCORE_EXPORT_ALL_SYMBOLS "Lets Embree library export all symbols.")
OPTION(RTCORE_ENABLE_RAYSTREAM_LOGGER "Enables ray stream logger.")
OPTION(RTCORE_RETURN_SUBDIV_NORMAL "Returns smooth subdivision surface normal in ray structure instead of triangle normal.")

SET(RTCORE_TASKING_SYSTEM "TBB" CACHE STRING "Selects tasking system")
SET_PROPERTY(CACHE RTCORE_TASKING_SYSTEM PROPERTY STRINGS LOCKSTEP TBB TBB_INTERNAL)

#IF (RTCORE_TASKING_SYSTEM STREQUAL "LOCKSTEP")
#  SET(TASKING_LOCKSTEP ON)
#  ADD_DEFINITIONS(-DTASKING_LOCKSTEP)
#ENDIF()

#IF (RTCORE_TASKING_SYSTEM STREQUAL "TBB")
#  SET(TASKING_TBB ON)
#  FIND_PACKAGE(TBB REQUIRED)
#  ADD_DEFINITIONS(-DTASKING_TBB)
#  INCLUDE_DIRECTORIES(${TBB_INCLUDE_DIRS})
#ENDIF()

#IF (RTCORE_TASKING_SYSTEM STREQUAL "TBB_INTERNAL")
#  SET(TASKING_TBB_INTERNAL ON)
#  ADD_DEFINITIONS(-DTASKING_TBB_INTERNAL)
#ENDIF()

CONFIGURE_FILE(
  "${PROJECT_SOURCE_DIR}/common/sys/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
)

##############################################################
# ISA configuration
##############################################################

SET(XEON_ISA "AVX2" CACHE STRING "Selects highest ISA to support.")
SET_PROPERTY(CACHE XEON_ISA PROPERTY STRINGS SSE2 SSE3 SSSE3 SSE4.1 SSE4.2 AVX AVX-I AVX2 AVX512)

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
  LIST(APPEND ISPC_TARGETS "sse4")
ENDIF ()

IF (ISA GREATER 4)
  SET(TARGET_SSE42  ON)
ENDIF ()

IF (ISA GREATER 5)
  SET(TARGET_AVX  ON)
  LIST(APPEND ISPC_TARGETS "avx")
ENDIF ()

IF (ISA GREATER 6)
  SET(TARGET_AVXI  ON)
ENDIF ()

IF (ISA GREATER 7)
  SET(TARGET_AVX2  ON)
  LIST(APPEND ISPC_TARGETS "avx2")
ENDIF ()

IF (ISA GREATER 8)
  SET(TARGET_AVX512  ON)
ENDIF ()


SET(CONFIGURATION_TYPES "Debug;Release;RelWithDebInfo")

##############################################################
# Windows specific
##############################################################
IF (WIN32)
  IF (NOT DEFAULT_CMAKE_CONFIGURATION_TYPES_SET)
    SET(CMAKE_CONFIGURATION_TYPES "${CONFIGURATION_TYPES}" CACHE STRING "List of generated configurations." FORCE)
    SET(DEFAULT_CMAKE_CONFIGURATION_TYPES_SET ON CACHE INTERNAL "Default CMake configuration types set.")
  ENDIF()

  SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

##############################################################
# Compiler
##############################################################
  INCLUDE (msvc)
ELSE (WIN32)
  SET(COMPILER "GCC" CACHE STRING "Selects compiler to use.")
  SET_PROPERTY(CACHE COMPILER PROPERTY STRINGS GCC CLANG ICC)

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
    SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Specifies the build type." FORCE)
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CONFIGURATION_TYPES})
  ENDIF(NOT CMAKE_BUILD_TYPE)

  OPTION(CMAKE_VERBOSE_MAKEFILE "Enables verbose mode.")
  MARK_AS_ADVANCED(CLEAR CMAKE_VERBOSE_MAKEFILE)

ENDIF (WIN32)

INCLUDE (ispc)

##############################################################
# Search paths
##############################################################
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/common)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES_ISPC(${PROJECT_SOURCE_DIR})

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

IF (ENABLE_TUTORIALS)
  ADD_SUBDIRECTORY(tutorials)

  SET (__XEON__ 1)
  ADD_SUBDIRECTORY(tests)

  IF (ENABLE_XEON_PHI_SUPPORT)
    SET (__XEON__ 0)
    ADD_SUBDIRECTORY(tests tests_xeonphi)
  ENDIF ()
ENDIF()

##############################################################
# Install Headers
##############################################################

INSTALL(DIRECTORY include/embree2 DESTINATION include)


# currently CMake does not support solution folders without projects
# SOURCE_GROUP("Documentation" FILES README.md CHANGELOG.md readme.pdf)

##############################################################
# Create Binary Packages
##############################################################
SET(CPACK_PACKAGE_VERSION_MAJOR ${EMBREE_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${EMBREE_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${EMBREE_VERSION_PATCH})
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "High Performance Ray Tracing Kernels")
SET(CPACK_PACKAGE_VENDOR "Intel Corporation")
SET(CPACK_PACKAGE_CONTACT embree_support@intel.com)
SET(CPACK_GENERATOR RPM TGZ) #DEB
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/README.md")
SET(CPACK_STRIP_FILES OFF) # strip failes on Xeon Phi executables
STRING(TOLOWER ${CMAKE_SYSTEM_NAME} CPACK_SYSTEM_NAME)

# RPM specific settings
SET(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0") # Apache Software License, Version 2.0
SET(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
#SET(CPACK_RPM_CHANGELOG_FILE "") # ChangeLog of the RPM; also CHANGELOG.md is not in the required format
SET(CPACK_RPM_PACKAGE_ARCHITECTURE x86_64)
SET(CPACK_RPM_PACKAGE_URL http://embree.github.io/)
#SET(CPACK_RPM_USER_FILELIST "%doc ${PROJECT_SOURCE_DIR}/README.md")
#SET(CPACK_RPM_USER_FILELIST "%doc ${PROJECT_SOURCE_DIR}/CHANGELOG.md")
#SET(CPACK_RPM_USER_FILELIST "%doc ${PROJECT_SOURCE_DIR}/readme.pdf")

INCLUDE(CPack)
