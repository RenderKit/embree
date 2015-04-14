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

SET(EMBREE_VERSION_MAJOR 2)
SET(EMBREE_VERSION_MINOR 5)
SET(EMBREE_VERSION_PATCH 1)
SET(EMBREE_VERSION_NOTE " (devel)")
SET(EMBREE_VERSION ${EMBREE_VERSION_MAJOR}.${EMBREE_VERSION_MINOR}.${EMBREE_VERSION_PATCH})
SET(CPACK_RPM_PACKAGE_RELEASE 1)

PROJECT(embree${EMBREE_VERSION_MAJOR})

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

CONFIGURE_FILE(
  "${PROJECT_SOURCE_DIR}/kernels/version.h.in"
  "${PROJECT_BINARY_DIR}/version.h"
)
INCLUDE_DIRECTORIES("${PROJECT_BINARY_DIR}")

IF (WIN32)
  SET(DOC_INSTALL_DIR doc)
  SET(TUTORIALS_INSTALL_DIR bin)
  SET(UTILITIES_INSTALL_DIR bin)
ELSEIF (APPLE)
  SET(DOC_INSTALL_DIR ../Applications/embree-${EMBREE_VERSION}/documentation)
  SET(TUTORIALS_INSTALL_DIR ../Applications/embree-${EMBREE_VERSION}/tutorials)
  SET(UTILITIES_INSTALL_DIR ../Applications/embree-${EMBREE_VERSION}/tutorials)
ELSE()
  SET(DOC_INSTALL_DIR share/doc/embree-${EMBREE_VERSION})
  SET(TUTORIALS_INSTALL_DIR bin)
  SET(UTILITIES_INSTALL_DIR bin)
ENDIF()

IF(COMMAND cmake_policy)
  if (POLICY CMP0003)
    cmake_policy(SET CMP0003 NEW)
  endif()
  if (POLICY CMP0042)
    cmake_policy(SET CMP0042 OLD)
  endif()
ENDIF(COMMAND cmake_policy)

MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY)
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
SET_PROPERTY(CACHE RTCORE_TASKING_SYSTEM PROPERTY STRINGS TBB INTERNAL)

IF (RTCORE_TASKING_SYSTEM STREQUAL "TBB")
  SET(TASKING_TBB ON)
  FIND_PACKAGE(TBB REQUIRED)
  INCLUDE_DIRECTORIES(${TBB_INCLUDE_DIRS})
ELSE()
  SET(TASKING_INTERNAL ON)
ENDIF()

CONFIGURE_FILE(
  "${PROJECT_SOURCE_DIR}/kernels/config.h.in"
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
INSTALL(DIRECTORY include/embree2 DESTINATION include COMPONENT headers)

##############################################################
# Install Embree CMake Configuration
##############################################################
IF (WIN32)
  CONFIGURE_FILE("common/cmake/embree-config-windows.cmake" "embree-config.cmake" @ONLY)
ELSEIF (APPLE)
  CONFIGURE_FILE("common/cmake/embree-config-macosx.cmake" "embree-config.cmake" @ONLY)
ELSE()
  CONFIGURE_FILE("common/cmake/embree-config-linux.cmake" "embree-config.cmake" @ONLY)
ENDIF()

CONFIGURE_FILE("common/cmake/embree-config-version.cmake" "embree-config-version.cmake" @ONLY)

INSTALL(FILES "${PROJECT_BINARY_DIR}/embree-config.cmake" DESTINATION "lib/cmake/embree-${EMBREE_VERSION}" COMPONENT libraries)
INSTALL(FILES "${PROJECT_BINARY_DIR}/embree-config-version.cmake" DESTINATION "lib/cmake/embree-${EMBREE_VERSION}" COMPONENT libraries)

##############################################################
# Documentation
##############################################################

FILE(GLOB DOC_FILES ${PROJECT_SOURCE_DIR}/embree-doc/docbin/*)
INSTALL(FILES ${DOC_FILES} OPTIONAL DESTINATION ${DOC_INSTALL_DIR} COMPONENT documentation)
INSTALL(FILES ${PROJECT_SOURCE_DIR}/LICENSE.txt DESTINATION ${DOC_INSTALL_DIR} COMPONENT documentation)

# currently CMake does not support solution folders without projects
# SOURCE_GROUP("Documentation" FILES README.md CHANGELOG.md LICENSE.txt readme.pdf)

##############################################################
# Uninstall
##############################################################

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/common/cmake/uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake)

##############################################################
# Create Binary Packages
##############################################################
SET(CPACK_PACKAGE_VERSION_MAJOR ${EMBREE_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${EMBREE_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${EMBREE_VERSION_PATCH})
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Embree: High Performance Ray Tracing Kernels")
SET(CPACK_PACKAGE_VENDOR "intel") # creates short name com.intel.embree2.xxx in pkgutil
SET(CPACK_PACKAGE_CONTACT embree_support@intel.com)

IF (WIN32)
  SET(CPACK_COMPONENTS_ALL libraries headers documentation tutorials utilities redist)
ELSEIF (APPLE)
    SET(CPACK_MONOLITHIC_INSTALL 1)
ELSE()
  SET(CPACK_COMPONENTS_ALL libraries headers documentation tutorials utilities libraries_xeonphi tutorials_xeonphi utilities_xeonphi)
ENDIF()

SET(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries")
SET(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "Headers")
SET(CPACK_COMPONENT_HEADERS_DESCRIPTION "C and ISPC header files needed to develop applications with Embree.")
SET(CPACK_COMPONENT_DOCUMENTATION_DISPLAY_NAME "Documentation")
SET(CPACK_COMPONENT_DOCUMENTATION_DESCRIPTION "Documents describing Embree's API, usage and tutorials.")
SET(CPACK_COMPONENT_TUTORIALS_DISPLAY_NAME "Tutorials")
SET(CPACK_COMPONENT_TUTORIALS_DESCRIPTION "A set of tutorials demonstrating how Embree can be used and extended.")
SET(CPACK_COMPONENT_UTILITIES_DISPLAY_NAME "Utilities")
SET(CPACK_COMPONENT_UTILITIES_DESCRIPTION "Tools to benchmark, test and debug Embree.")
SET(CPACK_COMPONENT_REDIST_DISPLAY_NAME "Redistributables")
SET(CPACK_COMPONENT_REDIST_DESCRIPTION "Install 3rd-party redistributables needed by Embree.")

SET(CPACK_COMPONENT_DOCUMENTATION_GROUP devel)
SET(CPACK_COMPONENT_LIBRARIES_GROUP devel)
SET(CPACK_COMPONENT_HEADERS_GROUP devel)
SET(CPACK_COMPONENT_TUTORIALS_GROUP examples)
SET(CPACK_COMPONENT_UTILITIES_GROUP examples)
SET(CPACK_COMPONENT_REDIST_GROUP examples)
SET(CPACK_COMPONENT_LIBRARIES_XEONPHI_GROUP devel_xeonphi)
SET(CPACK_COMPONENT_TUTORIALS_XEONPHI_GROUP examples_xeonphi)
SET(CPACK_COMPONENT_UTILITIES_XEONPHI_GROUP examples_xeonphi)

SET(CPACK_COMPONENT_GROUP_DEVEL_DISPLAY_NAME "Development")
SET(CPACK_COMPONENT_GROUP_DEVEL_DESCRIPTION "Files needed to develop applications based on Embree.")
SET(CPACK_COMPONENT_DEVEL_DESCRIPTION ${CPACK_COMPONENT_GROUP_DEVEL_DESCRIPTION})
SET(XEONPHI_DESCRIPTION "For The Intel Xeon Phi coprocessor." )
SET(CPACK_COMPONENT_DEVEL_XEONPHI_DESCRIPTION "${CPACK_COMPONENT_GROUP_DEVEL_DESCRIPTION}\n${XEONPHI_DESCRIPTION}")
SET(CPACK_COMPONENT_GROUP_EXAMPLES_DISPLAY_NAME "Examples")
SET(CPACK_COMPONENT_GROUP_EXAMPLES_DESCRIPTION "Example tutorials and tools for Embree.")
SET(CPACK_COMPONENT_EXAMPLES_DESCRIPTION ${CPACK_COMPONENT_GROUP_EXAMPLES_DESCRIPTION})
SET(CPACK_COMPONENT_EXAMPLES_XEONPHI_DESCRIPTION "${CPACK_COMPONENT_GROUP_EXAMPLES_DESCRIPTION}\n${XEONPHI_DESCRIPTION}")

SET(CPACK_COMPONENT_TUTORIALS_DEPENDS libraries)
SET(CPACK_COMPONENT_UTILITIES_DEPENDS libraries)
SET(CPACK_COMPONENT_LIBRARIES_XEONPHI_DEPENDS libraries)
SET(CPACK_COMPONENT_TUTORIALS_XEONPHI_DEPENDS libraries_xeonphi)
SET(CPACK_COMPONENT_UTILITIES_XEONPHI_DEPENDS libraries_xeonphi)

IF (APPLE)
  SET(CPACK_COMPONENT_LIBRARIES_REQUIRED ON)
  SET(CPACK_COMPONENT_UTILITIES_REQUIRED ON)
  SET(CPACK_COMPONENT_TUTORIALS_REQUIRED ON)
ELSE()
  SET(CPACK_COMPONENT_LIBRARIES_REQUIRED ON)
  SET(CPACK_COMPONENT_UTILITIES_DISABLED ON)
  SET(CPACK_COMPONENT_TUTORIALS_DISABLED ON)
ENDIF()

IF (APPLE)
  CONFIGURE_FILE(README.md README.txt)
  SET(CPACK_RESOURCE_FILE_README ${PROJECT_BINARY_DIR}/README.txt)
ELSE()
  SET(CPACK_RESOURCE_FILE_README ${PROJECT_SOURCE_DIR}/README.md)
ENDIF()
SET(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE.txt)
#SET(CPACK_PACKAGE_ICON ${PROJECT_SOURCE_DIR}/embree-doc/images/icon.png)
SET(CPACK_PACKAGE_FILE_NAME "embree-${EMBREE_VERSION}")


IF(WIN32)
  SET(CPACK_COMPONENT_REDIST_DISABLED ON)
  SET(CPACK_COMPONENT_TUTORIALS_DEPENDS redist)
  SET(CPACK_COMPONENT_UTILITIES_DEPENDS redist)
  # detect and select Win32 or x64
  IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ARCH x64)
    SET(PROGRAMFILES "\$PROGRAMFILES64")
  ELSE()
    SET(ARCH win32)
    SET(PROGRAMFILES "\$PROGRAMFILES")
  ENDIF()
  SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}-${ARCH}")
  SET(CPACK_PACKAGE_NAME Embree)

  # NSIS specific settings
  SET(CPACK_GENERATOR NSIS)
  SET(CPACK_NSIS_INSTALL_ROOT "${PROGRAMFILES}\\\\Intel")
  SET(CPACK_NSIS_DISPLAY_NAME "Embree: High Performance Ray Tracing Kernels")
  SET(CPACK_NSIS_PACKAGE_NAME "Embree ${EMBREE_VERSION}")
  SET(CPACK_NSIS_URL_INFO_ABOUT http://embree.github.io/)
#  SET(CPACK_NSIS_HELP_LINK http://embree.github.io/downloads.html#windows)
  SET(CPACK_NSIS_MUI_ICON ${PROJECT_SOURCE_DIR}/embree-doc/images/icon32.ico)
  SET(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
ELSEIF(APPLE)
  # MacOSX specific settings
  SET(CPACK_GENERATOR PackageMaker)
  SET(CPACK_OSX_PACKAGE_VERSION 10.7)
ELSE()
  SET(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}-${CPACK_RPM_PACKAGE_RELEASE}.x86_64")
  # RPM specific settings
  SET(CPACK_GENERATOR RPM) #TGZ DEB
  SET(CPACK_RPM_COMPONENT_INSTALL ON)
#  SET(CPACK_ARCHIVE_COMPONENT_INSTALL ON) # for TGZ
  SET(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0") # Apache Software License, Version 2.0
  SET(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
#  SET(CPACK_RPM_CHANGELOG_FILE "") # ChangeLog of the RPM; also CHANGELOG.md is not in the required format
  SET(CPACK_RPM_PACKAGE_ARCHITECTURE x86_64)
  SET(CPACK_RPM_PACKAGE_URL http://embree.github.io/)

  # need to use lower case component names
  SET(CPACK_RPM_devel_POST_INSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
  SET(CPACK_RPM_devel_POST_UNINSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
  SET(CPACK_RPM_devel_xeonphi_POST_INSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)
  SET(CPACK_RPM_devel_xeonphi_POST_UNINSTALL_SCRIPT_FILE ${PROJECT_SOURCE_DIR}/common/cmake/rpm_ldconfig.sh)

  
ENDIF()

INCLUDE(CPack)
