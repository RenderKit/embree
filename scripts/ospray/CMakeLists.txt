## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

## Global settings ##

cmake_minimum_required(VERSION 3.1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX
      "${CMAKE_BINARY_DIR}/install"
      CACHE STRING "Final install location." FORCE)
endif()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Establish project ##

project(OSPRay_superbuild)

include(ExternalProject)
include(GNUInstallDirs)
include(ProcessorCount)

## Superbuild options ##

set(BUILD_RKCOMMON_VERSION "v1.6.0" CACHE STRING "Which version of rkcommon to build?")
set(TBB_VERSION "2021.2.0" CACHE STRING "Which version of TBB to download?")
set(BUILD_OPENVKL_VERSION "v0.12.1" CACHE STRING "Which version of OpenVKL to build?")

set(installDir ${CMAKE_INSTALL_PREFIX})
get_filename_component(INSTALL_DIR_ABSOLUTE ${installDir} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})

ProcessorCount(PROCESSOR_COUNT)
if(NOT PROCESSOR_COUNT EQUAL 0)
  SET(PROCESSOR_COUNT 1)
endif()

if (${CMAKE_VERSION} VERSION_GREATER 3.11.4)
  set(PARALLEL_JOBS_OPTS -j ${BUILD_JOBS})
endif()

set(DEFAULT_BUILD_COMMAND cmake --build . --config release ${PARALLEL_JOBS_OPTS})

set(COMPONENT_PATH ${INSTALL_DIR_ABSOLUTE}) # install all components in one directory
list(APPEND CMAKE_PREFIX_PATH ${COMPONENT_PATH})
string(REPLACE ";" "|" CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")

set(ISPC_VERSION "v1.15.0")
set(ISPC_BASE_URL "https://github.com/ispc/ispc/releases/download")
if (APPLE)
  set(ISPC_URL ${ISPC_BASE_URL}/${ISPC_VERSION}/ispc-${ISPC_VERSION}-macOS.tar.gz)
elseif(WIN32)
  set(ISPC_URL ${ISPC_BASE_URL}/${ISPC_VERSION}/ispc-${ISPC_VERSION}-windows.zip)
else()
  set(ISPC_URL ${ISPC_BASE_URL}/${ISPC_VERSION}/ispc-${ISPC_VERSION}-linux.tar.gz)
endif()

ExternalProject_Add(ispc
  PREFIX ispc
  STAMP_DIR ispc/stamp
  SOURCE_DIR ispc/src
  BINARY_DIR ispc
  URL ${ISPC_URL}
  DOWNLOAD_NO_PROGRESS ON
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "${CMAKE_COMMAND}" -E copy_if_different
    <SOURCE_DIR>/bin/ispc${CMAKE_EXECUTABLE_SUFFIX}
    ${COMPONENT_PATH}/bin/ispc${CMAKE_EXECUTABLE_SUFFIX}
  BUILD_ALWAYS OFF
)

set(ISPC_PATH "${COMPONENT_PATH}/bin/ispc${CMAKE_EXECUTABLE_SUFFIX}")

if (APPLE)
  set(TBB_URL "http://github.com/intel/tbb/releases/download/v${TBB_VERSION}/oneapi-tbb-${TBB_VERSION}-mac.tgz")
elseif (WIN32)
  set(TBB_URL "http://github.com/intel/tbb/releases/download/v${TBB_VERSION}/oneapi-tbb-${TBB_VERSION}-win.zip")
else()
  set(TBB_URL "http://github.com/intel/tbb/releases/download/v${TBB_VERSION}/oneapi-tbb-${TBB_VERSION}-lin.tgz")
endif()

ExternalProject_Add(tbb
  PREFIX tbb
  DOWNLOAD_DIR tbb
  STAMP_DIR tbb/stamp
  SOURCE_DIR tbb/src
  BINARY_DIR tbb
  URL ${TBB_URL}
  DOWNLOAD_NO_PROGRESS ON
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "${CMAKE_COMMAND}" -E copy_directory
    <SOURCE_DIR>
    ${COMPONENT_PATH}
  BUILD_ALWAYS OFF
)

set(TBB_PATH "${COMPONENT_PATH}")

ExternalProject_Add(rkcommon
  PREFIX rkcommon
  DOWNLOAD_DIR rkcommon
  STAMP_DIR rkcommon/stamp
  SOURCE_DIR rkcommon/src
  BINARY_DIR rkcommon/build
  URL "http://github.com/ospray/rkcommon/archive/${BUILD_RKCOMMON_VERSION}.zip"
  DOWNLOAD_NO_PROGRESS ON
  CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=${COMPONENT_PATH}
    -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_INCLUDEDIR}
    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -DCMAKE_INSTALL_DOCDIR=${CMAKE_INSTALL_DOCDIR}
    -DCMAKE_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}
    -DCMAKE_BUILD_TYPE=Release
    -DINSTALL_DEPS=OFF
    -DBUILD_TESTING=OFF
    -DRKCOMMON_TBB_ROOT=${TBB_PATH}
  BUILD_COMMAND ${DEFAULT_BUILD_COMMAND}
  BUILD_ALWAYS ${ALWAYS_REBUILD}
)

ExternalProject_Add_StepDependencies(rkcommon configure tbb)

ExternalProject_Add(embree
    PREFIX embree
    SOURCE_DIR ${INSTALL_DIR_ABSOLUTE}/../../embree
    STAMP_DIR embree/stamp
    BINARY_DIR embree
    URL ${EMBREE_URL}
    DOWNLOAD_NO_PROGRESS ON
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND "${CMAKE_COMMAND}" -E copy_directory
      <SOURCE_DIR>/
      ${COMPONENT_PATH}
    BUILD_ALWAYS OFF
  )

ExternalProject_Add(openvkl
  PREFIX openvkl
  DOWNLOAD_DIR openvkl
  STAMP_DIR openvkl/stamp
  SOURCE_DIR openvkl/src
  BINARY_DIR openvkl/build
  LIST_SEPARATOR | # Use the alternate list separator
  URL "http://github.com/openvkl/openvkl/archive/${BUILD_OPENVKL_VERSION}.zip"
  DOWNLOAD_NO_PROGRESS ON
  CMAKE_ARGS
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=${COMPONENT_PATH}
    -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_INCLUDEDIR}
    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -DCMAKE_INSTALL_DOCDIR=${CMAKE_INSTALL_DOCDIR}
    -DCMAKE_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}
    -DCMAKE_BUILD_TYPE=Release
    -DRKCOMMON_TBB_ROOT=${TBB_PATH}
    -DISPC_EXECUTABLE=${ISPC_PATH}
    -DBUILD_BENCHMARKS=OFF
    -DBUILD_EXAMPLES=OFF
    -DBUILD_TESTING=OFF
  BUILD_COMMAND ${DEFAULT_BUILD_COMMAND}
  BUILD_ALWAYS ${ALWAYS_REBUILD}
)

ExternalProject_Add_StepDependencies(openvkl
  configure
    rkcommon
    embree
    ispc
)

ExternalProject_Add(glfw
  PREFIX glfw
  DOWNLOAD_DIR glfw
  STAMP_DIR glfw/stamp
  SOURCE_DIR glfw/src
  BINARY_DIR glfw/build
  URL "http://github.com/glfw/glfw/releases/download/3.2.1/glfw-3.2.1.zip"
  DOWNLOAD_NO_PROGRESS ON
  CMAKE_ARGS
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=${COMPONENT_PATH}
    -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_INCLUDEDIR}
    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -DCMAKE_INSTALL_DOCDIR=${CMAKE_INSTALL_DOCDIR}
    -DCMAKE_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}
    -DCMAKE_BUILD_TYPE=Release
    -DGLFW_BUILD_DOCS=OFF
    -DGLFW_BUILD_EXAMPLES=OFF
    -DGLFW_BUILD_TESTS=OFF
  BUILD_COMMAND ${DEFAULT_BUILD_COMMAND}
  BUILD_ALWAYS ${ALWAYS_REBUILD}
)

ExternalProject_Add(ospray
  PREFIX ospray
  URL "https://github.com/ospray/ospray/archive/v2.5.0.tar.gz"
  DOWNLOAD_NO_PROGRESS ON
  GIT_SUBMODULES ""
  STAMP_DIR ospray/stamp
  BINARY_DIR ospray/build
  LIST_SEPARATOR | # Use the alternate list separator
  CMAKE_ARGS
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=${COMPONENT_PATH}
    -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_INCLUDEDIR}
    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -DCMAKE_INSTALL_DOCDIR=${CMAKE_INSTALL_DOCDIR}
    -DCMAKE_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}
    -DOSPRAY_BUILD_ISA=ALL
    -DOSPRAY_ENABLE_TESTING=ON
    -DOSPRAY_MODULE_MULTIHIT=OFF
    -DOSPRAY_MODULE_BILINEAR_PATCH=ON
    -DOSPRAY_MODULE_DENOISER=OFF
    -DOSPRAY_STRICT_BUILD=ON
    -DOSPRAY_WARN_AS_ERRORS=OFF
    -DISPC_EXECUTABLE=${ISPC_PATH}
    -DRKCOMMON_TBB_ROOT=${TBB_PATH}
  BUILD_COMMAND ${DEFAULT_BUILD_COMMAND}
  BUILD_ALWAYS OFF
)

ExternalProject_Add_StepDependencies(ospray
  configure
  rkcommon
  embree
  openvkl
  glfw
  ispc
)
