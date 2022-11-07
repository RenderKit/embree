## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.1.0)

project(embree_rthwif)

SET(CMAKE_CXX_STANDARD 17)

#####################################################################
# This is required when compiling this folder stand alone

#SET(OPTION BUILD_TESTING ON)
INCLUDE(CTest)

IF (NOT DEFINED EMBREE_CMAKEEXPORT_DIR)
  SET(EMBREE_CMAKEEXPORT_DIR "cmake")
ENDIF()

IF (NOT DEFINED EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS)
  OPTION(EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS "Using L0 allocated Dispatch Globals" ON)
ENDIF()

IF (NOT EMBREE_SYCL_IMPLICIT_DISPATCH_GLOBALS)
  ADD_DEFINITIONS("-DEMBREE_SYCL_ALLOC_DISPATCH_GLOBALS")
ENDIF()

IF (NOT DEFINED EMBREE_SYCL_SUPPORT)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")                       # generate position independent code suitable for shared libraries
  
  ADD_SUBDIRECTORY(../../common/sys sys)
  ADD_SUBDIRECTORY(../../common/simd simd)

  GET_FILENAME_COMPONENT(SYCL_COMPILER_DIR ${CMAKE_CXX_COMPILER} PATH)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem \"${SYCL_COMPILER_DIR}/../include/sycl\" -isystem \"${SYCL_COMPILER_DIR}/../include/\"")       # disable warning from SYCL header (FIXME: why required?)
ENDIF()

####################################################################
# fetch TBB and build static version of it

option(TBB_STRICT "Treat compiler warnings as errors" OFF)
option(TBB_TEST "Enable testing" OFF)
option(TBBMALLOC_BUILD "Enable tbbmalloc build" OFF)
SET(TBB_DIR OFF)
SET(BUILD_SHARED_LIBS OFF)

INCLUDE(FetchContent)

SET(FETCHCONTENT_QUIET OFF)

FetchContent_Declare(
  tbb_static
  URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.6.0.zip
)
FetchContent_MakeAvailable(tbb_static)

# We want to build tbb_static to link it into embree_rthwif, but don't want to
# install it as part of the Embree install targets.
if (IS_DIRECTORY "${tbb_static_SOURCE_DIR}")
  set_property(DIRECTORY ${tbb_static_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL YES)
endif()

ADD_DEFINITIONS(-DTASKING_TBB)

####################################################################

ADD_LIBRARY(embree_rthwif_gpu STATIC rthwif_embree_builder_ploc.cpp builder/qbvh6.cpp builder/statistics.cpp)
SET_TARGET_PROPERTIES(embree_rthwif_gpu PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_SYCL})

#ADD_LIBRARY(embree_rthwif SHARED rthwif_builder.cpp builder/qbvh6.cpp builder/statistics.cpp ../common/alloc.cpp)
#TARGET_LINK_LIBRARIES(embree_rthwif PRIVATE embree_rthwif_gpu tbb simd sys -lze_loader)

ADD_LIBRARY(embree_rthwif SHARED rthwif_builder.cpp builder/qbvh6.cpp builder/statistics.cpp)
TARGET_LINK_LIBRARIES(embree_rthwif PRIVATE embree_rthwif_gpu tbb simd sys -lze_loader)
SET_PROPERTY(TARGET embree_rthwif APPEND PROPERTY COMPILE_FLAGS "-DEMBREE_SYCL_SUPPORT")

INSTALL(TARGETS embree_rthwif EXPORT embree_rthwif-targets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib)
INSTALL(EXPORT embree_rthwif-targets DESTINATION "${EMBREE_CMAKEEXPORT_DIR}" COMPONENT devel)

IF (NOT WIN32 AND NOT EMBREE_GFX_DRIVER STREQUAL "PUBLIC") # FIXME: see issue #1213
  ADD_SUBDIRECTORY(testing)
ENDIF()

