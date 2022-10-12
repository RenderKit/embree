## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

ADD_LIBRARY(embree_rthwif_gpu STATIC rthwif_embree_builder.cpp builder/qbvh6.cpp builder/statistics.cpp rthwif_embree_builder_ploc.cpp)
TARGET_LINK_LIBRARIES(embree_rthwif_gpu PRIVATE tasking)
SET_TARGET_PROPERTIES(embree_rthwif_gpu PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_SYCL})

####################################################################
# fetch TBB and build static version of it

option(TBB_STRICT "Treat compiler warnings as errors" OFF)
option(TBB_TEST "Enable testing" OFF)
option(TBBMALLOC_BUILD "Enable tbbmalloc build" OFF)
SET(TBB_DIR OFF)
SET(BUILD_SHARED_LIBS OFF)

INCLUDE(FetchContent)

FetchContent_Declare(
  tbb_static
  URL https://github.com/oneapi-src/oneTBB/archive/refs/tags/v2021.6.0.zip
)

FetchContent_GetProperties(tbb_static)

IF (NOT tbb_static_POPULATED)
  FetchContent_Populate(tbb_static)
  add_subdirectory(${tbb_static_SOURCE_DIR} ${tbb_static_BINARY_DIR})
ENDIF()

ADD_DEFINITIONS(-DTASKING_TBB)

####################################################################

ADD_LIBRARY(embree_rthwif SHARED rthwif_builder.cpp builder/qbvh6.cpp builder/statistics.cpp ../common/alloc.cpp)
TARGET_LINK_LIBRARIES(embree_rthwif PRIVATE embree_rthwif_gpu tbb simd sys -lze_loader)

INSTALL(TARGETS embree_rthwif EXPORT embree_rthwif-targets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib)
INSTALL(EXPORT embree_rthwif-targets DESTINATION "${EMBREE_CMAKEEXPORT_DIR}" COMPONENT devel)

IF (NOT WIN32 AND NOT EMBREE_GFX_DRIVER STREQUAL "PUBLIC") # FIXME: see issue #1213
  ADD_SUBDIRECTORY(testing)
ENDIF()

#ADD_LIBRARY(rthwif_sycl STATIC rthwif_embree.cpp rthwif_production.cpp)
#SET_TARGET_PROPERTIES(rthwif_sycl PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_SYCL})

#IF (EMBREE_STATIC_LIB)
#  INSTALL(TARGETS rthwif EXPORT rthwif-targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT devel)
#  INSTALL(EXPORT rthwif-targets DESTINATION ${EMBREE_CMAKEEXPORT_DIR} COMPONENT devel)
#
##  INSTALL(TARGETS rthwif_sycl EXPORT rthwif_sycl-targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT devel)
##  INSTALL(EXPORT rthwif_sycl-targets DESTINATION ${EMBREE_CMAKEEXPORT_DIR} COMPONENT devel)
#ENDIF()
