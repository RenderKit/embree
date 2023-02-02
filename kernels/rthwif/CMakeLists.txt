## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.1.0)

project(embree_rthwif)

IF(COMMAND cmake_policy)
  if(POLICY CMP0135)
    message("set policy CMP0135 to NEW")
    cmake_policy(SET CMP0135 NEW)
  endif()
ENDIF()

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

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-bitwise-instead-of-logical") # disables "use of bitwise '&' with boolean operands" warning
SET(CMAKE_CXX_FLAGS_SYCL "${CMAKE_CXX_FLAGS_SYCL} -Wno-bitwise-instead-of-logical") # disables "use of bitwise '&' with boolean operands" warning

option(EMBREE_BUILDER_TBB_STATIC "Use a staticly compiled TBB version for the Embree builder for GPU." OFF)

# by default link against the tasking target that has all TBB related
# information we need when TASKING_TBB is used
set(TBB_TARGET tasking)

if (EMBREE_BUILDER_TBB_STATIC OR NOT TASKING_TBB)

  ####################################################################
  # fetch TBB and build static version of it
  set(TBB_TARGET tbb)
  ADD_DEFINITIONS("-D_CRT_SECURE_NO_WARNINGS")

  option(TBB_STRICT "Treat compiler warnings as errors" OFF)
  option(TBB_TEST "Enable testing" OFF)
  option(TBBMALLOC_BUILD "Enable tbbmalloc build" OFF)
  SET(TBB_DIR OFF)
  SET(BUILD_SHARED_LIBS OFF)

  INCLUDE(FetchContent)

  SET(FETCHCONTENT_QUIET OFF)

  IF (NOT EMBREE_RTHWIF_TBB_GIT_REPOSITORY) # allow setting this externally
    SET(EMBREE_RTHWIF_TBB_GIT_REPOSITORY "https://github.com/oneapi-src/oneTBB.git")
  ENDIF()

  FetchContent_Declare(
    tbb_static
    GIT_REPOSITORY ${EMBREE_RTHWIF_TBB_GIT_REPOSITORY}
    GIT_TAG v2021.6.0
  )
  FetchContent_MakeAvailable(tbb_static)

  # We want to build tbb_static to link it into embree_rthwif, but don't want to
  # install it as part of the Embree install targets.
  if (IS_DIRECTORY "${tbb_static_SOURCE_DIR}")
    set_property(DIRECTORY ${tbb_static_SOURCE_DIR} PROPERTY EXCLUDE_FROM_ALL YES)
  endif()

  MARK_AS_ADVANCED(FETCHCONTENT_BASE_DIR)
  MARK_AS_ADVANCED(FETCHCONTENT_FULLY_DISCONNECTED)
  MARK_AS_ADVANCED(FETCHCONTENT_QUIET)
  MARK_AS_ADVANCED(FETCHCONTENT_SOURCE_DIR_TBB_STATIC)
  MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED)
  MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED_TBB_STATIC)

  MARK_AS_ADVANCED(TBB4PY_BUILD)
  MARK_AS_ADVANCED(TBBMALLOC_BUILD)
  MARK_AS_ADVANCED(TBB_BUILD)
  MARK_AS_ADVANCED(TBB_CPF)
  MARK_AS_ADVANCED(TBB_DISABLE_HWLOC_AUTOMATIC_SEARCH)
  MARK_AS_ADVANCED(TBB_ENABLE_IPO)
  MARK_AS_ADVANCED(TBB_EXAMPLES)
  MARK_AS_ADVANCED(TBB_FIND_PACKAGE)
  MARK_AS_ADVANCED(TBB_INSTALL_VARS)
  MARK_AS_ADVANCED(TBB_NO_APPCONTAINER)
  MARK_AS_ADVANCED(TBB_SANITIZE)
  MARK_AS_ADVANCED(TBB_STRICT)
  MARK_AS_ADVANCED(TBB_TEST)
  MARK_AS_ADVANCED(TBB_TEST_SPEC)
  MARK_AS_ADVANCED(TBB_VALGRIND_MEMCHECK)
  MARK_AS_ADVANCED(TBB_WINDOWS_DRIVER)

  ADD_DEFINITIONS(-DTASKING_TBB)
  ####################################################################
ENDIF()

IF (EMBREE_SYCL_RT_VALIDATION_API)
  ADD_LIBRARY(embree_rthwif_sycl STATIC rthwif_production.cpp)
  SET_PROPERTY(TARGET embree_rthwif_sycl APPEND PROPERTY COMPILE_FLAGS "-DEMBREE_SYCL_SUPPORT")
  SET_TARGET_PROPERTIES(embree_rthwif_sycl PROPERTIES COMPILE_FLAGS ${CMAKE_CXX_FLAGS_SYCL})

  INSTALL(TARGETS embree_rthwif_sycl EXPORT embree_rthwif_sycl-targets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib)
  INSTALL(EXPORT embree_rthwif_sycl-targets DESTINATION "${EMBREE_CMAKEEXPORT_DIR}" COMPONENT devel)

  SET(EMBREE_RTHWIF_SYCL embree_rthwif_sycl)
ENDIF()

ADD_LIBRARY(embree_rthwif STATIC rthwif_builder.cpp builder/qbvh6.cpp builder/statistics.cpp)
TARGET_LINK_LIBRARIES(embree_rthwif PUBLIC ${EMBREE_RTHWIF_SYCL} PRIVATE ${TBB_TARGET} simd sys)
if (EMBREE_LEVEL_ZERO)
  TARGET_LINK_LIBRARIES(embree_rthwif PUBLIC -lze_loader)
  TARGET_COMPILE_DEFINITIONS(embree_rthwif PUBLIC EMBREE_LEVEL_ZERO)
  TARGET_COMPILE_DEFINITIONS(embree_rthwif PUBLIC EMBREE_SYCL_SUPPORT)
endif()

IF (EMBREE_STATIC_LIB)
  INSTALL(TARGETS embree_rthwif EXPORT embree_rthwif-targets
    LIBRARY NAMELINK_SKIP DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
  INSTALL(EXPORT embree_rthwif-targets DESTINATION "${EMBREE_CMAKEEXPORT_DIR}" COMPONENT devel)
ENDIF()

IF (EMBREE_LEVEL_ZERO)
  ADD_SUBDIRECTORY(testing)
ENDIF()

