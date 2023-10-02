## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_TUTORIALS_GLFW)
  SET(IMGUI_LIBRARY imgui)
ENDIF()

ADD_LIBRARY(tutorial STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.cpp scene_device.cpp)
TARGET_LINK_LIBRARIES(tutorial sys math lexers scenegraph lights embree tasking ${GUI_LIBRARIES})
SET_PROPERTY(TARGET tutorial PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET tutorial APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_SYCL_SUPPORT)
  ADD_LIBRARY(tutorial_sycl STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.cpp scene_device.cpp)
  TARGET_LINK_LIBRARIES(tutorial_sycl sys math lexers scenegraph lights_sycl embree tasking ze_wrapper ${GUI_LIBRARIES})
  SET_PROPERTY(TARGET tutorial_sycl PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET tutorial_sycl APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST} ${CMAKE_CXX_FLAGS_SYCL}")
  TARGET_COMPILE_DEFINITIONS(tutorial_sycl PUBLIC EMBREE_SYCL_TUTORIAL)
ENDIF()

option(EMBREE_USE_GOOGLE_BENCHMARK "Use google benchmark (note: set benchmark_DIR to benchmark_install_dir/lib/cmake/benchmark)" OFF)
option(EMBREE_BUILD_GOOGLE_BENCHMARK_FROM_SOURCE "Force to download, build, and staticly link Google Benchmark" OFF)

IF(EMBREE_USE_GOOGLE_BENCHMARK)
  IF (NOT EMBREE_BUILD_GOOGLE_BENCHMARK_FROM_SOURCE)
    FIND_PACKAGE(benchmark QUIET)
  ENDIF()

  IF(NOT benchmark_FOUND)
    CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
    MESSAGE("-- google benchmark installation not found. download and build from source")

    INCLUDE(FetchContent)
    SET(FETCHCONTENT_QUIET OFF)

    SET(BENCHMARK_BUILD_32_BITS OFF CACHE INTERNAL "")
    SET(BENCHMARK_DOWNLOAD_DEPENDENCIES ON CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_ASSEMBLY_TESTS OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_DOXYGEN OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_EXCEPTIONS ON CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_INSTALL OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_LIBPFM OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_LTO OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_TESTING OFF CACHE INTERNAL "")
    SET(BENCHMARK_ENABLE_WERROR ON CACHE INTERNAL "")
    SET(BENCHMARK_FORCE_WERROR OFF CACHE INTERNAL "")
    SET(BENCHMARK_INSTALL_DOCS OFF CACHE INTERNAL "")
    SET(BENCHMARK_USE_BUNDLED_GTEST OFF CACHE INTERNAL "")
    SET(BENCHMARK_USE_LIBCXX OFF CACHE INTERNAL "")
    SET(BUILD_GMOCK OFF CACHE INTERNAL "")

    FetchContent_Declare(
      benchmark
      GIT_REPOSITORY https://github.com/google/benchmark.git
      GIT_TAG v1.7.1
    )

    FetchContent_GetProperties(benchmark)
    if(NOT benchmark_POPULATED)
      FetchContent_Populate(benchmark)
      # We want to build google benchmark and link statically, so no need to
      # install it as part of the Embree install targets.
      add_subdirectory(${benchmark_SOURCE_DIR} ${benchmark_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()

    MARK_AS_ADVANCED(FETCHCONTENT_BASE_DIR)
    MARK_AS_ADVANCED(FETCHCONTENT_FULLY_DISCONNECTED)
    MARK_AS_ADVANCED(FETCHCONTENT_QUIET)
    MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED)
    MARK_AS_ADVANCED(FETCHCONTENT_SOURCE_DIR_BENCHMARK)
    MARK_AS_ADVANCED(FETCHCONTENT_UPDATES_DISCONNECTED_BENCHMARK)
  ELSE()
    MESSAGE("-- google benchmark installation at ${benchmark_DIR}.")
  ENDIF()
ENDIF()

ADD_LIBRARY(embree_benchmark STATIC benchmark.cpp)
SET_PROPERTY(TARGET embree_benchmark PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET embree_benchmark APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
IF(EMBREE_USE_GOOGLE_BENCHMARK)
  TARGET_COMPILE_DEFINITIONS(embree_benchmark PUBLIC USE_GOOGLE_BENCHMARK)
  TARGET_LINK_LIBRARIES(embree_benchmark benchmark::benchmark benchmark::benchmark_main)
ENDIF()
TARGET_LINK_LIBRARIES(tutorial embree_benchmark)
IF (EMBREE_SYCL_SUPPORT)
  TARGET_LINK_LIBRARIES(tutorial_sycl embree_benchmark)
ENDIF()

ADD_LIBRARY(noise STATIC noise.cpp)
SET_PROPERTY(TARGET noise PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET noise APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_ISPC_SUPPORT)
  ADD_ISPC_LIBRARY(tutorial_ispc STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.ispc scene_device.cpp tasksys.cpp)
  TARGET_LINK_LIBRARIES(tutorial_ispc sys math lexers scenegraph lights_ispc embree tasking ${GUI_LIBRARIES})
  SET_PROPERTY(TARGET tutorial_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET tutorial_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

  ADD_ISPC_LIBRARY(noise_ispc STATIC noise.ispc)
  SET_TARGET_PROPERTIES(noise_ispc PROPERTIES LINKER_LANGUAGE CXX)
  SET_PROPERTY(TARGET noise_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET noise_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

  TARGET_LINK_LIBRARIES(tutorial_ispc embree_benchmark)
ENDIF()

IF(WIN32)
  GET_TARGET_PROPERTY(DLL_PATH_RELEASE tasking IMPORTED_LOCATION_RELEASE)
  GET_TARGET_PROPERTY(DLL_PATH_DEBUG tasking IMPORTED_LOCATION_DEBUG)
  IF (DLL_PATH_DEBUG OR DLL_PATH_RELEASE)
    SET(DLL_PATH $<$<CONFIG:Debug>:${DLL_PATH_DEBUG}>$<$<NOT:$<CONFIG:Debug>>:${DLL_PATH_RELEASE}>)
    ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${DLL_PATH} $<TARGET_FILE_DIR:tutorial>
      COMMENT "Copying TBB DLL" VERBATIM
    )
  ENDIF()
ENDIF()

ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/../../models" "$<TARGET_FILE_DIR:tutorial>/models"
  COMMENT "Copying example models")


