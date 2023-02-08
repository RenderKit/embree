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
  TARGET_LINK_LIBRARIES(tutorial_sycl sys math lexers scenegraph lights_sycl embree tasking ${GUI_LIBRARIES})
  SET_PROPERTY(TARGET tutorial_sycl PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET tutorial_sycl APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST} ${CMAKE_CXX_FLAGS_SYCL}")
  TARGET_COMPILE_DEFINITIONS(tutorial_sycl PUBLIC EMBREE_SYCL_TUTORIAL)
ENDIF()

option(EMBREE_USE_GOOGLE_BENCHMARK "Use google benchmark (note: set benchmark_DIR to benchmark_install_dir/lib/cmake/benchmark)" OFF)

IF(EMBREE_USE_GOOGLE_BENCHMARK)
  FIND_PACKAGE(benchmark REQUIRED)
  IF (NOT TARGET benchmark::benchmark)
    message(FATAL_ERROR "google benchmark not found. set benchmark_DIR to benchmark_install_dir/lib/cmake/benchmark")
  ENDIF()
ENDIF()

ADD_LIBRARY(benchmark STATIC benchmark.cpp)
SET_PROPERTY(TARGET benchmark PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET benchmark APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
IF(EMBREE_USE_GOOGLE_BENCHMARK)
  TARGET_COMPILE_DEFINITIONS(benchmark PUBLIC USE_GOOGLE_BENCHMARK)
  TARGET_LINK_LIBRARIES(benchmark benchmark::benchmark benchmark::benchmark_main)
ENDIF()
TARGET_LINK_LIBRARIES(tutorial benchmark)
IF (EMBREE_SYCL_SUPPORT)
  TARGET_LINK_LIBRARIES(tutorial_sycl benchmark)
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

  TARGET_LINK_LIBRARIES(tutorial_ispc benchmark)
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


