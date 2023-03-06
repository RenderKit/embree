## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
  ADD_DEFINITIONS("-DUSE_SPECIALIZATION_CONSTANTS")
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(viewer viewer_device_debug.cpp)
ADD_TUTORIAL_ISPC(viewer viewer_device_debug.ispc)
ADD_TUTORIAL_SYCL(viewer viewer_device_debug.cpp)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(tests_all.txt viewer embree_viewer viewer "")
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
  ADD_EMBREE_MODELS_TEST(tests_all.txt viewer_coherent      embree_viewer viewer "--coherent")
  ADD_EMBREE_MODELS_TEST(tests_all.txt viewer_quad_coherent embree_viewer viewer_quad_coherent "-convert-triangles-to-quads --coherent")
  ADD_EMBREE_MODELS_TEST(tests_all.txt viewer_grid_coherent embree_viewer viewer_quad_coherent "-convert-triangles-to-grids --coherent")
ENDIF()

IF (EMBREE_MIN_WIDTH AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(tests_min_width.txt viewer embree_viewer viewer "")
ENDIF()