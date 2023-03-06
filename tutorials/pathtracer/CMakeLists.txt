## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_AOT_DEVICES STREQUAL "none")
  ADD_DEFINITIONS("-DUSE_SPECIALIZATION_CONSTANTS")
ENDIF()

INCLUDE(tutorial)
ADD_TUTORIAL(pathtracer)
ADD_TUTORIAL_ISPC(pathtracer)
ADD_TUTORIAL_SYCL(pathtracer)

IF (NOT EMBREE_SYCL_AOT_DEVICES STREQUAL "none") # see issue #1209

  IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
    ADD_EMBREE_MODELS_TEST(tests_pathtracer.txt pathtracer            embree_pathtracer pathtracer "")
  ENDIF()

  IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
    ADD_EMBREE_MODELS_TEST(tests_pathtracer.txt pathtracer_coherent   embree_pathtracer pathtracer "--coherent")
  ENDIF()

ENDIF()
