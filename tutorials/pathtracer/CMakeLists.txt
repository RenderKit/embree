## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(pathtracer)
IF (EMBREE_RAY_PACKETS)
  ADD_TUTORIAL_ISPC(pathtracer)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt pathtracer_coherent pathtracer pathtracer --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt pathtracer pathtracer pathtracer)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 2)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt pathtracer_coherent pathtracer pathtracer --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt pathtracer pathtracer pathtracer)
ENDIF()

CMAKE_HOST_SYSTEM_INFORMATION(RESULT memory QUERY TOTAL_PHYSICAL_MEMORY)
IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 3 AND ${memory} GREATER 10000)
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt pathtracer pathtracer pathtracer)
ENDIF()
