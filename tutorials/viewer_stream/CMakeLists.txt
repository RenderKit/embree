## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(viewer_stream)
ADD_TUTORIAL_ISPC(viewer_stream)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_coherent viewer viewer_stream --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_incoherent viewer viewer_stream --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_quad_coherent viewer viewer_stream -convert-triangles-to-quads)
  ADD_EMBREE_MODELS_TEST(test-models-intensive2.txt viewer_stream_quad_incoherent viewer viewer_stream -convert-triangles-to-quads --incoherent)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_coherent viewer viewer_stream --coherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_incoherent viewer viewer_stream --incoherent)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_quad_coherent viewer viewer_stream -convert-triangles-to-quads)
  ADD_EMBREE_MODELS_TEST(test-models-intensive3.txt viewer_stream_quad_incoherent viewer viewer_stream -convert-triangles-to-quads --incoherent)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_EMBREE_MODELS_TEST(test-models-subdiv.txt viewer_stream viewer viewer_stream)
  ADD_EMBREE_MODELS_TEST(test-models-intensity2.txt viewer_stream viewer viewer_stream)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 3)
  ADD_EMBREE_MODELS_TEST(test-models-intensity3.txt viewer_stream viewer viewer_stream)
ENDIF()

CMAKE_HOST_SYSTEM_INFORMATION(RESULT memory QUERY TOTAL_PHYSICAL_MEMORY)
IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 3 AND ${memory} GREATER 10000)
  ADD_EMBREE_MODELS_TEST(test-models-intensity4.txt viewer_stream viewer viewer_stream)
ENDIF()
