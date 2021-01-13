## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(intersection_filter)
ADD_TUTORIAL_ISPC(intersection_filter)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(intersection_filter)
  IF(EMBREE_RAY_PACKETS)
    ADD_EMBREE_TEST2(intersection_filter_stream_coherent   intersection_filter "--mode stream --coherent")
    ADD_EMBREE_TEST2(intersection_filter_stream_incoherent intersection_filter "--mode stream --incoherent")
  ENDIF()
ENDIF()

