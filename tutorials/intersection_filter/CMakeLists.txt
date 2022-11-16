## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_FILTER_FUNCTION)

INCLUDE(tutorial)
ADD_TUTORIAL(intersection_filter)
ADD_TUTORIAL_ISPC(intersection_filter)
ADD_TUTORIAL_SYCL(intersection_filter)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(intersection_filter)
ENDIF()

ENDIF()
