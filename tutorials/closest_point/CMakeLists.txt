## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(closest_point)
ADD_TUTORIAL_ISPC(closest_point)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(closest_point)
ENDIF()


