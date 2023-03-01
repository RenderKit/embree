## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_RAY_MASK)

INCLUDE(tutorial)
ADD_TUTORIAL(ray_mask)
ADD_TUTORIAL_ISPC(ray_mask)
ADD_TUTORIAL_SYCL(ray_mask)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TUTORIAL_TEST(ray_mask "--frames 128")
ENDIF()

ENDIF()
