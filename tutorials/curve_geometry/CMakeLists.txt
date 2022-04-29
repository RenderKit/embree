## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_CURVE)

INCLUDE(tutorial)
ADD_TUTORIAL(curve_geometry)
ADD_TUTORIAL_ISPC(curve_geometry)
ADD_TUTORIAL_SYCL(curve_geometry)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(curve_geometry)
ENDIF()

ENDIF()
