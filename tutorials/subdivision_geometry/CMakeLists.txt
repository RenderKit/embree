## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_SUBDIVISION)

INCLUDE(tutorial)
ADD_TUTORIAL(subdivision_geometry)
ADD_TUTORIAL_ISPC(subdivision_geometry)

SET(EMBREE_SYCL_SUPPORT OFF)
IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(subdivision_geometry)
ENDIF()

ENDIF()
