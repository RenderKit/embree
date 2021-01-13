## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

SET(EMBREE_ISPC_SUPPORT OFF)

INCLUDE(tutorial)
ADD_TUTORIAL(multiscene_geometry)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(multiscene_geometry)
ENDIF()
