## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(multi_instanced_geometry scene.cpp)
ADD_TUTORIAL_ISPC(multi_instanced_geometry scene.cpp)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(multi_instanced_geometry)
ENDIF()

