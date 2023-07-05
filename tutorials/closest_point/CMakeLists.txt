## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_CURVE)
  
INCLUDE(tutorial)
ADD_TUTORIAL(closest_point)
ADD_TUTORIAL_ISPC(closest_point)

ENDIF()

ADD_EMBREE_TEST_ECS(closest_point embree_closest_point)