## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_CURVE)
  
INCLUDE(tutorial)
ADD_TUTORIAL(interpolation)
ADD_TUTORIAL_ISPC(interpolation)

ENDIF()

ADD_EMBREE_TEST_ECS(interpolation embree_interpolation)