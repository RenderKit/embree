## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)
  
INCLUDE(tutorial)
ADD_TUTORIAL(collide constraints.cpp)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 1)
  ADD_TEST(NAME collide COMMAND embree_collide --benchmark 0 128)
ENDIF()

ENDIF()
