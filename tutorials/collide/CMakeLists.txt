## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)
  
INCLUDE(tutorial)
ADD_TUTORIAL(collide constraints.cpp)

ENDIF()

ADD_EMBREE_TEST_ECS(collide embree_collide NO_REFERENCE NO_ISPC NO_SYCL ARGS --benchmark 0 128)