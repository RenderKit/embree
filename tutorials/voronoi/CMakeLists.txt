## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)
 
INCLUDE(tutorial)
ADD_TUTORIAL(voronoi)
ADD_TUTORIAL_ISPC(voronoi)

ENDIF()

ADD_EMBREE_TEST_ECS(voronoi embree_voronoi)