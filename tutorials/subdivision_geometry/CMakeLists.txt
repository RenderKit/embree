## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_SUBDIVISION)

INCLUDE(tutorial)
ADD_TUTORIAL(subdivision_geometry)
ADD_TUTORIAL_ISPC(subdivision_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(subdivision_geometry embree_subdivision_geometry)