## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF(EMBREE_GEOMETRY_SUBDIVISION)

INCLUDE(tutorial)
ADD_TUTORIAL(displacement_geometry)
ADD_TUTORIAL_ISPC(displacement_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(displacement_geometry embree_displacement_geometry)