## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF(EMBREE_GEOMETRY_POINT)

INCLUDE(tutorial)
ADD_TUTORIAL(point_geometry)
ADD_TUTORIAL_ISPC(point_geometry)
ADD_TUTORIAL_SYCL(point_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(point_geometry embree_point_geometry)