## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF(EMBREE_GEOMETRY_GRID)

INCLUDE(tutorial)
ADD_TUTORIAL(grid_geometry)
ADD_TUTORIAL_ISPC(grid_geometry)
ADD_TUTORIAL_SYCL(grid_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(grid_geometry embree_grid_geometry)