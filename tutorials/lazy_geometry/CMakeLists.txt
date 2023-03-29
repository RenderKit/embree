## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)

INCLUDE(tutorial)
ADD_TUTORIAL(lazy_geometry)
ADD_TUTORIAL_ISPC(lazy_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(lazy_geometry embree_lazy_geometry)