## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_INSTANCE)

INCLUDE(tutorial)
ADD_TUTORIAL(instanced_geometry)
ADD_TUTORIAL_ISPC(instanced_geometry)
ADD_TUTORIAL_SYCL(instanced_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(instanced_geometry embree_instanced_geometry)