## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_INSTANCE AND EMBREE_MAX_INSTANCE_LEVEL_COUNT GREATER 1)

INCLUDE(tutorial)
ADD_TUTORIAL(multi_instanced_geometry scene.cpp)
ADD_TUTORIAL_ISPC(multi_instanced_geometry scene.cpp)
ADD_TUTORIAL_SYCL(multi_instanced_geometry scene.cpp)

ENDIF()

ADD_EMBREE_TEST_ECS(multi_instanced_geometry embree_multi_instanced_geometry)