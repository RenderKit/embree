## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_INSTANCE_ARRAY)

INCLUDE(tutorial)
ADD_TUTORIAL(forest)
ADD_TUTORIAL_ISPC(forest)
ADD_TUTORIAL_SYCL(forest)

ADD_EMBREE_TEST_ECS(forest embree_forest ARGS --frames 4)

ENDIF()
