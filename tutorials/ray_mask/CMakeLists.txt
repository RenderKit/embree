## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_RAY_MASK)

INCLUDE(tutorial)
ADD_TUTORIAL(ray_mask)
ADD_TUTORIAL_ISPC(ray_mask)
ADD_TUTORIAL_SYCL(ray_mask)

ENDIF()

ADD_EMBREE_TEST_ECS(ray_mask embree_ray_mask ARGS --frames 128)