## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER AND EMBREE_GEOMETRY_CURVE)
  
INCLUDE(tutorial)
ADD_TUTORIAL(motion_blur_geometry)
ADD_TUTORIAL_ISPC(motion_blur_geometry)
ADD_TUTORIAL_SYCL(motion_blur_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(motion_blur_geometry embree_motion_blur_geometry --time 0.5)