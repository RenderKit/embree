## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER AND EMBREE_GEOMETRY_CURVE)
  
INCLUDE(tutorial)
ADD_TUTORIAL(motion_blur_geometry)
ADD_TUTORIAL_ISPC(motion_blur_geometry)
ADD_TUTORIAL_SYCL(motion_blur_geometry)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST2(motion_blur_geometry motion_blur_geometry "--time 0.5")
ENDIF()

ENDIF()
