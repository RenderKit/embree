## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

INCLUDE(tutorial)
ADD_TUTORIAL(motion_blur_geometry)
IF (EMBREE_RAY_PACKETS)
  ADD_TUTORIAL_ISPC(motion_blur_geometry)
ENDIF()

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  IF (NOT ${EMBREE_RAY_PACKETS})
    SET(DISABLE_ISPC_TEST ON)
  ENDIF()
  ADD_EMBREE_TEST2(motion_blur_geometry motion_blur_geometry "--time 0.5")
ENDIF()

