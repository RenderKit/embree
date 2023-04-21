## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)

INCLUDE(tutorial)
ADD_TUTORIAL(quaternion_motion_blur)
ADD_TUTORIAL_ISPC(quaternion_motion_blur)
ADD_TUTORIAL_SYCL(quaternion_motion_blur)

ENDIF()

ADD_EMBREE_TEST_ECS(quaternion_motion_blur embree_quaternion_motion_blur)