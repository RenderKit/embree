## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)
    
INCLUDE(tutorial)
ADD_TUTORIAL(user_geometry)
ADD_TUTORIAL_ISPC(user_geometry)
ADD_TUTORIAL_SYCL(user_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(user_geometry embree_user_geometry)