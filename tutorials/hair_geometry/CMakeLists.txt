## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER AND EMBREE_GEOMETRY_CURVE)

INCLUDE(tutorial)
ADD_TUTORIAL(hair_geometry)
ADD_TUTORIAL_ISPC(hair_geometry)
ADD_TUTORIAL_SYCL(hair_geometry) 

ENDIF()

ADD_EMBREE_TEST_ECS(hair_geometry embree_hair_geometry)