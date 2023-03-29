## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_CURVE)

INCLUDE(tutorial)
ADD_TUTORIAL(curve_geometry)
ADD_TUTORIAL_ISPC(curve_geometry)
ADD_TUTORIAL_SYCL(curve_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(curve_geometry embree_curve_geometry)