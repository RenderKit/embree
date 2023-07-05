## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_TRIANGLE)

INCLUDE(tutorial)
ADD_TUTORIAL(triangle_geometry)
ADD_TUTORIAL_ISPC(triangle_geometry)
ADD_TUTORIAL_SYCL(triangle_geometry)

ENDIF()

ADD_EMBREE_TEST_ECS(triangle_geometry embree_triangle_geometry ARGS --frames 128 --compare-threshold 40)  # because of AVX512 in fedora32