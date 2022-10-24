## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_TRIANGLE)

INCLUDE(tutorial)
ADD_TUTORIAL(triangle_geometry)
ADD_TUTORIAL_ISPC(triangle_geometry)
ADD_TUTORIAL_SYCL(triangle_geometry)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST2(triangle_geometry triangle_geometry "--frames 128 --compare-threshold 40")  # because of AVX512 in fedora32
ENDIF()

ENDIF()
