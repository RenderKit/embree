## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_FILTER_FUNCTION)

INCLUDE(tutorial)
ADD_TUTORIAL(intersection_filter)
ADD_TUTORIAL_ISPC(intersection_filter)
ADD_TUTORIAL_SYCL(intersection_filter)

ENDIF()

ADD_EMBREE_TEST_ECS(intersection_filter embree_intersection_filter)