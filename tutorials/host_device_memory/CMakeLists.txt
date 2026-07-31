## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_TRIANGLE)

INCLUDE(tutorial)
ADD_TUTORIAL(host_device_memory)
ADD_TUTORIAL_SYCL(host_device_memory)

ADD_EMBREE_TEST_ECS(host_device_memory embree_host_device_memory ARGS NO_ISPC)

ENDIF()