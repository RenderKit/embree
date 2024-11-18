## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_SYCL_SUPPORT)
IF (EMBREE_GEOMETRY_TRIANGLE)

INCLUDE(tutorial)
ADD_TUTORIAL(debug_device_memory)
ADD_TUTORIAL_ISPC(debug_device_memory)
ADD_TUTORIAL_SYCL(debug_device_memory)

ENDIF()
ENDIF()