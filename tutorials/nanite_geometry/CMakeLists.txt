## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_INSTANCE)

INCLUDE(tutorial)
ADD_TUTORIAL(nanite_geometry)
ADD_TUTORIAL_SYCL(nanite_geometry)

ENDIF()
