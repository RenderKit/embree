## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_USER)
IF (NOT EMBREE_DPCPP_IMPLICIT_DISPATCH_GLOBALS)  # FIXME: bug workaround
    
INCLUDE(tutorial)
ADD_TUTORIAL(user_geometry)
ADD_TUTORIAL_ISPC(user_geometry)
ADD_TUTORIAL_SYCL(user_geometry)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_EMBREE_TEST(user_geometry)
  IF (EMBREE_RAY_PACKETS AND EMBREE_GEOMETRY_USER_IN_GEOMETRY AND EMBREE_FILTER_FUNCTION_IN_GEOMETRY)
    ADD_EMBREE_TEST2(user_geometry_stream_coherent   user_geometry "--mode stream --coherent")
    ADD_EMBREE_TEST2(user_geometry_stream_incoherent user_geometry "--mode stream --incoherent")
  ENDIF()
ENDIF()

ENDIF()
ENDIF()
