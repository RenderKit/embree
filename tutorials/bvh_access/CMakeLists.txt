## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (EMBREE_GEOMETRY_CURVE)
  
ADD_EXECUTABLE(embree_bvh_access ../../kernels/embree.rc bvh_access.cpp)
TARGET_LINK_LIBRARIES(embree_bvh_access embree math sys tasking tutorial)

SET_PROPERTY(TARGET embree_bvh_access PROPERTY FOLDER tutorials/single)
SET_PROPERTY(TARGET embree_bvh_access APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")
INSTALL(TARGETS embree_bvh_access DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT examples)
SIGN_TARGET(embree_bvh_access)

IF (BUILD_TESTING AND EMBREE_TESTING_INTENSITY GREATER 0)
  ADD_TEST(NAME bvh_access COMMAND embree_bvh_access)
ENDIF()

ENDIF()
