## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(find_embree)
CMAKE_MINIMUM_REQUIRED(VERSION 3.2.0)

FIND_PACKAGE(embree 3.0 REQUIRED)

INCLUDE_DIRECTORIES(${EMBREE_INCLUDE_DIRS})
ADD_EXECUTABLE(find_embree find_embree.cpp)
TARGET_LINK_LIBRARIES(find_embree ${EMBREE_LIBRARY})

OPTION(EMBREE_ISPC_SUPPORT "Build Embree with support for ISPC applications." OFF)

# this configures the ADD_EMBREE_ISPC_EXECUTABLE from Embree
IF (EMBREE_ISPC_SUPPORT)
  SET(ISPC_TARGETS "sse2;sse4;avx;avx2")
  SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake ${CMAKE_MODULE_PATH})
  INCLUDE(ispc)

  INCLUDE_DIRECTORIES_ISPC(${EMBREE_INCLUDE_DIRS})
  ADD_EMBREE_ISPC_EXECUTABLE(find_embree_ispc find_embree_ispc.cpp find_embree_ispc.ispc)
  TARGET_LINK_LIBRARIES(find_embree_ispc ${EMBREE_LIBRARY})
ENDIF()
