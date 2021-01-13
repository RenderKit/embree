## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PROJECT(find_embree)
CMAKE_MINIMUM_REQUIRED(VERSION 3.2.0)

FIND_PACKAGE(embree 3.0 REQUIRED)

ADD_EXECUTABLE(find_embree find_embree.cpp)
TARGET_LINK_LIBRARIES(find_embree embree)

IF (EMBREE_ISPC_SUPPORT)

  OPTION(ENABLE_ISPC_SUPPORT "Build ISPC version of find_embree tutorial." OFF)

  # this configures the ADD_EMBREE_ISPC_EXECUTABLE from Embree
  IF (ENABLE_ISPC_SUPPORT)
    SET(ISPC_TARGETS "sse2;sse4;avx;avx2")
    SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake ${CMAKE_MODULE_PATH})
    INCLUDE(ispc)

    get_target_property(embree_include_dir embree INTERFACE_INCLUDE_DIRECTORIES)
    INCLUDE_DIRECTORIES_ISPC(${embree_include_dir})
    ADD_EMBREE_ISPC_EXECUTABLE(find_embree_ispc find_embree_ispc.cpp find_embree_ispc.ispc)
    TARGET_LINK_LIBRARIES(find_embree_ispc embree)
  ENDIF()
ENDIF()
