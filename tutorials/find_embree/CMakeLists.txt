## ======================================================================== ##
## Copyright 2009-2018 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

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
