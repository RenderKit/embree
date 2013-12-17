## ======================================================================== ##
## Copyright 2009-2013 Intel Corporation                                    ##
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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../xeon)

INCLUDE(icc_xeonphi)

ADD_DEFINITIONS(-D__TARGET_XEON_PHI__)

ADD_ISPC_LIBRARY(embree_xeonphi SHARED

  ../common/stat.cpp 
  ../common/alloc.cpp 
  ../common/tasksys.cpp 
  ../common/accel3.cpp

  ../xeon/rtcore/rtcore.cpp 
  ../xeon/rtcore/rtcore_ispc.cpp 
  ../xeon/rtcore/rtcore_ispc.ispc 
  ../xeon/rtcore/scene.cpp
  ../xeon/rtcore/scene_user_geometry.cpp
  ../xeon/rtcore/scene_triangle_mesh.cpp
  ../xeon/rtcore/scene_quadratic_bezier_curves.cpp
  ../xeon/rtcore/geometry.cpp
  
  ../xeon/builders/heuristic_binning.cpp
  ../xeon/builders/heuristic_spatial.cpp
  ../xeon/builders/splitter.cpp
  ../xeon/builders/splitter_fallback.cpp
  ../xeon/builders/splitter_parallel.cpp
  ../xeon/builders/primrefgen.cpp
  
  ../xeon/geometry/instance_intersector1.cpp
  ../xeon/geometry/instance_intersector16.cpp
  
  ../xeon/geometry/triangle1.cpp
  ../xeon/geometry/triangle4.cpp
  ../xeon/geometry/triangle1v.cpp
  ../xeon/geometry/triangle4v.cpp
  ../xeon/geometry/triangle4i.cpp
  geometry/ispc_wrapper_knc.cpp

  ../xeon/bvh4i/bvh4i_builder_util.cpp

  bvh4i/bvh4i.cpp
  bvh4i/bvh4i_builder.cpp
  bvh4i/bvh4i_builder_ext.cpp
  bvh4i/bvh4i_builder_binner.cpp
  bvh4i/bvh4i_builder_morton.cpp
  bvh4i/bvh4i_builder_morton_enhanced.cpp
  bvh4i/bvh4i_intersector16_chunk.cpp
  bvh4i/bvh4i_intersector16_single.cpp
  bvh4i/bvh4i_intersector16_hybrid.cpp
  bvh4i/bvh4i_intersector16_virtual.cpp
  bvh4i/bvh4i_intersector1.cpp
  bvh4i/bvh4i_statistics.cpp

  ../xeon/bvh4mb/bvh4mb.cpp
  ../xeon/bvh4mb/bvh4mb_builder.cpp
  ../xeon/bvh4mb/bvh4mb_intersector1.cpp   
  ../xeon/bvh4mb/bvh4mb_intersector4.cpp
)

TARGET_LINK_LIBRARIES(embree_xeonphi sys_xeonphi simd_xeonphi)
SET_TARGET_PROPERTIES(embree_xeonphi PROPERTIES VERSION 2.1.0 SOVERSION 2)
INSTALL(TARGETS embree_xeonphi DESTINATION lib)
