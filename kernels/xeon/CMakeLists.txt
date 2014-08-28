## ======================================================================== ##
## Copyright 2009-2014 Intel Corporation                                    ##
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

IF (TARGET_SSE41)
 ADD_DEFINITIONS(-D__TARGET_SSE41__)
ENDIF()

IF (TARGET_SSE42)
 ADD_DEFINITIONS(-D__TARGET_SSE42__)
ENDIF()

IF (TARGET_AVX)
 ADD_DEFINITIONS(-D__TARGET_AVX__)
ENDIF()

IF (TARGET_AVX2)
 ADD_DEFINITIONS(-D__TARGET_AVX2__)
ENDIF()

ADD_ISPC_LIBRARY(embree SHARED   

  ../common/stat.cpp 
  ../common/globals.cpp 
  ../common/alloc.cpp 
  ../common/tasksys.cpp 
  ../common/acceln.cpp
  ../common/rtcore.cpp 
  ../common/rtcore_ispc.cpp 
  ../common/rtcore_ispc.ispc 
  ../common/buffer.cpp
  ../common/scene.cpp
  ../common/geometry.cpp
  ../common/scene_user_geometry.cpp
  ../common/scene_triangle_mesh.cpp
  ../common/scene_bezier_curves.cpp
  
  builders/bezierrefgen.cpp 
  builders/primrefgen.cpp
  builders/heuristic_object_partition_unaligned.cpp
  builders/heuristic_object_partition.cpp
  builders/heuristic_spatial_split.cpp
  builders/heuristic_strand_partition.cpp
  builders/heuristic_fallback.cpp
    
  geometry/bezier1.cpp
  geometry/bezier1i.cpp
  geometry/triangle1.cpp
  geometry/triangle4.cpp
  geometry/triangle1v.cpp
  geometry/triangle4v.cpp
  geometry/triangle4i.cpp
  geometry/instance_intersector1.cpp
  geometry/instance_intersector4.cpp
  
  bvh4/bvh4.cpp
  bvh4/bvh4_rotate.cpp
  bvh4/bvh4_refit.cpp
  bvh4/bvh4_builder.cpp
  bvh4/bvh4_builder_mb.cpp
  bvh4/bvh4_builder_fast.cpp
  bvh4/bvh4_builder_morton.cpp
  bvh4/bvh4_builder_binner.cpp
  bvh4/bvh4_builder_toplevel.cpp
  bvh4/bvh4_intersector1.cpp   
  bvh4/bvh4_intersector4_chunk.cpp
  bvh4/bvh4_statistics.cpp

  bvh4hair/bvh4hair.cpp
  bvh4hair/bvh4hair_statistics.cpp
  bvh4hair/bvh4hair_builder.cpp 
  bvh4hair/bvh4hair_builder_mb.cpp 
  bvh4hair/bvh4hair_intersector1.cpp
  bvh4hair/bvh4hair_intersector4.cpp
)

IF (TARGET_SSE41) 
  ADD_LIBRARY(embree_sse41 STATIC
    bvh4/bvh4_builder_binner.cpp
    bvh4/bvh4_builder_toplevel.cpp
    bvh4/bvh4_intersector1.cpp   
    bvh4/bvh4_intersector4_chunk.cpp
  )
  SET_TARGET_PROPERTIES(embree_sse41 PROPERTIES COMPILE_FLAGS "${FLAGS_SSE41}")
  SET(EMBREE_LIBRARIES ${EMBREE_LIBRARIES} embree_sse41)
ENDIF ()

IF (TARGET_SSE42) 
  ADD_LIBRARY(embree_sse42 STATIC
    bvh4/bvh4_intersector4_hybrid.cpp
  )
  SET_TARGET_PROPERTIES(embree_sse42 PROPERTIES COMPILE_FLAGS "${FLAGS_SSE42}")
  SET(EMBREE_LIBRARIES ${EMBREE_LIBRARIES} embree_sse42)
ENDIF ()

IF (TARGET_AVX) 
  ADD_LIBRARY(embree_avx STATIC

  builders/bezierrefgen.cpp 
  builders/primrefgen.cpp
  builders/heuristic_object_partition_unaligned.cpp
  builders/heuristic_object_partition.cpp
  builders/heuristic_spatial_split.cpp
  builders/heuristic_strand_partition.cpp
  builders/heuristic_fallback.cpp

   geometry/triangle8.cpp

   geometry/instance_intersector1.cpp
   geometry/instance_intersector4.cpp
   geometry/instance_intersector8.cpp

   bvh4/bvh4_builder_morton.cpp
   bvh4/bvh4_builder_fast.cpp
   bvh4/bvh4_builder.cpp
   bvh4/bvh4_builder_mb.cpp
   bvh4/bvh4_refit.cpp

   bvh4/bvh4_intersector1.cpp   
   bvh4/bvh4_intersector4_chunk.cpp
   bvh4/bvh4_intersector4_hybrid.cpp
   bvh4/bvh4_intersector8_chunk.cpp
   bvh4/bvh4_intersector8_hybrid.cpp

   bvh4hair/bvh4hair_builder.cpp
   bvh4hair/bvh4hair_builder_mb.cpp
   bvh4hair/bvh4hair_intersector1.cpp
   bvh4hair/bvh4hair_intersector4.cpp
   bvh4hair/bvh4hair_intersector8.cpp

   bvh8/bvh8.cpp
   bvh8/bvh8_builder.cpp
   bvh8/bvh8_statistics.cpp
   bvh8/bvh8_intersector1.cpp  
   bvh8/bvh8_intersector4_hybrid.cpp   
   bvh8/bvh8_intersector8_chunk.cpp   
   bvh8/bvh8_intersector8_hybrid.cpp   
)

  SET_TARGET_PROPERTIES(embree_avx PROPERTIES COMPILE_FLAGS "${FLAGS_AVX}")
  SET(EMBREE_LIBRARIES ${EMBREE_LIBRARIES} embree_avx)
 ENDIF()

IF (TARGET_AVX2) 
  ADD_LIBRARY(embree_avx2 STATIC

    geometry/instance_intersector1.cpp
    geometry/instance_intersector4.cpp
    geometry/instance_intersector8.cpp
		
    bvh4/bvh4_intersector1.cpp
    bvh4/bvh4_intersector4_chunk.cpp
    bvh4/bvh4_intersector4_hybrid.cpp
    bvh4/bvh4_intersector8_chunk.cpp
    bvh4/bvh4_intersector8_hybrid.cpp

    bvh4hair/bvh4hair_intersector1.cpp
    bvh4hair/bvh4hair_intersector4.cpp
    bvh4hair/bvh4hair_intersector8.cpp

    bvh8/bvh8_intersector1.cpp   
    bvh8/bvh8_intersector4_hybrid.cpp 
    bvh8/bvh8_intersector8_chunk.cpp   
    bvh8/bvh8_intersector8_hybrid.cpp   
)
  SET_TARGET_PROPERTIES(embree_avx2 PROPERTIES COMPILE_FLAGS "${FLAGS_AVX2}")
  SET(EMBREE_LIBRARIES ${EMBREE_LIBRARIES} embree_avx2)
ENDIF()

TARGET_LINK_LIBRARIES(embree ${EMBREE_LIBRARIES} sys simd)
SET_TARGET_PROPERTIES(embree PROPERTIES VERSION 2.3.2 SOVERSION 2)
INSTALL(TARGETS embree DESTINATION lib)
