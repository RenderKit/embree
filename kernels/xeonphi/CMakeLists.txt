## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
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

#IF (TASKING_INTERNAL)
  ADD_DEFINITIONS(-DTASKING_LOCKSTEP)
#ELSE()
#  ADD_DEFINITIONS(-DTASKING_TBB)
#  INCLUDE_DIRECTORIES(${TBB_INCLUDE_DIRS_MIC})
#  SET(ADDITIONAL_LIBRARIES ${TBB_LIBRARIES_MIC})
#ENDIF()

INCLUDE(icc_xeonphi)

ADD_DEFINITIONS(-D__TARGET_XEON_PHI__)

ADD_ISPC_LIBRARY(embree_xeonphi ${EMBREE_LIB_TYPE}

  ../../common/tasking/taskscheduler_tbb.cpp # FIXME: remove
  ../../common/tasking/taskscheduler_mic.cpp
  ../../common/tasking/tasksys.cpp

  ../common/device.cpp
  ../common/stat.cpp 
  ../common/globals.cpp 
  ../common/acceln.cpp
  ../common/accelset.cpp
  ../common/state.cpp
  ../common/rtcore.cpp 
  ../common/rtcore_ispc.cpp 
  ../common/rtcore_ispc.ispc 
  ../common/buffer.cpp
  ../common/scene.cpp
  ../common/geometry.cpp
  ../common/scene_user_geometry.cpp
  ../common/scene_instance.cpp
  ../common/scene_triangle_mesh.cpp
  ../common/scene_bezier_curves.cpp
  ../common/scene_subdiv_mesh.cpp
  ../common/raystream_log.cpp
  ../common/subdiv/subdivpatch1base.cpp
  ../common/subdiv/tessellation_cache.cpp
  ../common/subdiv/catmullclark_coefficients.cpp

  ../algorithms/parallel_for.cpp
  ../algorithms/parallel_reduce.cpp
  ../algorithms/parallel_prefix_sum.cpp
  ../algorithms/parallel_for_for.cpp
  ../algorithms/parallel_for_for_prefix_sum.cpp
  ../algorithms/sort.cpp
  ../algorithms/pset.cpp
  ../algorithms/pmap.cpp
  ../algorithms/prefix.cpp

  geometry/triangle1.cpp
  geometry/instance_intersector1.cpp
  geometry/instance_intersector16.cpp

  builders/parallel_builder.cpp

  bvh4i/bvh4i.cpp
  bvh4i/bvh4i_builder.cpp
  bvh4i/bvh4i_builder_ext.cpp
  bvh4i/bvh4i_builder_morton.cpp
  bvh4i/bvh4i_builder_morton_64bit.cpp
  bvh4i/bvh4i_intersector16_chunk.cpp
  bvh4i/bvh4i_intersector16_single.cpp
  bvh4i/bvh4i_intersector16_hybrid.cpp
  bvh4i/bvh4i_intersector16_test.cpp
  bvh4i/bvh4i_intersector16_subdiv.cpp
  bvh4i/bvh4i_intersector1.cpp
  bvh4i/bvh4i_statistics.cpp
  bvh4i/bvh4i_rotate.cpp


  bvh4mb/bvh4mb.cpp
  bvh4mb/bvh4mb_builder.cpp
  bvh4mb/bvh4mb_intersector16_chunk.cpp
  bvh4mb/bvh4mb_intersector16_single.cpp  
  bvh4mb/bvh4mb_intersector16_hybrid.cpp
  bvh4mb/bvh4mb_intersector1.cpp

  bvh4hair/bvh4hair_builder.cpp
  bvh4hair/bvh4hair.cpp	
  bvh4hair/bvh4hair_intersector16_single.cpp
)

TARGET_LINK_LIBRARIES(embree_xeonphi sys_xeonphi simd_xeonphi lexers_xeonphi ${ADDITIONAL_LIBRARIES})

IF (ENABLE_INSTALLER)
  SET_TARGET_PROPERTIES(embree_xeonphi PROPERTIES VERSION ${EMBREE_VERSION} SOVERSION ${EMBREE_VERSION_MAJOR})
ELSE()
  SET_TARGET_PROPERTIES(embree_xeonphi PROPERTIES VERSION ${EMBREE_VERSION_MAJOR} SOVERSION ${EMBREE_VERSION_MAJOR})
ENDIF()

INSTALL(TARGETS embree_xeonphi LIBRARY DESTINATION lib COMPONENT lib_xeonphi NAMELINK_SKIP)
