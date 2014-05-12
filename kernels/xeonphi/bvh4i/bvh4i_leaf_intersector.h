// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "bvh4i.h"
#include "geometry/triangle1.h"
#include "geometry/triangle1_intersector1_moeller.h"
#include "geometry/triangle1mc_intersector1_moeller.h"
#include "geometry/triangle1_intersector16_moeller.h"
#include "geometry/triangle1mc_intersector16_moeller.h"
#include "geometry/filter.h"

namespace embree
{

    // ============================================================================================
    // ============================================================================================
    // ============================================================================================
    static __aligned(64) int zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};

    struct Triangle1LeafIntersector
    {
      // === single ray === 
      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray& ray, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry)
      {
	const Triangle1* __restrict__ const tptr  = (Triangle1*) curNode.leaf(accel);	      
	const mic_i and_mask = broadcast4to16i(zlc4);
	return Triangle1Intersector1MoellerTrumbore::intersect1(dir_xyz,
								 org_xyz,
								 min_dist_xyz,
								 max_dist_xyz,
								 and_mask,
								 ray,
								 geometry,
								 tptr);	
      }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 const Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
      {
	const Triangle1* __restrict__ const tptr  = (Triangle1*) curNode.leaf(accel);	      
	const mic_i and_mask = broadcast4to16i(zlc4);
	return Triangle1Intersector1MoellerTrumbore::occluded1(dir_xyz,
							       org_xyz,
							       min_dist_xyz,
							       max_dist_xyz,
							       and_mask,
							       ray,
							       geometry,
							       tptr);	
      }

      // ==== single ray in packet ===

      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const size_t rayIndex, 
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray16& ray16, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry)
      {
	const Triangle1* __restrict__ const tptr  = (Triangle1*) curNode.leaf(accel);	      
	const mic_i and_mask = broadcast4to16i(zlc4);
	return Triangle1Intersector16MoellerTrumbore::intersect1(rayIndex,
								 dir_xyz,
								 org_xyz,
								 min_dist_xyz,
								 max_dist_xyz,
								 and_mask,
								 ray16,
								 geometry,
								 tptr);	
      }



      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const size_t rayIndex, 
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 const Ray16& ray16, 
					 mic_m &m_terminated,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
      {
	const Triangle1* __restrict__ const tptr  = (Triangle1*) curNode.leaf(accel);	      
	const mic_i and_mask = broadcast4to16i(zlc4);
	return Triangle1Intersector16MoellerTrumbore::occluded1(rayIndex,
								dir_xyz,
								org_xyz,
								min_dist_xyz,
								max_dist_xyz,
								and_mask,
								ray16,
								m_terminated,
								geometry,
								tptr);	
      }

    };


    // ============================================================================================
    // ============================================================================================
    // ============================================================================================

    struct Triangle1mcLeafIntersector
    {

      // === single ray === 
      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray& ray, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const Triangle1mc *__restrict__ const tptr = (Triangle1mc*)accel + index;
	const mic_i and_mask = broadcast4to16i(zlc4);

	return Triangle1mcIntersector1MoellerTrumbore::intersect1(dir_xyz,
								 org_xyz,
								 min_dist_xyz,
								 max_dist_xyz,
								 and_mask,
								 ray,
								 geometry,
								 tptr);	
      }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 const Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const Triangle1mc *__restrict__ const tptr = (Triangle1mc*)accel + index;
	const mic_i and_mask = broadcast4to16i(zlc4);

	return Triangle1mcIntersector1MoellerTrumbore::occluded1(dir_xyz,
								 org_xyz,
								 min_dist_xyz,
								 max_dist_xyz,
								 and_mask,
								 ray,
								 geometry,
								 tptr);	
      }

      // ==== single ray in packet ===

      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const size_t rayIndex, 
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray16& ray16, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const Triangle1mc *__restrict__ const tptr = (Triangle1mc*)accel + index;

	const mic_i and_mask = broadcast4to16i(zlc4);
	return Triangle1mcIntersector16MoellerTrumbore::intersect1(rayIndex,
								   dir_xyz,
								   org_xyz,
								   min_dist_xyz,
								   max_dist_xyz,
								   and_mask,
								   ray16,
								   geometry,
								   tptr);	
      }


      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const size_t rayIndex, 
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 const Ray16& ray16, 
					 mic_m &m_terminated,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const Triangle1mc *__restrict__ const tptr = (Triangle1mc*)accel + index;

	const mic_i and_mask = broadcast4to16i(zlc4);
	return Triangle1mcIntersector16MoellerTrumbore::occluded1(rayIndex,
								  dir_xyz,
								  org_xyz,
								  min_dist_xyz,
								  max_dist_xyz,
								  and_mask,
								  ray16,
								  m_terminated,
								  geometry,
								  tptr);	
      }

    };

};
