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

#include "bvh4mb.h"

namespace embree
{

    struct Triangle1mbLeafIntersector
    {
  
      // ========================
      // ==== 16-wide packets ===
      // ========================

      __forceinline static void intersect16(BVH4i::NodeRef curNode,
					    const mic_m m_valid_leaf, 
					    const mic3f &dir,
					    const mic3f &org,
					    Ray16& ray16, 
					    const void *__restrict__ const accel,
					    const Scene     *__restrict__ const geometry)
      {

	const mic_f time     = ray16.time;
	const mic_f one_time = (mic_f::one() - time);

	unsigned int items; 
	const BVH4mb::Triangle01* tris  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel,items);

	const mic_f zero = mic_f::zero();
	const mic_f one  = mic_f::one();

	prefetch<PFHINT_L1>((mic_f*)tris +  0); 
	prefetch<PFHINT_L2>((mic_f*)tris +  1); 
	prefetch<PFHINT_L2>((mic_f*)tris +  2); 
	prefetch<PFHINT_L2>((mic_f*)tris +  3); 
	prefetch<PFHINT_L2>((mic_f*)tris +  4); 
	prefetch<PFHINT_L2>((mic_f*)tris +  5); 
	prefetch<PFHINT_L2>((mic_f*)tris +  6); 
	prefetch<PFHINT_L2>((mic_f*)tris +  7); 

	for (size_t i=0; i<items; i++) 
	  {
	    const Triangle1& tri_t0 = tris[i].t0;
	    const Triangle1& tri_t1 = tris[i].t1;

	    prefetch<PFHINT_L1>(&tris[i+1].t0); 
	    prefetch<PFHINT_L1>(&tris[i+1].t1); 

	    STAT3(normal.trav_prims,1,popcnt(valid_i),16);
        
	    /* load vertices and calculate edges */
	    const mic3f v0_t0( broadcast1to16f(&tri_t0.v0.x), broadcast1to16f(&tri_t0.v0.y), broadcast1to16f(&tri_t0.v0.z) );
	    const mic3f v0_t1( broadcast1to16f(&tri_t1.v0.x), broadcast1to16f(&tri_t1.v0.y), broadcast1to16f(&tri_t1.v0.z) );
	    const mic3f v0 = v0_t0 * one_time + time * v0_t1;
	    const mic3f v1_t0( broadcast1to16f(&tri_t0.v1.x), broadcast1to16f(&tri_t0.v1.y), broadcast1to16f(&tri_t0.v1.z) );
	    const mic3f v1_t1( broadcast1to16f(&tri_t1.v1.x), broadcast1to16f(&tri_t1.v1.y), broadcast1to16f(&tri_t1.v1.z) );
	    const mic3f v1 = v1_t0 * one_time + time * v1_t1;
	    const mic3f v2_t0( broadcast1to16f(&tri_t0.v2.x), broadcast1to16f(&tri_t0.v2.y), broadcast1to16f(&tri_t0.v2.z) );
	    const mic3f v2_t1( broadcast1to16f(&tri_t1.v2.x), broadcast1to16f(&tri_t1.v2.y), broadcast1to16f(&tri_t1.v2.z) );
	    const mic3f v2 = v2_t0 * one_time + time * v2_t1;

	    const mic3f e1 = v0-v1;
	    const mic3f e2 = v2-v0;

	    const mic3f Ng = cross(e1,e2);

	    /* calculate denominator */
	    const mic3f C =  v0 - org;
	    
	    const mic_f den = dot(Ng,dir);

	    mic_m valid = m_valid_leaf;

#if defined(__BACKFACE_CULLING__)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const mic_f rcp_den = rcp(den);
	    const mic3f R = cross(dir,C);
	    const mic_f u = dot(R,e2)*rcp_den;
	    const mic_f v = dot(R,e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);
	    prefetch<PFHINT_L1EX>(&ray16.u);      
	    prefetch<PFHINT_L1EX>(&ray16.v);      
	    prefetch<PFHINT_L1EX>(&ray16.tfar);      
	    const mic_f t = dot(C,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    const mic_i geomID = tri_t0.geomID();
	    const mic_i primID = tri_t0.primID();
	    prefetch<PFHINT_L1EX>(&ray16.geomID);      
	    prefetch<PFHINT_L1EX>(&ray16.primID);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.x);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.y);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.z);      

	    /* ray masking test */
#if defined(__USE_RAY_MASK__)
	    valid &= (mic_i(tri_t0.mask()) & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;
        
	    /* update hit information */
	    store16f(valid,(float*)&ray16.u,u);
	    store16f(valid,(float*)&ray16.v,v);
	    store16f(valid,(float*)&ray16.tfar,t);
	    store16i(valid,(float*)&ray16.geomID,geomID);
	    store16i(valid,(float*)&ray16.primID,primID);
	    store16f(valid,(float*)&ray16.Ng.x,Ng.x);
	    store16f(valid,(float*)&ray16.Ng.y,Ng.y);
	    store16f(valid,(float*)&ray16.Ng.z,Ng.z);
	  }
      }

      __forceinline static void occluded16(BVH4i::NodeRef curNode,
					   const mic_m m_valid_leaf_active, 
					   const mic3f &dir,
					   const mic3f &org,
					   Ray16& ray16, 
					   mic_m &m_terminated,					    
					   const void *__restrict__ const accel,
					   const Scene     *__restrict__ const geometry)
      {
	mic_m m_valid_leaf = m_valid_leaf_active;

	const mic_f time     = ray16.time;
	const mic_f one_time = (mic_f::one() - time);

	unsigned int items; 
	const BVH4mb::Triangle01* tris  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel,items);

	prefetch<PFHINT_L1>((mic_f*)tris +  0); 
	prefetch<PFHINT_L2>((mic_f*)tris +  1); 
	prefetch<PFHINT_L2>((mic_f*)tris +  2); 
	prefetch<PFHINT_L2>((mic_f*)tris +  3); 
	prefetch<PFHINT_L2>((mic_f*)tris +  4); 
	prefetch<PFHINT_L2>((mic_f*)tris +  5); 
	prefetch<PFHINT_L2>((mic_f*)tris +  6); 
	prefetch<PFHINT_L2>((mic_f*)tris +  7); 

	const mic_f zero = mic_f::zero();

	for (size_t i=0; i<items; i++) 
	  {
	    const Triangle1& tri_t0 = tris[i].t0;
	    const Triangle1& tri_t1 = tris[i].t1;

	    prefetch<PFHINT_L1>(&tris[i+1].t0); 
	    prefetch<PFHINT_L1>(&tris[i+1].t1); 

	    STAT3(normal.trav_prims,1,popcnt(valid_i),16);
        
	    /* load vertices and calculate edges */
	    const mic3f v0_t0( broadcast1to16f(&tri_t0.v0.x), broadcast1to16f(&tri_t0.v0.y), broadcast1to16f(&tri_t0.v0.z) );
	    const mic3f v0_t1( broadcast1to16f(&tri_t1.v0.x), broadcast1to16f(&tri_t1.v0.y), broadcast1to16f(&tri_t1.v0.z) );
	    const mic3f v0 = v0_t0 * one_time + time * v0_t1;
	    const mic3f v1_t0( broadcast1to16f(&tri_t0.v1.x), broadcast1to16f(&tri_t0.v1.y), broadcast1to16f(&tri_t0.v1.z) );
	    const mic3f v1_t1( broadcast1to16f(&tri_t1.v1.x), broadcast1to16f(&tri_t1.v1.y), broadcast1to16f(&tri_t1.v1.z) );
	    const mic3f v1 = v1_t0 * one_time + time * v1_t1;
	    const mic3f v2_t0( broadcast1to16f(&tri_t0.v2.x), broadcast1to16f(&tri_t0.v2.y), broadcast1to16f(&tri_t0.v2.z) );
	    const mic3f v2_t1( broadcast1to16f(&tri_t1.v2.x), broadcast1to16f(&tri_t1.v2.y), broadcast1to16f(&tri_t1.v2.z) );
	    const mic3f v2 = v2_t0 * one_time + time * v2_t1;

	    const mic3f e1 = v0-v1;
	    const mic3f e2 = v2-v0;

	    const mic3f Ng = cross(e1,e2);

	    /* calculate denominator */
	    const mic3f C =  v0 - org;
	    
	    const mic_f den = dot(Ng,dir);

	    mic_m valid = m_valid_leaf;

#if defined(__BACKFACE_CULLING__)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const mic_f rcp_den = rcp(den);
	    const mic3f R = cross(dir,C);
	    const mic_f u = dot(R,e2)*rcp_den;
	    const mic_f v = dot(R,e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);
	    const mic_f t = dot(C,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    /* ray masking test */
#if defined(__USE_RAY_MASK__)
	    valid &= (mic_i(tri_t0.mask()) & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;
	    
	    /* update occlusion */
	    m_terminated |= valid;
	    m_valid_leaf &= ~valid;
	    if (unlikely(none(m_valid_leaf))) break;
	  }

      }

    };

};
