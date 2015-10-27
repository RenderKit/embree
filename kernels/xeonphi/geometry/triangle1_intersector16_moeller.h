// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "triangle1.h"
#include "../../common/ray.h"
#include "filter.h"

#define COMPUTE_NORMAL
//#define INTERSECT_TRIANGLE_PAIRS

namespace embree
{
  /*! Intersector for individual precomputed triangles with 16
   *  rays. This intersector implements a modified version of the
   *  Moeller Trumbore intersector from the paper "Fast, Minimum
   *  Storage Ray-Triangle Intersection". In contrast to the paper we
   *  precalculate some factors and factor the calculations
   *  differently to allow precalculating the cross product e1 x
   *  e2. */

  template< bool ENABLE_INTERSECTION_FILTER>
    struct Triangle1Intersector16MoellerTrumbore
    {
      typedef Triangle1 Primitive;


      __forceinline static bool intersect1(const BVH4i::NodeRef curNode,
					   const size_t rayIndex, 
					   const vfloat16 &dir_xyz,
					   const vfloat16 &org_xyz,
					   const vfloat16 &min_dist_xyz,
					   vfloat16 &max_dist_xyz,
					   const vint16 &and_mask,
					   Ray16& ray16, 
					   const Scene     *__restrict__ const geometry,
					   const Triangle1 * __restrict__ const tptr)
      {
	//	__aligned(64) float norm[16];

	const vfloat16 zero = vfloat16::zero();
	vfloat16 v0,v1,v2;
#if defined(INTERSECT_TRIANGLE_PAIRS)

	prefetch<PFHINT_L1>(tptr + 1);
	prefetch<PFHINT_L1>(tptr + 0); 

	if (likely(curNode.isAuxFlagSet()))
	  {
	    const TrianglePair1 * __restrict__ const pptr = (TrianglePair1*)tptr;
	    const vfloat16 _v0 = gather_2f_zlc(and_mask,0xff00,
					    (float*)&pptr[0].v0,
					    (float*)&pptr[1].v0);

	    const vfloat16 _v1 = gather_2f_zlc(and_mask,0xff00,
					    (float*)&pptr[0].v1,
					    (float*)&pptr[1].v1);

#if 1
	     const vfloat16 p0 = load16f(&pptr[0]); 
	     const vfloat16 p1 = load16f(&pptr[1]); 
             const vfloat16 _v2 = asFloat(asInt(select(0x00ff,align_shift_right<8>(p0,p0),p1)) & and_mask);
#else
	    const vfloat16 p0 = broadcast8to16f(&pptr[0].v2);
	    const vfloat16 p1 = broadcast8to16f(&pptr[1].v2);
            const vfloat16 _v2 = asFloat(asInt(select(0xff00,p1,p0)) & and_mask);
#endif

	    v0 = _v0;
	    v1 = _v1;
	    v2 = _v2;

	  }
	else
#endif
	  {
	    prefetch<PFHINT_L1>(tptr + 3);
	    prefetch<PFHINT_L1>(tptr + 2);
	    prefetch<PFHINT_L1>(tptr + 1); 
	    prefetch<PFHINT_L1>(tptr + 0);  
	    const vfloat16 _v0 = gather_4f_zlc(and_mask,
					   (float*)&tptr[0].v0,
					   (float*)&tptr[1].v0,
					   (float*)&tptr[2].v0,
					   (float*)&tptr[3].v0);
	      
	    const vfloat16 _v1 = gather_4f_zlc(and_mask,
					   (float*)&tptr[0].v1,
					   (float*)&tptr[1].v1,
					   (float*)&tptr[2].v1,
					   (float*)&tptr[3].v1);
	      
	    const vfloat16 _v2 = gather_4f_zlc(and_mask,
					   (float*)&tptr[0].v2,
					   (float*)&tptr[1].v2,
					   (float*)&tptr[2].v2,
					   (float*)&tptr[3].v2);
	    v0 = _v0;
	    v1 = _v1;
	    v2 = _v2;
	  }

	const vfloat16 e1 = v1 - v0;
	const vfloat16 e2 = v0 - v2;	     
	const vfloat16 normal = lcross_zxy(e1,e2);
	const vfloat16 org = v0 - org_xyz;
	const vfloat16 odzxy = msubr231(org * shuffle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, shuffle(org,_MM_SWIZ_REG_DACB));
	const vfloat16 den = ldot3_zxy(dir_xyz,normal);	      
	const vfloat16 rcp_den = rcp(den);
	const vfloat16 uu = ldot3_zxy(e2,odzxy); 
	const vfloat16 vv = ldot3_zxy(e1,odzxy); 
	const vfloat16 u = uu * rcp_den;
	const vfloat16 v = vv * rcp_den;

#if defined(RTCORE_BACKFACE_CULLING)
	const vbool16 m_init = (vbool16)0x1111 & (den > zero);
#else
	const vbool16 m_init = 0x1111;
#endif

	const vbool16 valid_u = ge(m_init,u,zero);
	const vbool16 valid_v = ge(valid_u,v,zero);
	const vbool16 m_aperture = le(valid_v,u+v,vfloat16::one()); 

	const vfloat16 nom = ldot3_zxy(org,normal);

	if (unlikely(none(m_aperture))) return false;
	const vfloat16 t = rcp_den*nom;
	//store16f(norm,normal);
	vbool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);


#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray16.mask[rayIndex]);
	const vint16 triMask = getTriMasks(tptr); 
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif

	//////////////////////////////////////////////////////////////////////////////////////////////////

	/* did the ray hit one of the four triangles? */
	if (unlikely(any(m_final)))
	  {
	    STAT3(normal.trav_prim_hits,1,1,1);

	    /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		vfloat16 org_max_dist_xyz = max_dist_xyz;

		/* did the ray hit one of the four triangles? */
		while (any(m_final)) 
		  {
		    max_dist_xyz  = select(m_final,t,org_max_dist_xyz);
		    const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		    const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		    const size_t vecIndex = bitscan(toInt(m_dist));
		    const size_t triIndex = vecIndex >> 2;
		    const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		    const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

#ifndef COMPUTE_NORMAL
		    const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		    const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		    const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		    const vfloat16 gnormalz = vfloat16(normal[vecIndex+0]);
		    const vfloat16 gnormalx = vfloat16(normal[vecIndex+1]);
		    const vfloat16 gnormaly = vfloat16(normal[vecIndex+2]);
#endif                

		    const int geomID = tri_ptr->geomID();
		    const int primID = tri_ptr->primID();
                
		    const Geometry* const geom = geometry->get(geomID);
		    if (likely(!geom->hasIntersectionFilter<vfloat16>())) 
		      {
			ray16.update(m_tri,rayIndex,min_dist,u,v,gnormalx,gnormaly,gnormalz,geomID,primID);
			break;
		      }
                
		    if (runIntersectionFilter16(geom,ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
		      break;
		    }
		    m_final ^= m_tri;
		  }
		max_dist_xyz = ray16.tfar[rayIndex];
	      }
	    else
	      {
		ray16.prefetchHitData<PFHINT_L1EX>();

		max_dist_xyz  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;

		const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;

		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

#ifndef COMPUTE_NORMAL
		const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		const vfloat16 gnormalz = vfloat16(normal[vecIndex+0]);
		const vfloat16 gnormalx = vfloat16(normal[vecIndex+1]);
		const vfloat16 gnormaly = vfloat16(normal[vecIndex+2]);
#endif                
		max_dist_xyz = min_dist;

	
		ray16.update(m_tri,rayIndex,min_dist,u,v,gnormalx,gnormaly,gnormalz,tri_ptr->geomID(),tri_ptr->primID());
	      }
	    return true;
      
	  }
	return false;
      }


      __forceinline static bool occluded1(const BVH4i::NodeRef curNode,					   
					  const size_t rayIndex, 
					  const vfloat16 &dir_xyz,
					  const vfloat16 &org_xyz,
					  const vfloat16 &min_dist_xyz,
					  const vfloat16 &max_dist_xyz,
					  const vint16 &and_mask,
					  const Ray16& ray16, 
					  vbool16 &m_terminated,
					  const Scene     *__restrict__ const geometry,
					  const Triangle1 * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();

	vfloat16 v0,v1,v2;
#if defined(INTERSECT_TRIANGLE_PAIRS)
	prefetch<PFHINT_L1>(tptr + 1);
	prefetch<PFHINT_L1>(tptr + 0); 

	if (likely(curNode.isAuxFlagSet()))
	  {
	    const TrianglePair1 * __restrict__ const pptr = (TrianglePair1*)tptr;
	    const vfloat16 _v0 = gather_2f_zlc(and_mask,0xff00,
					    (float*)&pptr[0].v0,
					    (float*)&pptr[1].v0);

	    const vfloat16 _v1 = gather_2f_zlc(and_mask,0xff00,
					    (float*)&pptr[0].v1,
					    (float*)&pptr[1].v1);

	    const vfloat16 p0 = load16f(&pptr[0]);
	    const vfloat16 p1 = load16f(&pptr[1]);
            const vfloat16 _v2 = asFloat(asInt(select(0x00ff,align_shift_right<8>(p0,p0),p1)) & and_mask);

	    v0 = _v0;
	    v1 = _v1;
	    v2 = _v2;
	  }
	else
#endif
	  {
	    prefetch<PFHINT_L1>(tptr + 3);
	    prefetch<PFHINT_L1>(tptr + 2);
	    prefetch<PFHINT_L1>(tptr + 1);
	    prefetch<PFHINT_L1>(tptr + 0); 
	      
	    const vfloat16 _v0 = gather_4f_zlc(and_mask,
					   (float*)&tptr[0].v0,
					   (float*)&tptr[1].v0,
					   (float*)&tptr[2].v0,
					   (float*)&tptr[3].v0);
	      
	    const vfloat16 _v1 = gather_4f_zlc(and_mask,
					   (float*)&tptr[0].v1,
					   (float*)&tptr[1].v1,
					   (float*)&tptr[2].v1,
					   (float*)&tptr[3].v1);
	      
	    const vfloat16 _v2 = gather_4f_zlc(and_mask,
					   (float*)&tptr[0].v2,
					   (float*)&tptr[1].v2,
					   (float*)&tptr[2].v2,
					   (float*)&tptr[3].v2);
	    v0 = _v0;
	    v1 = _v1;
	    v2 = _v2;
	  }
	const vfloat16 e1 = v1 - v0;
	const vfloat16 e2 = v0 - v2;	     
	const vfloat16 normal = lcross_zxy(e1,e2);

	const vfloat16 org = v0 - org_xyz;
	const vfloat16 odzxy = msubr231(org * shuffle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, shuffle(org,_MM_SWIZ_REG_DACB));
	const vfloat16 den = ldot3_zxy(dir_xyz,normal);	      
	const vfloat16 rcp_den = rcp(den);
	const vfloat16 uu = ldot3_zxy(e2,odzxy); 
	const vfloat16 vv = ldot3_zxy(e1,odzxy); 
	const vfloat16 u = uu * rcp_den;
	const vfloat16 v = vv * rcp_den;

#if defined(RTCORE_BACKFACE_CULLING)
	const vbool16 m_init = (vbool16)0x1111 & (den > zero);
#else
	const vbool16 m_init = 0x1111;
#endif

	const vbool16 valid_u = ge((vbool16)m_init,u,zero);
	const vbool16 valid_v = ge(valid_u,v,zero);
	const vbool16 m_aperture = le(valid_v,u+v,vfloat16::one()); 

	const vfloat16 nom = ldot3_zxy(org,normal);
	const vfloat16 t = rcp_den*nom;
	if (unlikely(none(m_aperture))) return false;

	vbool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray16.mask[rayIndex]);
	const vint16 triMask = getTriMasks(tptr); 
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif

	if (ENABLE_INTERSECTION_FILTER) 
	  {
              
	    /* did the ray hit one of the four triangles? */
	    while (any(m_final)) 
	      {
		const vfloat16 temp_t  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(temp_t);
		const vbool16 m_dist = eq(min_dist,temp_t);
		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;
		const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));
#ifndef COMPUTE_NORMAL
		const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		const vfloat16 gnormalz = vfloat16(normal[vecIndex+0]);
		const vfloat16 gnormalx = vfloat16(normal[vecIndex+1]);
		const vfloat16 gnormaly = vfloat16(normal[vecIndex+2]);
#endif                

		const int geomID = tri_ptr->geomID();
		const int primID = tri_ptr->primID();                
		const Geometry* const geom = geometry->get(geomID);
		if (likely(!geom->hasOcclusionFilter<vfloat16>())) break;
                
		if (runOcclusionFilter16(geom,(Ray16&)ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) 
		  break;

		m_final ^= m_tri; /* clear bit */
	      }
      
	  }
	if (unlikely(any(m_final)))
	  {
	    STAT3(shadow.trav_prim_hits,1,1,1);
	    m_terminated |= vbool16::shift1[rayIndex];
	    return true;
	  }
	return false;
      }

      // ==================================================================================================
      // ==================================================================================================
      // ==================================================================================================

      __forceinline static void intersect16(const vbool16 valid_leaf, 
					    const unsigned int items,
					    const Vec3vf16 &dir,
					    const Vec3vf16 &org,
					    Ray16& ray16, 
					    const Scene     *__restrict__ const scene,
					    const Triangle1 * __restrict__ tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	const vfloat16 one  = vfloat16::one();
	
	prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  1); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  2); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  3); 

	for (size_t i=0; i<items; i++,tptr++) 
	  {
	    const Triangle1& tri = *tptr;

	    prefetch<PFHINT_L1>(tptr + 1 ); 

	    STAT3(normal.trav_prims,1,popcnt(valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const vfloat16 v0 = broadcast4to16f(&tri.v0);
	    const vfloat16 v1 = broadcast4to16f(&tri.v1);
	    const vfloat16 v2 = broadcast4to16f(&tri.v2);
	    

	    const vfloat16 e1 = v0-v1;
	    const vfloat16 e2 = v2-v0;

	    /* calculate denominator */
	    const Vec3vf16 _v0 = Vec3vf16(shuffle<0>(v0),shuffle<1>(v0),shuffle<2>(v0));
	    const Vec3vf16 C =  _v0 - org;
	    
#ifndef COMPUTE_NORMAL
	    const Vec3vf16 Ng = Vec3vf16(tri.Ng);
#else
	    const vfloat16 _Ng = lcross_zxy(e1,e2);
	    const Vec3vf16 Ng(swBBBB(_Ng),swCCCC(_Ng),swAAAA(_Ng));
#endif
	    const vfloat16 den = dot(ray16.dir,Ng);

	    const vfloat16 rcp_den = rcp(den);

	    vbool16 valid = valid_leaf;

#if defined(RTCORE_BACKFACE_CULLING)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const Vec3vf16 R = -cross(C,ray16.dir);
	    const Vec3vf16 _e2(shuffle<0>(e2),shuffle<1>(e2),shuffle<2>(e2));
	    const vfloat16 u = dot(R,_e2)*rcp_den;
	    const Vec3vf16 _e1(shuffle<0>(e1),shuffle<1>(e1),shuffle<2>(e1));
	    const vfloat16 v = dot(R,_e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);

	    if (unlikely(none(valid))) continue;

	    const vfloat16 dot_C_Ng = dot(C,Ng);
	    const vfloat16 t = dot_C_Ng * rcp_den;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    const vint16 geomID = tri.geomID();
	    const vint16 primID = tri.primID();

	    ray16.prefetchHitData<PFHINT_L1EX>();

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (tri.mask() & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;


            /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		Geometry* geom = ((Scene*)scene)->get(tri.geomID());
		if (unlikely(geom->hasIntersectionFilter<vfloat16>())) {
		  runIntersectionFilter16(valid,geom,ray16,u,v,t,Ng,geomID,primID);
		  continue;
		}
	      }

	    /* update hit information */
	    ray16.update(valid,t,u,v,Ng.x,Ng.y,Ng.z,geomID,primID);
	  }

      }

      __forceinline static void occluded16(const vbool16 m_valid_leaf, 
					   const unsigned int items,
					   const Vec3vf16 &dir,
					   const Vec3vf16 &org,
					   Ray16& ray16, 
					   vbool16 &m_terminated,
					   const Scene     *__restrict__ const scene,
					   const Triangle1 * __restrict__ tptr)
      {
	prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  1); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  2); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  3); 

	const vfloat16 zero = vfloat16::zero();
	vbool16 valid_leaf = m_valid_leaf;
	for (size_t i=0; i<items; i++,tptr++) 
	  {
	    const Triangle1& tri = *tptr;

	    prefetch<PFHINT_L1>(tptr + 1 ); 

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const vfloat16 v0 = broadcast4to16f(&tri.v0);
	    const vfloat16 v1 = broadcast4to16f(&tri.v1);
	    const vfloat16 v2 = broadcast4to16f(&tri.v2);
	    const vfloat16 e1 = v0-v1;
	    const vfloat16 e2 = v2-v0;

	    /* calculate denominator */
	    const Vec3vf16 _v0 = Vec3vf16(shuffle<0>(v0),shuffle<1>(v0),shuffle<2>(v0));
	    const Vec3vf16 C =  _v0 - org;
	    
#ifndef COMPUTE_NORMAL
	    const Vec3vf16 Ng = Vec3vf16(tri.Ng);
#else
	    const vfloat16 _Ng = lcross_zxy(e1,e2);
	    const Vec3vf16 Ng(swBBBB(_Ng),swCCCC(_Ng),swAAAA(_Ng));
#endif

	    const vfloat16 den = dot(dir,Ng);

	    vbool16 valid = valid_leaf;

#if defined(RTCORE_BACKFACE_CULLING)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const vfloat16 rcp_den = rcp(den);
	    const Vec3vf16 R = cross(dir,C);
	    const Vec3vf16 _e2(shuffle<0>(e2),shuffle<1>(e2),shuffle<2>(e2));
	    const vfloat16 u = dot(R,_e2)*rcp_den;
	    const Vec3vf16 _e1(shuffle<0>(e1),shuffle<1>(e1),shuffle<2>(e1));
	    const vfloat16 v = dot(R,_e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);
	    const vfloat16 t = dot(C,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (vint16(tri.mask()) & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;
	    
	    /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		const int geomID = tri.geomID();
		const int primID = tri.primID();
		const Geometry* geom = scene->get(geomID);
		if (unlikely(geom->hasOcclusionFilter<vfloat16>()))
		  valid = runOcclusionFilter16(valid,geom,ray16,u,v,t,Ng,geomID,primID);
	      }

	    /* update occlusion */
	    m_terminated |= valid;
	    valid_leaf &= ~valid;
	    if (unlikely(none(valid_leaf))) break;
	  }
      }

    };

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template< bool ENABLE_INTERSECTION_FILTER>
    struct Triangle1Intersector16MoellerTrumboreRobust
    {
      typedef Triangle1 Primitive;

#if 1

      __forceinline static bool intersect1(const BVH4i::NodeRef curNode,
					   const size_t rayIndex, 
					   const vfloat16 &dir_xyz,
					   const vfloat16 &org_xyz,
					   const vfloat16 &min_dist_xyz,
					   vfloat16 &max_dist_xyz,
					   const vint16 &and_mask,
					   Ray16& ray16, 
					   const Precalculations &pre,
					   const Scene     *__restrict__ const geometry,
					   const Triangle1 * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	prefetch<PFHINT_L1>(tptr + 3);
	prefetch<PFHINT_L1>(tptr + 2);
	prefetch<PFHINT_L1>(tptr + 1); 
	prefetch<PFHINT_L1>(tptr + 0);  
	const vfloat16 _v0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v0,
					(float*)&tptr[1].v0,
					(float*)&tptr[2].v0,
					(float*)&tptr[3].v0);
	      
	const vfloat16 _v1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v1,
					(float*)&tptr[1].v1,
					(float*)&tptr[2].v1,
					(float*)&tptr[3].v1);
	      
	const vfloat16 _v2 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v2,
					(float*)&tptr[1].v2,
					(float*)&tptr[2].v2,
					(float*)&tptr[3].v2);

	const vfloat16 v0 = _v0 * pre.rdir_xyz - pre.org_rdir_xyz;
	const vfloat16 v1 = _v1 * pre.rdir_xyz - pre.org_rdir_xyz;
	const vfloat16 v2 = _v2 * pre.rdir_xyz - pre.org_rdir_xyz;


	const vfloat16 e0 = v2 - v0;
	const vfloat16 e1 = v0 - v1;	     
	const vfloat16 e2 = v1 - v2;	     

	const vfloat16 Ng1     = lcross_xyz(e1,e0);
	vfloat16 Ng            = Ng1+Ng1;
	const vfloat16 den     = lsum3_xyz(Ng);	      
	const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
	vbool16 m_valid = (vbool16)0x1111 & (den > zero);
#else
	vbool16 m_valid = (vbool16)0x1111;
#endif

	const vfloat16 u = lsum3_xyz(lcross_xyz(v2+v0,e0)) * rcp_den; 
	m_valid       = ge( m_valid, u, zero);

	const vfloat16 v = lsum3_xyz(lcross_xyz(v0+v1,e1)) * rcp_den; 
	m_valid       = ge( m_valid, v, zero);

	const vfloat16 w = lsum3_xyz(lcross_xyz(v1+v2,e2)) * rcp_den;  
	m_valid       = ge( m_valid, w, zero);


	if (unlikely(none(m_valid))) return false;

	const vbool16 m_den = ne(m_valid,den,zero);
	const vfloat16 t = ldot3_xyz(v0,Ng) * rcp_den;
	vbool16 m_final      = lt(lt(m_den,min_dist_xyz,t),t,max_dist_xyz);


#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray16.mask[rayIndex]);
	const vint16 triMask = getTriMasks(tptr); 
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif

	/* correct normal */
	Ng *= shuffle(dir_xyz * shuffle(dir_xyz,_MM_SWIZ_REG_DACB),_MM_SWIZ_REG_DACB);


	//////////////////////////////////////////////////////////////////////////////////////////////////

	/* did the ray hit one of the four triangles? */
	if (unlikely(any(m_final)))
	  {
	    STAT3(normal.trav_prim_hits,1,1,1);

	    /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		vfloat16 org_max_dist_xyz = max_dist_xyz;

		/* did the ray hit one of the four triangles? */
		while (any(m_final)) 
		  {
		    max_dist_xyz  = select(m_final,t,org_max_dist_xyz);
		    const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		    const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		    const size_t vecIndex = bitscan(toInt(m_dist));
		    const size_t triIndex = vecIndex >> 2;
		    const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		    const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

#ifndef COMPUTE_NORMAL
		    const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		    const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		    const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		    const vfloat16 gnormalx = swAAAA(Ng);
		    const vfloat16 gnormaly = swBBBB(Ng);
		    const vfloat16 gnormalz = swCCCC(Ng);
#endif                

		    const int geomID = tri_ptr->geomID();
		    const int primID = tri_ptr->primID();
                
		    const Geometry* const geom = geometry->get(geomID);
		    if (likely(!geom->hasIntersectionFilter<vfloat16>())) 
		      {
			ray16.update(m_tri,rayIndex,min_dist,u,v,gnormalx,gnormaly,gnormalz,geomID,primID);
			break;
		      }
                
		    if (runIntersectionFilter16(geom,ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
		      break;
		    }
		    m_final ^= m_tri;
		  }
		max_dist_xyz = ray16.tfar[rayIndex];
	      }
	    else
	      {
		ray16.prefetchHitData<PFHINT_L1EX>();

		max_dist_xyz  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;

		const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;

		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

#ifndef COMPUTE_NORMAL
		const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		const vfloat16 gnormalx = swAAAA(Ng);
		const vfloat16 gnormaly = swBBBB(Ng);
		const vfloat16 gnormalz = swCCCC(Ng);
#endif                
		max_dist_xyz = min_dist;

	
		ray16.update(m_tri,rayIndex,min_dist,u,v,gnormalx,gnormaly,gnormalz,tri_ptr->geomID(),tri_ptr->primID());
	      }
	    return true;
      
	  }
	return false;
      }
#else

      __forceinline static bool intersect1(const BVH4i::NodeRef curNode,
					   const size_t rayIndex, 
					   const vfloat16 &dir_xyz,
					   const vfloat16 &org_xyz,
					   const vfloat16 &min_dist_xyz,
					   vfloat16 &max_dist_xyz,
					   const vint16 &and_mask,
					   Ray16& ray16, 
					   const Precalculations &pre,
					   const Scene     *__restrict__ const geometry,
					   const Triangle1 * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	prefetch<PFHINT_L1>(tptr + 3);
	prefetch<PFHINT_L1>(tptr + 2);
	prefetch<PFHINT_L1>(tptr + 1); 
	prefetch<PFHINT_L1>(tptr + 0);  
	const vfloat16 _v0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v0,
					(float*)&tptr[1].v0,
					(float*)&tptr[2].v0,
					(float*)&tptr[3].v0);
	      
	const vfloat16 _v1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v1,
					(float*)&tptr[1].v1,
					(float*)&tptr[2].v1,
					(float*)&tptr[3].v1);
	      
	const vfloat16 _v2 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v2,
					(float*)&tptr[1].v2,
					(float*)&tptr[2].v2,
					(float*)&tptr[3].v2);
	const vfloat16 v0 = _v0 - org_xyz;
	const vfloat16 v1 = _v1 - org_xyz;
	const vfloat16 v2 = _v2 - org_xyz;

	const vfloat16 e0 = v2 - v0;
	const vfloat16 e1 = v0 - v1;	     
	const vfloat16 e2 = v1 - v2;	     

	const vfloat16 Ng1     = lcross_xyz(e1,e0);
	const vfloat16 Ng      = Ng1+Ng1;
	const vfloat16 den     = ldot3_xyz(Ng,dir_xyz);	      
	const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
	vbool16 m_valid = (vbool16)0x1111 & (den > zero);
#else
	vbool16 m_valid = (vbool16)0x1111;
#endif

	const vfloat16 u = ldot3_xyz(lcross_xyz(v2+v0,e0),dir_xyz) * rcp_den; 
	m_valid       = ge( m_valid, u, zero);

	const vfloat16 v = ldot3_xyz(lcross_xyz(v0+v1,e1),dir_xyz) * rcp_den; 
	m_valid       = ge( m_valid, v, zero);

	const vfloat16 w = ldot3_xyz(lcross_xyz(v1+v2,e2),dir_xyz) * rcp_den;  
	m_valid       = ge( m_valid, w, zero);


	if (unlikely(none(m_valid))) return false;

	const vbool16 m_den = ne(m_valid,den,zero);
	const vfloat16 t = ldot3_xyz(v0,Ng) * rcp_den;
	vbool16 m_final      = lt(lt(m_den,min_dist_xyz,t),t,max_dist_xyz);


#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray16.mask[rayIndex]);
	const vint16 triMask = getTriMasks(tptr); 
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif

	//////////////////////////////////////////////////////////////////////////////////////////////////

	/* did the ray hit one of the four triangles? */
	if (unlikely(any(m_final)))
	  {
	    STAT3(normal.trav_prim_hits,1,1,1);

	    /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		vfloat16 org_max_dist_xyz = max_dist_xyz;

		/* did the ray hit one of the four triangles? */
		while (any(m_final)) 
		  {
		    max_dist_xyz  = select(m_final,t,org_max_dist_xyz);
		    const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		    const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		    const size_t vecIndex = bitscan(toInt(m_dist));
		    const size_t triIndex = vecIndex >> 2;
		    const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		    const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

#ifndef COMPUTE_NORMAL
		    const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		    const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		    const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		    const vfloat16 gnormalx = swAAAA(Ng);
		    const vfloat16 gnormaly = swBBBB(Ng);
		    const vfloat16 gnormalz = swCCCC(Ng);
#endif                

		    const int geomID = tri_ptr->geomID();
		    const int primID = tri_ptr->primID();
                
		    const Geometry* const geom = geometry->get(geomID);
		    if (likely(!geom->hasIntersectionFilter<vfloat16>())) 
		      {
			ray16.update(m_tri,rayIndex,min_dist,u,v,gnormalx,gnormaly,gnormalz,geomID,primID);
			break;
		      }
                
		    if (runIntersectionFilter16(geom,ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
		      break;
		    }
		    m_final ^= m_tri;
		  }
		max_dist_xyz = ray16.tfar[rayIndex];
	      }
	    else
	      {
		ray16.prefetchHitData<PFHINT_L1EX>();

		max_dist_xyz  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;

		const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;

		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

#ifndef COMPUTE_NORMAL
		const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		const vfloat16 gnormalx = swAAAA(Ng);
		const vfloat16 gnormaly = swBBBB(Ng);
		const vfloat16 gnormalz = swCCCC(Ng);
#endif                
		max_dist_xyz = min_dist;

	
		ray16.update(m_tri,rayIndex,min_dist,u,v,gnormalx,gnormaly,gnormalz,tri_ptr->geomID(),tri_ptr->primID());
	      }
	    return true;
      
	  }
	return false;
      }

#endif

      __forceinline static bool occluded1(const BVH4i::NodeRef curNode,					   
					  const size_t rayIndex, 
					  const vfloat16 &dir_xyz,
					  const vfloat16 &org_xyz,
					  const vfloat16 &min_dist_xyz,
					  const vfloat16 &max_dist_xyz,
					  const vint16 &and_mask,
					  const Ray16& ray16, 
					  const Precalculations &pre,
					  vbool16 &m_terminated,
					  const Scene     *__restrict__ const geometry,
					  const Triangle1 * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	prefetch<PFHINT_L1>(tptr + 3);
	prefetch<PFHINT_L1>(tptr + 2);
	prefetch<PFHINT_L1>(tptr + 1); 
	prefetch<PFHINT_L1>(tptr + 0);  
	const vfloat16 _v0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v0,
					(float*)&tptr[1].v0,
					(float*)&tptr[2].v0,
					(float*)&tptr[3].v0);
	      
	const vfloat16 _v1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v1,
					(float*)&tptr[1].v1,
					(float*)&tptr[2].v1,
					(float*)&tptr[3].v1);
	      
	const vfloat16 _v2 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].v2,
					(float*)&tptr[1].v2,
					(float*)&tptr[2].v2,
					(float*)&tptr[3].v2);

	const vfloat16 v0 = _v0 - org_xyz;
	const vfloat16 v1 = _v1 - org_xyz;
	const vfloat16 v2 = _v2 - org_xyz;

	const vfloat16 e0 = v2 - v0;
	const vfloat16 e1 = v0 - v1;	     
	const vfloat16 e2 = v1 - v2;	     

	const vfloat16 Ng1     = lcross_xyz(e1,e0);
	const vfloat16 Ng      = Ng1+Ng1;
	const vfloat16 den     = ldot3_xyz(Ng,dir_xyz);	      
	const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
	vbool16 m_valid = (vbool16)0x1111 & (den > zero);
#else
	vbool16 m_valid = (vbool16)0x1111;
#endif

	const vfloat16 u = ldot3_xyz(lcross_xyz(v2+v0,e0),dir_xyz) * rcp_den; 
	m_valid       = ge( m_valid, u, zero);

	const vfloat16 v = ldot3_xyz(lcross_xyz(v0+v1,e1),dir_xyz) * rcp_den; 
	m_valid       = ge( m_valid, v, zero);

	const vfloat16 w = ldot3_xyz(lcross_xyz(v1+v2,e2),dir_xyz) * rcp_den;  
	m_valid       = ge( m_valid, w, zero);

	const vbool16 m_den = ne(m_valid,den,zero);
	const vfloat16 t = ldot3_xyz(v0,Ng) * rcp_den;

	if (unlikely(none(m_valid))) return false;

	vbool16 m_final  = lt(lt(m_valid,min_dist_xyz,t),t,max_dist_xyz);

#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray16.mask[rayIndex]);
	const vint16 triMask = getTriMasks(tptr); 
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif

	if (ENABLE_INTERSECTION_FILTER) 
	  {
              
	    /* did the ray hit one of the four triangles? */
	    while (any(m_final)) 
	      {
		const vfloat16 temp_t  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(temp_t);
		const vbool16 m_dist = eq(min_dist,temp_t);
		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;
		const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));
#ifndef COMPUTE_NORMAL
		const vfloat16 gnormalx = vfloat16(tri_ptr->Ng.x);
		const vfloat16 gnormaly = vfloat16(tri_ptr->Ng.y);
		const vfloat16 gnormalz = vfloat16(tri_ptr->Ng.z);
#else
		const vfloat16 gnormalx = swAAAA(Ng);
		const vfloat16 gnormaly = swBBBB(Ng);
		const vfloat16 gnormalz = swCCCC(Ng);
#endif                

		const int geomID = tri_ptr->geomID();
		const int primID = tri_ptr->primID();                
		const Geometry* const geom = geometry->get(geomID);
		if (likely(!geom->hasOcclusionFilter<vfloat16>())) break;
                
		if (runOcclusionFilter16(geom,(Ray16&)ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) 
		  break;

		m_final ^= m_tri; /* clear bit */
	      }
      
	  }
	if (unlikely(any(m_final)))
	  {
	    STAT3(shadow.trav_prim_hits,1,1,1);
	    m_terminated |= vbool16::shift1[rayIndex];
	    return true;
	  }
	return false;
      }

      // ==================================================================================================
      // ==================================================================================================
      // ==================================================================================================

      __forceinline static void intersect16(const vbool16 valid_leaf, 
					    const unsigned int items,
					    const Vec3vf16 &dir,
					    const Vec3vf16 &org,
					    Ray16& ray16, 
					    const Scene     *__restrict__ const scene,
					    const Triangle1 * __restrict__ tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	const vfloat16 one  = vfloat16::one();
	
	prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  1); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  2); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  3); 

	for (size_t i=0; i<items; i++,tptr++) 
	  {
	    const Triangle1& tri = *tptr;

	    prefetch<PFHINT_L1>(tptr + 1 ); 

	    STAT3(normal.trav_prims,1,popcnt(valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const vfloat16 v0 = broadcast4to16f(&tri.v0) - org.x;
	    const vfloat16 v1 = broadcast4to16f(&tri.v1) - org.y;
	    const vfloat16 v2 = broadcast4to16f(&tri.v2) - org.z;
	    

	    const Vec3vf16 Ng1     = cross(e1,e0);
	    const Vec3vf16 Ng      = Ng1+Ng1;
	    const vfloat16 den     = dot(Ng,ray_dir);	      
	    const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)	    
	    vbool16 valid = den > zero;
#else
	    vbool16 valid = MIC_M_ALL;
#endif

	    const vfloat16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
	    valid       = ge( valid, u, zero);

	    const vfloat16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
	    valid       = ge( valid, v, zero);

	    const vfloat16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
	    valid       = ge( valid, w, zero);


	    if (unlikely(none(m_valid))) continue;

	    const vbool16 m_den = ne(m_valid,den,zero);
	    const vfloat16 t = dot(v0,Ng) * rcp_den;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    const vint16 geomID = tri.geomID();
	    const vint16 primID = tri.primID();

	    ray16.prefetchHitData<PFHINT_L1EX>();

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (tri.mask() & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;


            /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		Geometry* geom = ((Scene*)scene)->get(tri.geomID());
		if (unlikely(geom->hasIntersectionFilter<vfloat16>())) {
		  runIntersectionFilter16(valid,geom,ray16,u,v,t,Ng,geomID,primID);
		  continue;
		}
	      }

	    /* update hit information */
	    ray16.update(valid,t,u,v,Ng.x,Ng.y,Ng.z,geomID,primID);
	  }

      }

      __forceinline static void occluded16(const vbool16 m_valid_leaf, 
					   const unsigned int items,
					   const Vec3vf16 &dir,
					   const Vec3vf16 &org,
					   Ray16& ray16, 
					   vbool16 &m_terminated,
					   const Scene     *__restrict__ const scene,
					   const Triangle1 * __restrict__ tptr)
      {
	prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  1); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  2); 
	prefetch<PFHINT_L2>((vfloat16*)tptr +  3); 

	const vfloat16 zero = vfloat16::zero();
	vbool16 valid_leaf = m_valid_leaf;
	for (size_t i=0; i<items; i++,tptr++) 
	  {
	    const Triangle1& tri = *tptr;

	    prefetch<PFHINT_L1>(tptr + 1 ); 

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const vfloat16 v0 = broadcast4to16f(&tri.v0) - org.x;
	    const vfloat16 v1 = broadcast4to16f(&tri.v1) - org.y;
	    const vfloat16 v2 = broadcast4to16f(&tri.v2) - org.z;
	    

	    const Vec3vf16 Ng1     = cross(e1,e0);
	    const Vec3vf16 Ng      = Ng1+Ng1;
	    const vfloat16 den     = dot(Ng,ray_dir);	      
	    const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)	    
	    vbool16 valid = den > zero;
#else
	    vbool16 valid = MIC_M_ALL;
#endif

	    const vfloat16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
	    valid       = ge( valid, u, zero);

	    const vfloat16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
	    valid       = ge( valid, v, zero);

	    const vfloat16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
	    valid       = ge( valid, w, zero);

	    const vbool16 m_den = ne(m_valid,den,zero);
	    const vfloat16 t = dot(v0,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (vint16(tri.mask()) & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;
	    
	    /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		const int geomID = tri.geomID();
		const int primID = tri.primID();
		const Geometry* geom = scene->get(geomID);
		if (unlikely(geom->hasOcclusionFilter<vfloat16>()))
		  valid = runOcclusionFilter16(valid,geom,ray16,u,v,t,Ng,geomID,primID);
	      }

	    /* update occlusion */
	    m_terminated |= valid;
	    valid_leaf &= ~valid;
	    if (unlikely(none(valid_leaf))) break;
	  }
      }

    };

}
