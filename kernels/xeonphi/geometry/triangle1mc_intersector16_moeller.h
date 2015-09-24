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
    struct Triangle1mcIntersector16MoellerTrumbore
    {

      __forceinline static bool intersect1(const size_t rayIndex, 
					   const vfloat16 &dir_xyz,
					   const vfloat16 &org_xyz,
					   const vfloat16 &min_dist_xyz,
					   vfloat16 &max_dist_xyz,
					   const vint16 &and_mask,
					   Ray16& ray16, 
					   const Scene     *__restrict__ const scene,
					   const Triangle1mc * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	prefetch<PFHINT_L1>(tptr + 0); 
	prefetch<PFHINT_L1>(tptr + 2);


	const vfloat16 v0 = gather_4f_zlc(and_mask,
				       tptr[0].v0,
				       tptr[1].v0,
				       tptr[2].v0,
				       tptr[3].v0);
	      
	const vfloat16 v1 = gather_4f_zlc(and_mask,
				       tptr[0].v1,
				       tptr[1].v1,
				       tptr[2].v1,
				       tptr[3].v1);
	      
	const vfloat16 v2 = gather_4f_zlc(and_mask,
				       tptr[0].v2,
				       tptr[1].v2,
				       tptr[2].v2,
				       tptr[3].v2);

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

	vbool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);


#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray16.mask[rayIndex]);
	const vint16 triMask0( scene->getTriangleMesh( tptr[0].geomID() )->mask );
	const vint16 triMask1( scene->getTriangleMesh( tptr[1].geomID() )->mask );
	const vint16 triMask2( scene->getTriangleMesh( tptr[2].geomID() )->mask );
	const vint16 triMask3( scene->getTriangleMesh( tptr[3].geomID() )->mask );
	const vint16 triMask = gather16i_4i_align(triMask0,triMask1,triMask2,triMask3);
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif

	//////////////////////////////////////////////////////////////////////////////////////////////////

	/* did the ray hit one of the four triangles? */
	if (unlikely(any(m_final)))
	  {
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
		    const Triangle1mc  *__restrict__ tri_ptr = tptr + triIndex;
		    const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));
		    const int geomID = tri_ptr->geomID();
		    const int primID = tri_ptr->primID();

		    const vfloat16 gnormalx(normal[triIndex*4+1]);
		    const vfloat16 gnormaly(normal[triIndex*4+2]);
		    const vfloat16 gnormalz(normal[triIndex*4+0]);

                
		    const Geometry* const geom = scene->get(geomID);
		    if (likely(!geom->hasIntersectionFilter<vfloat16>())) 
		      {

			compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
			compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
			compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
			compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
			compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
			compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 
			ray16.geomID[rayIndex] = geomID;
			ray16.primID[rayIndex] = primID;
			max_dist_xyz = min_dist;
			break;
		      }
                
		    if (runIntersectionFilter16(geom,ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
		      max_dist_xyz = min_dist;
		      break;
		    }
		    m_final ^= m_tri;
		  }
		max_dist_xyz = ray16.tfar[rayIndex];
	      }
	    else
	      {
		STAT3(normal.trav_prim_hits,1,1,1);
		max_dist_xyz  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		const vbool16 m_dist = eq(min_dist,max_dist_xyz);
		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;

		const Triangle1mc  *__restrict__ tri_ptr = tptr + triIndex;

		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

                
		prefetch<PFHINT_L1EX>(&ray16.tfar);  
		prefetch<PFHINT_L1EX>(&ray16.u);
		prefetch<PFHINT_L1EX>(&ray16.v);
		prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
		prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
		prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
		prefetch<PFHINT_L1EX>(&ray16.geomID);
		prefetch<PFHINT_L1EX>(&ray16.primID);

		max_dist_xyz = min_dist;

		const vfloat16 gnormalx(normal[triIndex*4+1]);
		const vfloat16 gnormaly(normal[triIndex*4+2]);
		const vfloat16 gnormalz(normal[triIndex*4+0]);
		  
		compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
		compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
		compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
		compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
		compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
		compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 

		ray16.geomID[rayIndex] = tri_ptr->geomID();
		ray16.primID[rayIndex] = tri_ptr->primID();
	      }
	    return true;
      
	  }
	return false;
      }


      __forceinline static bool occluded1(const size_t rayIndex, 
					  const vfloat16 &dir_xyz,
					  const vfloat16 &org_xyz,
					  const vfloat16 &min_dist_xyz,
					  const vfloat16 &max_dist_xyz,
					  const vint16 &and_mask,
					  const Ray16& ray16, 
					  vbool16 &m_terminated,
					  const Scene     *__restrict__ const scene,
					  const Triangle1mc * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	prefetch<PFHINT_L1>(tptr + 0); 
	prefetch<PFHINT_L1>(tptr + 2);

	      
	const vfloat16 v0 = gather_4f_zlc(and_mask,
				       tptr[0].v0,
				       tptr[1].v0,
				       tptr[2].v0,
				       tptr[3].v0);
	      
	const vfloat16 v1 = gather_4f_zlc(and_mask,
				       tptr[0].v1,
				       tptr[1].v1,
				       tptr[2].v1,
				       tptr[3].v1);
	      
	const vfloat16 v2 = gather_4f_zlc(and_mask,
				       tptr[0].v2,
				       tptr[1].v2,
				       tptr[2].v2,
				       tptr[3].v2);

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
	const vint16 triMask0( scene->getTriangleMesh( tptr[0].geomID() )->mask );
	const vint16 triMask1( scene->getTriangleMesh( tptr[1].geomID() )->mask );
	const vint16 triMask2( scene->getTriangleMesh( tptr[2].geomID() )->mask );
	const vint16 triMask3( scene->getTriangleMesh( tptr[3].geomID() )->mask );
	const vint16 triMask = gather16i_4i_align(triMask0,triMask1,triMask2,triMask3);

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
		const Triangle1mc  *__restrict__ tri_ptr = tptr + triIndex;
		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));
		const int geomID = tri_ptr->geomID();
		const int primID = tri_ptr->primID();                
		const Geometry* const geom = scene->get(geomID);
		const vfloat16 gnormalx(normal[triIndex*4+1]);
		const vfloat16 gnormaly(normal[triIndex*4+2]);
		const vfloat16 gnormalz(normal[triIndex*4+0]);

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
					    const Triangle1mc * __restrict__ tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	const vfloat16 one  = vfloat16::one();
	
	prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
	prefetch<PFHINT_L1>((vfloat16*)tptr +  2); 

	for (size_t i=0; i<items; i++,tptr++) 
	  {

	    const Triangle1mc& tri = *tptr;
	    STAT3(normal.trav_prims,1,popcnt(valid_leaf),16);

	    /* load vertices and calculate edges */
	    const vfloat16 v0 = broadcast4to16f(tri.v0);
	    const vfloat16 v1 = broadcast4to16f(tri.v1);
	    const vfloat16 v2 = broadcast4to16f(tri.v2);
	    

	    const vfloat16 e1 = v0-v1;
	    const vfloat16 e2 = v2-v0;

	    /* calculate denominator */
	    const Vec3vf16 _v0 = Vec3vf16(shuffle<0>(v0),shuffle<1>(v0),shuffle<2>(v0));
	    const Vec3vf16 C =  _v0 - org;
	    
	    //const Vec3vf16 Ng = Vec3vf16(tri.Ng);
	    const vfloat16 normal_xyz = lcross_zxy(e1,e2);
	    const Vec3vf16 Ng(swBBBB(normal_xyz),swCCCC(normal_xyz),swAAAA(normal_xyz));

	    const vfloat16 den = dot(Ng,ray16.dir);

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

	    prefetch<PFHINT_L1EX>(&ray16.u);      
	    prefetch<PFHINT_L1EX>(&ray16.v);      
	    prefetch<PFHINT_L1EX>(&ray16.tfar);      


	    if (unlikely(none(valid))) continue;

	    const vfloat16 dot_C_Ng = dot(C,Ng);
	    const vfloat16 t = dot_C_Ng * rcp_den;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    const vint16 geomID = tri.geomID();
	    const vint16 primID = tri.primID();
	    prefetch<PFHINT_L1EX>(&ray16.geomID);      
	    prefetch<PFHINT_L1EX>(&ray16.primID);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.x);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.y);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.z);      

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    const vint16 tri_mask = scene->getTriangleMesh( tri.geomID() )->mask;
	    valid &= (tri_mask & ray16.mask) != 0;
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
	    vfloat16::store(valid,(float*)&ray16.u,u);
	    vfloat16::store(valid,(float*)&ray16.v,v);
	    vfloat16::store(valid,(float*)&ray16.tfar,t);
	    vint16::store(valid,(float*)&ray16.geomID,geomID);
	    vint16::store(valid,(float*)&ray16.primID,primID);
	    vfloat16::store(valid,(float*)&ray16.Ng.x,Ng.x);
	    vfloat16::store(valid,(float*)&ray16.Ng.y,Ng.y);
	    vfloat16::store(valid,(float*)&ray16.Ng.z,Ng.z);
	  }

      }

      __forceinline static void occluded16(const vbool16 m_valid_leaf, 
					   const unsigned int items,
					   const Vec3vf16 &dir,
					   const Vec3vf16 &org,
					   Ray16& ray16, 
					   vbool16 &m_terminated,
					   const Scene     *__restrict__ const scene,
					   const Triangle1mc * __restrict__ tptr)
      {
	prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
	prefetch<PFHINT_L1>((vfloat16*)tptr +  2); 

	const vfloat16 zero = vfloat16::zero();
	vbool16 valid_leaf = m_valid_leaf;
	for (size_t i=0; i<items; i++,tptr++) 
	  {
	    const Triangle1mc& tri = *tptr;

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const vfloat16 v0 = broadcast4to16f(tri.v0);
	    const vfloat16 v1 = broadcast4to16f(tri.v1);
	    const vfloat16 v2 = broadcast4to16f(tri.v2);
	    const vfloat16 e1 = v0-v1;
	    const vfloat16 e2 = v2-v0;

	    /* calculate denominator */
	    const Vec3vf16 _v0 = Vec3vf16(shuffle<0>(v0),shuffle<1>(v0),shuffle<2>(v0));
	    const Vec3vf16 C =  _v0 - org;
	    
	    const vfloat16 normal_xyz = lcross_zxy(e1,e2);
	    const Vec3vf16 Ng(swBBBB(normal_xyz),swCCCC(normal_xyz),swAAAA(normal_xyz));

	    const vfloat16 den = dot(Ng,dir);

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
	    const vint16 tri_mask = scene->getTriangleMesh( tri.geomID() )->mask;
	    valid &= (tri_mask & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;
	    
	    /* intersection filter test */
	    if (ENABLE_INTERSECTION_FILTER) 
	      {
		const int geomID = tri.geomID();
		const int primID = tri.primID();
		const Geometry* geom = scene->get(geomID);
		if (unlikely(geom->hasOcclusionFilter<vfloat16>()))
		  valid = runOcclusionFilter16(valid,geom,(Ray16&)ray16,u,v,t,Ng,geomID,primID);
	      }
	    /* update occlusion */
	    m_terminated |= valid;
	  valid_leaf &= ~valid;
	  if (unlikely(none(valid_leaf))) break;
	}
    }

  };
}
