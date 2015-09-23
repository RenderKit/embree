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
  /*! Intersector for individual precomputed triangles with one
   *  ray. This intersector implements a modified version of the
   *  Moeller Trumbore intersector from the paper "Fast, Minimum
   *  Storage Ray-Triangle Intersection". In contrast to the paper we
   *  precalculate some factors and factor the calculations
   *  differently to allow precalculating the cross product e1 x
   *  e2. */

  template< bool ENABLE_INTERSECTION_FILTER>
    struct Triangle1mcIntersector1MoellerTrumbore
    {

      __forceinline static bool intersect1(const vfloat16 &dir_xyz,
					   const vfloat16 &org_xyz,
					   const vfloat16 &min_dist_xyz,
					   vfloat16 &max_dist_xyz,
					   const vint16 &and_mask,
					   Ray& ray, 
					   const Scene     *__restrict__ const scene,
					   const Triangle1mc * __restrict__ const tptr)
      {
	const vfloat16 zero = vfloat16::zero();
	prefetch<PFHINT_L1>(tptr + 2);
	prefetch<PFHINT_L1>(tptr + 0); 

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
	const vint16 rayMask(ray.mask);
	const vint16 triMask0( scene->getTriangleMesh( tptr[0].geomID() )->mask );
	const vint16 triMask1( scene->getTriangleMesh( tptr[1].geomID() )->mask );
	const vint16 triMask2( scene->getTriangleMesh( tptr[2].geomID() )->mask );
	const vint16 triMask3( scene->getTriangleMesh( tptr[3].geomID() )->mask );
	const vint16 triMask = gather16i_4i_align(triMask0,triMask1,triMask2,triMask3);
	const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
	m_final &= m_ray_mask;	      
#endif


	//////////////////////////////////////////////////////////////////////////////////////////////////

	/* did the ray hot one of the four triangles? */
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
                
		    const Geometry* geom = scene->get(geomID);
		    const vfloat16 gnormalx(normal[triIndex*4+1]);
		    const vfloat16 gnormaly(normal[triIndex*4+2]);
		    const vfloat16 gnormalz(normal[triIndex*4+0]);

		    if (likely(!geom->hasIntersectionFilter1())) 
		      {
			compactustore16f_low(m_tri,&ray.tfar,min_dist);
			compactustore16f_low(m_tri,&ray.u,u); 
			compactustore16f_low(m_tri,&ray.v,v); 
			compactustore16f_low(m_tri,&ray.Ng.x,gnormalx); 
			compactustore16f_low(m_tri,&ray.Ng.y,gnormaly); 
			compactustore16f_low(m_tri,&ray.Ng.z,gnormalz); 
			ray.geomID = geomID;
			ray.primID = primID;
			max_dist_xyz = min_dist;
			break;
		      }
                
		    if (runIntersectionFilter1(geom,ray,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
		      max_dist_xyz = min_dist;
		      break;
		    }
		    m_final ^= m_tri;
		  }
		max_dist_xyz = ray.tfar;
	      }
	    else
	      {
		max_dist_xyz  = select(m_final,t,max_dist_xyz);
		const vfloat16 min_dist = vreduce_min(max_dist_xyz);
		const vbool16 m_dist = eq(min_dist,max_dist_xyz);

		prefetch<PFHINT_L1EX>((vfloat16*)&ray + 0);
		prefetch<PFHINT_L1EX>((vfloat16*)&ray + 1);

		const size_t vecIndex = bitscan(toInt(m_dist));
		const size_t triIndex = vecIndex >> 2;

		const Triangle1mc  *__restrict__ tri_ptr = tptr + triIndex;

		const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

		max_dist_xyz = min_dist;

		const vfloat16 gnormalx(normal[triIndex*4+1]);
		const vfloat16 gnormaly(normal[triIndex*4+2]);
		const vfloat16 gnormalz(normal[triIndex*4+0]);

		compactustore16f_low(m_tri,&ray.tfar,min_dist);
		compactustore16f_low(m_tri,&ray.u,u); 
		compactustore16f_low(m_tri,&ray.v,v); 
		compactustore16f_low(m_tri,&ray.Ng.x,gnormalx); 
		compactustore16f_low(m_tri,&ray.Ng.y,gnormaly); 
		compactustore16f_low(m_tri,&ray.Ng.z,gnormalz); 

		ray.geomID = tri_ptr->geomID();
		ray.primID = tri_ptr->primID();
	      }
	    return true;

	  }
	return false;
      }


      __forceinline static bool occluded1(const vfloat16 &dir_xyz,
					  const vfloat16 &org_xyz,
					  const vfloat16 &min_dist_xyz,
					  const vfloat16 &max_dist_xyz,
					  const vint16 &and_mask,
					  const Ray& ray, 
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
	if (unlikely(none(m_aperture))) return m_aperture;

	vbool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(RTCORE_RAY_MASK)
	const vint16 rayMask(ray.mask);
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

		const vfloat16 gnormalx(normal[triIndex*4+1]);
		const vfloat16 gnormaly(normal[triIndex*4+2]);
		const vfloat16 gnormalz(normal[triIndex*4+0]);

		const int geomID = tri_ptr->geomID();
		const int primID = tri_ptr->primID();                
		const Geometry* geom = scene->get(geomID);

		if (likely(!geom->hasOcclusionFilter1())) break;
                
		if (runOcclusionFilter1(geom,(Ray&)ray,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) 
		  break;

		m_final ^= m_tri; /* clear bit */
	      }
	  }
      return any(m_final);
    }

  };
}
