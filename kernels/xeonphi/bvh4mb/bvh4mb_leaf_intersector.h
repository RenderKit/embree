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

#include "bvh4mb.h"

namespace embree
{
  static __aligned(64) int zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};

  struct Triangle1mbLeafIntersector
  {
    // ==================
    // === single ray === 
    // ==================
    static __forceinline bool intersect(BVH4i::NodeRef curNode,
					const vfloat16 &dir_xyz,
					const vfloat16 &org_xyz,
					const vfloat16 &min_dist_xyz,
					vfloat16 &max_dist_xyz,
					Ray& ray, 
					const void *__restrict__ const accel,
					const Scene*__restrict__ const geometry)
    {
      const vfloat16 time     = vfloat16::broadcast(&ray.time);
      const vfloat16 one_time = (vfloat16::one() - time);

      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);
      prefetch<PFHINT_L2>((vfloat16*)tptr +  0); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  1); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  2); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  3); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  4); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  5); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  6); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  7); 

      const vint16 and_mask = broadcast4to16i(zlc4);
	      
      const vfloat16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const vfloat16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const vfloat16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);

      const vfloat16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const vfloat16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const vfloat16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);

      const vfloat16 v0 = v0_t0 * one_time + time * v0_t1;
      const vfloat16 v1 = v1_t0 * one_time + time * v1_t1;
      const vfloat16 v2 = v2_t0 * one_time + time * v2_t1;

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

      max_dist_xyz  = select(m_final,t,max_dist_xyz);
		    
      //////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(RTCORE_RAY_MASK)
      const vint16 rayMask(ray.mask);
      const vint16 triMask = getTriMasks(tptr); 
      const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
      m_final &= m_ray_mask;	      
#endif


      /* did the ray hot one of the four triangles? */
      if (unlikely(any(m_final)))
	{
	  const vfloat16 min_dist = vreduce_min(max_dist_xyz);
	  const vbool16 m_dist = eq(min_dist,max_dist_xyz);

	  prefetch<PFHINT_L1EX>((vfloat16*)&ray + 0);
	  prefetch<PFHINT_L1EX>((vfloat16*)&ray + 1);

	  const size_t vecIndex = bitscan(toInt(m_dist));
	  const size_t triIndex = vecIndex >> 2;

	  const BVH4mb::Triangle01  *__restrict__ tri_ptr = tptr + triIndex;

	  const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));

	  const vfloat16 gnormalz = swAAAA(normal);
	  const vfloat16 gnormalx = swBBBB(normal);
	  const vfloat16 gnormaly = swCCCC(normal);
		  
	  max_dist_xyz = min_dist;

	  compactustore16f_low(m_tri,&ray.tfar,min_dist);
	  compactustore16f_low(m_tri,&ray.u,u); 
	  compactustore16f_low(m_tri,&ray.v,v); 
	  compactustore16f_low(m_tri,&ray.Ng.x,gnormalx); 
	  compactustore16f_low(m_tri,&ray.Ng.y,gnormaly); 
	  compactustore16f_low(m_tri,&ray.Ng.z,gnormalz); 

	  ray.geomID = tri_ptr->t0.geomID();
	  ray.primID = tri_ptr->t0.primID();
	  return true;
	}
      return false;
    }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const vfloat16 &dir_xyz,
					 const vfloat16 &org_xyz,
					 const vfloat16 &min_dist_xyz,
					 const vfloat16 &max_dist_xyz,
					 Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
    {
      const vfloat16 time     = vfloat16::broadcast(&ray.time);
      const vfloat16 one_time = (vfloat16::one() - time);
      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);

      prefetch<PFHINT_L2>((vfloat16*)tptr +  0); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  1); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  2); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  3); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  4); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  5); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  6); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  7); 

      const vint16 and_mask = broadcast4to16i(zlc4);
	      
      const vfloat16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const vfloat16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const vfloat16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);

      const vfloat16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const vfloat16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const vfloat16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);


      const vfloat16 v0 = v0_t0 * one_time + time * v0_t1;
      const vfloat16 v1 = v1_t0 * one_time + time * v1_t1;
      const vfloat16 v2 = v2_t0 * one_time + time * v2_t1;

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
      const vfloat16 t = rcp_den*nom;

      if (unlikely(none(m_aperture))) return false;

      vbool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(RTCORE_RAY_MASK)
      const vint16 rayMask(ray.mask);
      const vint16 triMask = getTriMasks(tptr); 
      const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
      m_final &= m_ray_mask;	      
#endif

      return any(m_final);
    }


    // ============================================
    // ==== single ray mode for 16-wide packets ===
    // ============================================
    static __forceinline bool intersect(BVH4i::NodeRef curNode,
					const size_t rayIndex, 
					const vfloat16 &dir_xyz,
					const vfloat16 &org_xyz,
					const vfloat16 &min_dist_xyz,
					vfloat16 &max_dist_xyz,
					Ray16& ray16, 
					const void *__restrict__ const accel,
					const Scene*__restrict__ const geometry)
    {
      const vfloat16 time     = vfloat16::broadcast(&ray16.time[rayIndex]);
      const vfloat16 one_time = (vfloat16::one() - time);

      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);
	      
      prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
      prefetch<PFHINT_L1>((vfloat16*)tptr +  1); 
      prefetch<PFHINT_L1>((vfloat16*)tptr +  2); 
      prefetch<PFHINT_L1>((vfloat16*)tptr +  3); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  4); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  5); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  6); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  7); 

      const vint16 and_mask = broadcast4to16i(zlc4);
	     

      const vfloat16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const vfloat16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const vfloat16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);

      const vfloat16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const vfloat16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const vfloat16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);

      const vfloat16 v0 = v0_t0 * one_time + time * v0_t1;
      const vfloat16 v1 = v1_t0 * one_time + time * v1_t1;
      const vfloat16 v2 = v2_t0 * one_time + time * v2_t1;

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

      max_dist_xyz  = select(m_final,t,max_dist_xyz);
		    
      //////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(RTCORE_RAY_MASK)
      const vint16 rayMask(ray16.mask[rayIndex]);
      const vint16 triMask = getTriMasks(tptr); 
      const vbool16 m_ray_mask = (rayMask & triMask) != vint16::zero();
      m_final &= m_ray_mask;	      
#endif


      /* did the ray hot one of the four triangles? */
      if (unlikely(any(m_final)))
	{
	  const vfloat16 min_dist = vreduce_min(max_dist_xyz);
	  const vbool16 m_dist = eq(min_dist,max_dist_xyz);

	  const size_t vecIndex = bitscan(toInt(m_dist));
	  const size_t triIndex = vecIndex >> 2;

	  const BVH4mb::Triangle01  *__restrict__ tri_ptr = tptr + triIndex;

	  const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));
		  
	  const vfloat16 gnormalz = swAAAA(normal);
	  const vfloat16 gnormalx = swBBBB(normal);
	  const vfloat16 gnormaly = swCCCC(normal);

	  prefetch<PFHINT_L1EX>(&ray16.tfar);  
	  prefetch<PFHINT_L1EX>(&ray16.u);
	  prefetch<PFHINT_L1EX>(&ray16.v);
	  prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	  prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	  prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	  prefetch<PFHINT_L1EX>(&ray16.geomID);
	  prefetch<PFHINT_L1EX>(&ray16.primID);

	  max_dist_xyz = min_dist;
		  
	  compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
	  compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
	  compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
	  compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
	  compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
	  compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 

	  ray16.geomID[rayIndex] = tri_ptr->t0.geomID();
	  ray16.primID[rayIndex] = tri_ptr->t0.primID();
	  return true;
	}
      return false;
    }



    static __forceinline bool occluded(BVH4i::NodeRef curNode,
				       const size_t rayIndex, 
				       const vfloat16 &dir_xyz,
				       const vfloat16 &org_xyz,
				       const vfloat16 &min_dist_xyz,
				       const vfloat16 &max_dist_xyz,
				       const Ray16& ray16, 
				       vbool16 &m_terminated,
				       const void *__restrict__ const accel,
				       const Scene*__restrict__ const geometry)
    {
      const vfloat16 time     = vfloat16::broadcast(&ray16.time[rayIndex]);
      const vfloat16 one_time = (vfloat16::one() - time);

      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);

      prefetch<PFHINT_L1>((vfloat16*)tptr +  0); 
      prefetch<PFHINT_L1>((vfloat16*)tptr +  1); 
      prefetch<PFHINT_L1>((vfloat16*)tptr +  2); 
      prefetch<PFHINT_L1>((vfloat16*)tptr +  3); 

      const vint16 and_mask = broadcast4to16i(zlc4);
	      
      const vfloat16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const vfloat16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const vfloat16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);


      prefetch<PFHINT_L2>((vfloat16*)tptr +  4); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  5); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  6); 
      prefetch<PFHINT_L2>((vfloat16*)tptr +  7); 

      const vfloat16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const vfloat16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const vfloat16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);

      const vfloat16 v0 = v0_t0 * one_time + time * v0_t1;
      const vfloat16 v1 = v1_t0 * one_time + time * v1_t1;
      const vfloat16 v2 = v2_t0 * one_time + time * v2_t1;

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

      if (unlikely(any(m_final)))
	{
	  m_terminated |= vbool16::shift1[rayIndex];
	  return true;
	}
      return false;
    }

  
      // ========================
      // ==== 16-wide packets ===
      // ========================

      __forceinline static void intersect16(BVH4i::NodeRef curNode,
					    const vbool16 m_valid_leaf, 
					    const Vec3vf16 &dir,
					    const Vec3vf16 &org,
					    Ray16& ray16, 
					    const void *__restrict__ const accel,
					    const Scene     *__restrict__ const geometry)
      {

	const vfloat16 time     = ray16.time;
	const vfloat16 one_time = (vfloat16::one() - time);

	unsigned int items; 
	const BVH4mb::Triangle01* tris  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel,items);

	const vfloat16 zero = vfloat16::zero();
	const vfloat16 one  = vfloat16::one();

	prefetch<PFHINT_L1>((vfloat16*)tris +  0); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  1); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  2); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  3); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  4); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  5); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  6); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  7); 

	for (size_t i=0; i<items; i++) 
	  {
	    const Triangle1& tri_t0 = tris[i].t0;
	    const Triangle1& tri_t1 = tris[i].t1;

	    prefetch<PFHINT_L1>(&tris[i+1].t0); 
	    prefetch<PFHINT_L1>(&tris[i+1].t1); 

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const Vec3vf16 v0_t0( vfloat16::broadcast(&tri_t0.v0.x), vfloat16::broadcast(&tri_t0.v0.y), vfloat16::broadcast(&tri_t0.v0.z) );
	    const Vec3vf16 v0_t1( vfloat16::broadcast(&tri_t1.v0.x), vfloat16::broadcast(&tri_t1.v0.y), vfloat16::broadcast(&tri_t1.v0.z) );
	    const Vec3vf16 v0 = v0_t0 * one_time + time * v0_t1;
	    const Vec3vf16 v1_t0( vfloat16::broadcast(&tri_t0.v1.x), vfloat16::broadcast(&tri_t0.v1.y), vfloat16::broadcast(&tri_t0.v1.z) );
	    const Vec3vf16 v1_t1( vfloat16::broadcast(&tri_t1.v1.x), vfloat16::broadcast(&tri_t1.v1.y), vfloat16::broadcast(&tri_t1.v1.z) );
	    const Vec3vf16 v1 = v1_t0 * one_time + time * v1_t1;
	    const Vec3vf16 v2_t0( vfloat16::broadcast(&tri_t0.v2.x), vfloat16::broadcast(&tri_t0.v2.y), vfloat16::broadcast(&tri_t0.v2.z) );
	    const Vec3vf16 v2_t1( vfloat16::broadcast(&tri_t1.v2.x), vfloat16::broadcast(&tri_t1.v2.y), vfloat16::broadcast(&tri_t1.v2.z) );
	    const Vec3vf16 v2 = v2_t0 * one_time + time * v2_t1;

	    const Vec3vf16 e1 = v0-v1;
	    const Vec3vf16 e2 = v2-v0;

	    const Vec3vf16 Ng = cross(e1,e2);

	    /* calculate denominator */
	    const Vec3vf16 C =  v0 - org;
	    
	    const vfloat16 den = dot(Ng,dir);

	    vbool16 valid = m_valid_leaf;

#if defined(RTCORE_BACKFACE_CULLING)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const vfloat16 rcp_den = rcp(den);
	    const Vec3vf16 R = cross(dir,C);
	    const vfloat16 u = dot(R,e2)*rcp_den;
	    const vfloat16 v = dot(R,e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);
	    prefetch<PFHINT_L1EX>(&ray16.u);      
	    prefetch<PFHINT_L1EX>(&ray16.v);      
	    prefetch<PFHINT_L1EX>(&ray16.tfar);      
	    const vfloat16 t = dot(C,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    const vint16 geomID = tri_t0.geomID();
	    const vint16 primID = tri_t0.primID();
	    prefetch<PFHINT_L1EX>(&ray16.geomID);      
	    prefetch<PFHINT_L1EX>(&ray16.primID);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.x);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.y);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.z);      

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (vint16(tri_t0.mask()) & ray16.mask) != 0;
#endif
	    if (unlikely(none(valid))) continue;
        
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

      __forceinline static void occluded16(BVH4i::NodeRef curNode,
					   const vbool16 m_valid_leaf_active, 
					   const Vec3vf16 &dir,
					   const Vec3vf16 &org,
					   Ray16& ray16, 
					   vbool16 &m_terminated,					    
					   const void *__restrict__ const accel,
					   const Scene     *__restrict__ const geometry)
      {
	vbool16 m_valid_leaf = m_valid_leaf_active;

	const vfloat16 time     = ray16.time;
	const vfloat16 one_time = (vfloat16::one() - time);

	unsigned int items; 
	const BVH4mb::Triangle01* tris  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel,items);

	prefetch<PFHINT_L1>((vfloat16*)tris +  0); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  1); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  2); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  3); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  4); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  5); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  6); 
	prefetch<PFHINT_L2>((vfloat16*)tris +  7); 

	const vfloat16 zero = vfloat16::zero();

	for (size_t i=0; i<items; i++) 
	  {
	    const Triangle1& tri_t0 = tris[i].t0;
	    const Triangle1& tri_t1 = tris[i].t1;

	    prefetch<PFHINT_L1>(&tris[i+1].t0); 
	    prefetch<PFHINT_L1>(&tris[i+1].t1); 

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf_active),16);
        
	    /* load vertices and calculate edges */
	    const Vec3vf16 v0_t0( vfloat16::broadcast(&tri_t0.v0.x), vfloat16::broadcast(&tri_t0.v0.y), vfloat16::broadcast(&tri_t0.v0.z) );
	    const Vec3vf16 v0_t1( vfloat16::broadcast(&tri_t1.v0.x), vfloat16::broadcast(&tri_t1.v0.y), vfloat16::broadcast(&tri_t1.v0.z) );
	    const Vec3vf16 v0 = v0_t0 * one_time + time * v0_t1;
	    const Vec3vf16 v1_t0( vfloat16::broadcast(&tri_t0.v1.x), vfloat16::broadcast(&tri_t0.v1.y), vfloat16::broadcast(&tri_t0.v1.z) );
	    const Vec3vf16 v1_t1( vfloat16::broadcast(&tri_t1.v1.x), vfloat16::broadcast(&tri_t1.v1.y), vfloat16::broadcast(&tri_t1.v1.z) );
	    const Vec3vf16 v1 = v1_t0 * one_time + time * v1_t1;
	    const Vec3vf16 v2_t0( vfloat16::broadcast(&tri_t0.v2.x), vfloat16::broadcast(&tri_t0.v2.y), vfloat16::broadcast(&tri_t0.v2.z) );
	    const Vec3vf16 v2_t1( vfloat16::broadcast(&tri_t1.v2.x), vfloat16::broadcast(&tri_t1.v2.y), vfloat16::broadcast(&tri_t1.v2.z) );
	    const Vec3vf16 v2 = v2_t0 * one_time + time * v2_t1;

	    const Vec3vf16 e1 = v0-v1;
	    const Vec3vf16 e2 = v2-v0;

	    const Vec3vf16 Ng = cross(e1,e2);

	    /* calculate denominator */
	    const Vec3vf16 C =  v0 - org;
	    
	    const vfloat16 den = dot(Ng,dir);

	    vbool16 valid = m_valid_leaf;

#if defined(RTCORE_BACKFACE_CULLING)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const vfloat16 rcp_den = rcp(den);
	    const Vec3vf16 R = cross(dir,C);
	    const vfloat16 u = dot(R,e2)*rcp_den;
	    const vfloat16 v = dot(R,e1)*rcp_den;
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
	    valid &= (vint16(tri_t0.mask()) & ray16.mask) != 0;
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
