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
					const float16 &dir_xyz,
					const float16 &org_xyz,
					const float16 &min_dist_xyz,
					float16 &max_dist_xyz,
					Ray& ray, 
					const void *__restrict__ const accel,
					const Scene*__restrict__ const geometry)
    {
      const float16 time     = broadcast1to16f(&ray.time);
      const float16 one_time = (float16::one() - time);

      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);
      prefetch<PFHINT_L2>((float16*)tptr +  0); 
      prefetch<PFHINT_L2>((float16*)tptr +  1); 
      prefetch<PFHINT_L2>((float16*)tptr +  2); 
      prefetch<PFHINT_L2>((float16*)tptr +  3); 
      prefetch<PFHINT_L2>((float16*)tptr +  4); 
      prefetch<PFHINT_L2>((float16*)tptr +  5); 
      prefetch<PFHINT_L2>((float16*)tptr +  6); 
      prefetch<PFHINT_L2>((float16*)tptr +  7); 

      const int16 and_mask = broadcast4to16i(zlc4);
	      
      const float16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const float16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const float16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);

      const float16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const float16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const float16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);

      const float16 v0 = v0_t0 * one_time + time * v0_t1;
      const float16 v1 = v1_t0 * one_time + time * v1_t1;
      const float16 v2 = v2_t0 * one_time + time * v2_t1;

      const float16 e1 = v1 - v0;
      const float16 e2 = v0 - v2;	     
      const float16 normal = lcross_zxy(e1,e2);
      const float16 org = v0 - org_xyz;
      const float16 odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
      const float16 den = ldot3_zxy(dir_xyz,normal);	      
      const float16 rcp_den = rcp(den);
      const float16 uu = ldot3_zxy(e2,odzxy); 
      const float16 vv = ldot3_zxy(e1,odzxy); 
      const float16 u = uu * rcp_den;
      const float16 v = vv * rcp_den;
#if defined(RTCORE_BACKFACE_CULLING)
      const bool16 m_init = (bool16)0x1111 & (den > zero);
#else
      const bool16 m_init = 0x1111;
#endif
      const bool16 valid_u = ge(m_init,u,zero);
      const bool16 valid_v = ge(valid_u,v,zero);
      const bool16 m_aperture = le(valid_v,u+v,float16::one()); 

      const float16 nom = ldot3_zxy(org,normal);

      if (unlikely(none(m_aperture))) return false;

      const float16 t = rcp_den*nom;
      bool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

      max_dist_xyz  = select(m_final,t,max_dist_xyz);
		    
      //////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(RTCORE_RAY_MASK)
      const int16 rayMask(ray.mask);
      const int16 triMask = getTriMasks(tptr); 
      const bool16 m_ray_mask = (rayMask & triMask) != int16::zero();
      m_final &= m_ray_mask;	      
#endif


      /* did the ray hot one of the four triangles? */
      if (unlikely(any(m_final)))
	{
	  const float16 min_dist = vreduce_min(max_dist_xyz);
	  const bool16 m_dist = eq(min_dist,max_dist_xyz);

	  prefetch<PFHINT_L1EX>((float16*)&ray + 0);
	  prefetch<PFHINT_L1EX>((float16*)&ray + 1);

	  const size_t vecIndex = bitscan(toInt(m_dist));
	  const size_t triIndex = vecIndex >> 2;

	  const BVH4mb::Triangle01  *__restrict__ tri_ptr = tptr + triIndex;

	  const bool16 m_tri = m_dist^(m_dist & (bool16)((unsigned int)m_dist - 1));

	  const float16 gnormalz = swAAAA(normal);
	  const float16 gnormalx = swBBBB(normal);
	  const float16 gnormaly = swCCCC(normal);
		  
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
					 const float16 &dir_xyz,
					 const float16 &org_xyz,
					 const float16 &min_dist_xyz,
					 const float16 &max_dist_xyz,
					 Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
    {
      const float16 time     = broadcast1to16f(&ray.time);
      const float16 one_time = (float16::one() - time);
      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);

      prefetch<PFHINT_L2>((float16*)tptr +  0); 
      prefetch<PFHINT_L2>((float16*)tptr +  1); 
      prefetch<PFHINT_L2>((float16*)tptr +  2); 
      prefetch<PFHINT_L2>((float16*)tptr +  3); 
      prefetch<PFHINT_L2>((float16*)tptr +  4); 
      prefetch<PFHINT_L2>((float16*)tptr +  5); 
      prefetch<PFHINT_L2>((float16*)tptr +  6); 
      prefetch<PFHINT_L2>((float16*)tptr +  7); 

      const int16 and_mask = broadcast4to16i(zlc4);
	      
      const float16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const float16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const float16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);

      const float16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const float16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const float16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);


      const float16 v0 = v0_t0 * one_time + time * v0_t1;
      const float16 v1 = v1_t0 * one_time + time * v1_t1;
      const float16 v2 = v2_t0 * one_time + time * v2_t1;

      const float16 e1 = v1 - v0;
      const float16 e2 = v0 - v2;	     
      const float16 normal = lcross_zxy(e1,e2);
      const float16 org = v0 - org_xyz;
      const float16 odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
      const float16 den = ldot3_zxy(dir_xyz,normal);	      
      const float16 rcp_den = rcp(den);
      const float16 uu = ldot3_zxy(e2,odzxy); 
      const float16 vv = ldot3_zxy(e1,odzxy); 
      const float16 u = uu * rcp_den;
      const float16 v = vv * rcp_den;

#if defined(RTCORE_BACKFACE_CULLING)
      const bool16 m_init = (bool16)0x1111 & (den > zero);
#else
      const bool16 m_init = 0x1111;
#endif
      const bool16 valid_u = ge(m_init,u,zero);
      const bool16 valid_v = ge(valid_u,v,zero);
      const bool16 m_aperture = le(valid_v,u+v,float16::one()); 

      const float16 nom = ldot3_zxy(org,normal);
      const float16 t = rcp_den*nom;

      if (unlikely(none(m_aperture))) return false;

      bool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(RTCORE_RAY_MASK)
      const int16 rayMask(ray.mask);
      const int16 triMask = getTriMasks(tptr); 
      const bool16 m_ray_mask = (rayMask & triMask) != int16::zero();
      m_final &= m_ray_mask;	      
#endif

      return any(m_final);
    }


    // ============================================
    // ==== single ray mode for 16-wide packets ===
    // ============================================
    static __forceinline bool intersect(BVH4i::NodeRef curNode,
					const size_t rayIndex, 
					const float16 &dir_xyz,
					const float16 &org_xyz,
					const float16 &min_dist_xyz,
					float16 &max_dist_xyz,
					Ray16& ray16, 
					const void *__restrict__ const accel,
					const Scene*__restrict__ const geometry)
    {
      const float16 time     = broadcast1to16f(&ray16.time[rayIndex]);
      const float16 one_time = (float16::one() - time);

      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);
	      
      prefetch<PFHINT_L1>((float16*)tptr +  0); 
      prefetch<PFHINT_L1>((float16*)tptr +  1); 
      prefetch<PFHINT_L1>((float16*)tptr +  2); 
      prefetch<PFHINT_L1>((float16*)tptr +  3); 
      prefetch<PFHINT_L2>((float16*)tptr +  4); 
      prefetch<PFHINT_L2>((float16*)tptr +  5); 
      prefetch<PFHINT_L2>((float16*)tptr +  6); 
      prefetch<PFHINT_L2>((float16*)tptr +  7); 

      const int16 and_mask = broadcast4to16i(zlc4);
	     

      const float16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const float16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const float16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);

      const float16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const float16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const float16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);

      const float16 v0 = v0_t0 * one_time + time * v0_t1;
      const float16 v1 = v1_t0 * one_time + time * v1_t1;
      const float16 v2 = v2_t0 * one_time + time * v2_t1;

      const float16 e1 = v1 - v0;
      const float16 e2 = v0 - v2;	     
      const float16 normal = lcross_zxy(e1,e2);
      const float16 org = v0 - org_xyz;
      const float16 odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
      const float16 den = ldot3_zxy(dir_xyz,normal);	      
      const float16 rcp_den = rcp(den);
      const float16 uu = ldot3_zxy(e2,odzxy); 
      const float16 vv = ldot3_zxy(e1,odzxy); 
      const float16 u = uu * rcp_den;
      const float16 v = vv * rcp_den;

#if defined(RTCORE_BACKFACE_CULLING)
      const bool16 m_init = (bool16)0x1111 & (den > zero);
#else
      const bool16 m_init = 0x1111;
#endif

      const bool16 valid_u = ge(m_init,u,zero);
      const bool16 valid_v = ge(valid_u,v,zero);
      const bool16 m_aperture = le(valid_v,u+v,float16::one()); 

      const float16 nom = ldot3_zxy(org,normal);

      if (unlikely(none(m_aperture))) return false;
      const float16 t = rcp_den*nom;

      bool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

      max_dist_xyz  = select(m_final,t,max_dist_xyz);
		    
      //////////////////////////////////////////////////////////////////////////////////////////////////

#if defined(RTCORE_RAY_MASK)
      const int16 rayMask(ray16.mask[rayIndex]);
      const int16 triMask = getTriMasks(tptr); 
      const bool16 m_ray_mask = (rayMask & triMask) != int16::zero();
      m_final &= m_ray_mask;	      
#endif


      /* did the ray hot one of the four triangles? */
      if (unlikely(any(m_final)))
	{
	  const float16 min_dist = vreduce_min(max_dist_xyz);
	  const bool16 m_dist = eq(min_dist,max_dist_xyz);

	  const size_t vecIndex = bitscan(toInt(m_dist));
	  const size_t triIndex = vecIndex >> 2;

	  const BVH4mb::Triangle01  *__restrict__ tri_ptr = tptr + triIndex;

	  const bool16 m_tri = m_dist^(m_dist & (bool16)((unsigned int)m_dist - 1));
		  
	  const float16 gnormalz = swAAAA(normal);
	  const float16 gnormalx = swBBBB(normal);
	  const float16 gnormaly = swCCCC(normal);

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
				       const float16 &dir_xyz,
				       const float16 &org_xyz,
				       const float16 &min_dist_xyz,
				       const float16 &max_dist_xyz,
				       const Ray16& ray16, 
				       bool16 &m_terminated,
				       const void *__restrict__ const accel,
				       const Scene*__restrict__ const geometry)
    {
      const float16 time     = broadcast1to16f(&ray16.time[rayIndex]);
      const float16 one_time = (float16::one() - time);

      const BVH4mb::Triangle01* tptr  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel);

      prefetch<PFHINT_L1>((float16*)tptr +  0); 
      prefetch<PFHINT_L1>((float16*)tptr +  1); 
      prefetch<PFHINT_L1>((float16*)tptr +  2); 
      prefetch<PFHINT_L1>((float16*)tptr +  3); 

      const int16 and_mask = broadcast4to16i(zlc4);
	      
      const float16 v0_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v0,
					(float*)&tptr[1].t0.v0,
					(float*)&tptr[2].t0.v0,
					(float*)&tptr[3].t0.v0);
	      
      const float16 v1_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v1,
					(float*)&tptr[1].t0.v1,
					(float*)&tptr[2].t0.v1,
					(float*)&tptr[3].t0.v1);
	      
      const float16 v2_t0 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t0.v2,
					(float*)&tptr[1].t0.v2,
					(float*)&tptr[2].t0.v2,
					(float*)&tptr[3].t0.v2);


      prefetch<PFHINT_L2>((float16*)tptr +  4); 
      prefetch<PFHINT_L2>((float16*)tptr +  5); 
      prefetch<PFHINT_L2>((float16*)tptr +  6); 
      prefetch<PFHINT_L2>((float16*)tptr +  7); 

      const float16 v0_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v0,
					(float*)&tptr[1].t1.v0,
					(float*)&tptr[2].t1.v0,
					(float*)&tptr[3].t1.v0);
	      
      const float16 v1_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v1,
					(float*)&tptr[1].t1.v1,
					(float*)&tptr[2].t1.v1,
					(float*)&tptr[3].t1.v1);
	      
      const float16 v2_t1 = gather_4f_zlc(and_mask,
					(float*)&tptr[0].t1.v2,
					(float*)&tptr[1].t1.v2,
					(float*)&tptr[2].t1.v2,
					(float*)&tptr[3].t1.v2);

      const float16 v0 = v0_t0 * one_time + time * v0_t1;
      const float16 v1 = v1_t0 * one_time + time * v1_t1;
      const float16 v2 = v2_t0 * one_time + time * v2_t1;

      const float16 e1 = v1 - v0;
      const float16 e2 = v0 - v2;	     
      const float16 normal = lcross_zxy(e1,e2);
      const float16 org = v0 - org_xyz;
      const float16 odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
      const float16 den = ldot3_zxy(dir_xyz,normal);	      
      const float16 rcp_den = rcp(den);
      const float16 uu = ldot3_zxy(e2,odzxy); 
      const float16 vv = ldot3_zxy(e1,odzxy); 
      const float16 u = uu * rcp_den;
      const float16 v = vv * rcp_den;

#if defined(RTCORE_BACKFACE_CULLING)
      const bool16 m_init = (bool16)0x1111 & (den > zero);
#else
      const bool16 m_init = 0x1111;
#endif

      const bool16 valid_u = ge((bool16)m_init,u,zero);
      const bool16 valid_v = ge(valid_u,v,zero);
      const bool16 m_aperture = le(valid_v,u+v,float16::one()); 

      const float16 nom = ldot3_zxy(org,normal);
      const float16 t = rcp_den*nom;
      if (unlikely(none(m_aperture))) return false;

      bool16 m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(RTCORE_RAY_MASK)
      const int16 rayMask(ray16.mask[rayIndex]);
      const int16 triMask = getTriMasks(tptr); 
      const bool16 m_ray_mask = (rayMask & triMask) != int16::zero();
      m_final &= m_ray_mask;	      
#endif

      if (unlikely(any(m_final)))
	{
	  m_terminated |= bool16::shift1[rayIndex];
	  return true;
	}
      return false;
    }

  
      // ========================
      // ==== 16-wide packets ===
      // ========================

      __forceinline static void intersect16(BVH4i::NodeRef curNode,
					    const bool16 m_valid_leaf, 
					    const Vec3f16 &dir,
					    const Vec3f16 &org,
					    Ray16& ray16, 
					    const void *__restrict__ const accel,
					    const Scene     *__restrict__ const geometry)
      {

	const float16 time     = ray16.time;
	const float16 one_time = (float16::one() - time);

	unsigned int items; 
	const BVH4mb::Triangle01* tris  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel,items);

	const float16 zero = float16::zero();
	const float16 one  = float16::one();

	prefetch<PFHINT_L1>((float16*)tris +  0); 
	prefetch<PFHINT_L2>((float16*)tris +  1); 
	prefetch<PFHINT_L2>((float16*)tris +  2); 
	prefetch<PFHINT_L2>((float16*)tris +  3); 
	prefetch<PFHINT_L2>((float16*)tris +  4); 
	prefetch<PFHINT_L2>((float16*)tris +  5); 
	prefetch<PFHINT_L2>((float16*)tris +  6); 
	prefetch<PFHINT_L2>((float16*)tris +  7); 

	for (size_t i=0; i<items; i++) 
	  {
	    const Triangle1& tri_t0 = tris[i].t0;
	    const Triangle1& tri_t1 = tris[i].t1;

	    prefetch<PFHINT_L1>(&tris[i+1].t0); 
	    prefetch<PFHINT_L1>(&tris[i+1].t1); 

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf),16);
        
	    /* load vertices and calculate edges */
	    const Vec3f16 v0_t0( broadcast1to16f(&tri_t0.v0.x), broadcast1to16f(&tri_t0.v0.y), broadcast1to16f(&tri_t0.v0.z) );
	    const Vec3f16 v0_t1( broadcast1to16f(&tri_t1.v0.x), broadcast1to16f(&tri_t1.v0.y), broadcast1to16f(&tri_t1.v0.z) );
	    const Vec3f16 v0 = v0_t0 * one_time + time * v0_t1;
	    const Vec3f16 v1_t0( broadcast1to16f(&tri_t0.v1.x), broadcast1to16f(&tri_t0.v1.y), broadcast1to16f(&tri_t0.v1.z) );
	    const Vec3f16 v1_t1( broadcast1to16f(&tri_t1.v1.x), broadcast1to16f(&tri_t1.v1.y), broadcast1to16f(&tri_t1.v1.z) );
	    const Vec3f16 v1 = v1_t0 * one_time + time * v1_t1;
	    const Vec3f16 v2_t0( broadcast1to16f(&tri_t0.v2.x), broadcast1to16f(&tri_t0.v2.y), broadcast1to16f(&tri_t0.v2.z) );
	    const Vec3f16 v2_t1( broadcast1to16f(&tri_t1.v2.x), broadcast1to16f(&tri_t1.v2.y), broadcast1to16f(&tri_t1.v2.z) );
	    const Vec3f16 v2 = v2_t0 * one_time + time * v2_t1;

	    const Vec3f16 e1 = v0-v1;
	    const Vec3f16 e2 = v2-v0;

	    const Vec3f16 Ng = cross(e1,e2);

	    /* calculate denominator */
	    const Vec3f16 C =  v0 - org;
	    
	    const float16 den = dot(Ng,dir);

	    bool16 valid = m_valid_leaf;

#if defined(RTCORE_BACKFACE_CULLING)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const float16 rcp_den = rcp(den);
	    const Vec3f16 R = cross(dir,C);
	    const float16 u = dot(R,e2)*rcp_den;
	    const float16 v = dot(R,e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);
	    prefetch<PFHINT_L1EX>(&ray16.u);      
	    prefetch<PFHINT_L1EX>(&ray16.v);      
	    prefetch<PFHINT_L1EX>(&ray16.tfar);      
	    const float16 t = dot(C,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    const int16 geomID = tri_t0.geomID();
	    const int16 primID = tri_t0.primID();
	    prefetch<PFHINT_L1EX>(&ray16.geomID);      
	    prefetch<PFHINT_L1EX>(&ray16.primID);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.x);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.y);      
	    prefetch<PFHINT_L1EX>(&ray16.Ng.z);      

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (int16(tri_t0.mask()) & ray16.mask) != 0;
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
					   const bool16 m_valid_leaf_active, 
					   const Vec3f16 &dir,
					   const Vec3f16 &org,
					   Ray16& ray16, 
					   bool16 &m_terminated,					    
					   const void *__restrict__ const accel,
					   const Scene     *__restrict__ const geometry)
      {
	bool16 m_valid_leaf = m_valid_leaf_active;

	const float16 time     = ray16.time;
	const float16 one_time = (float16::one() - time);

	unsigned int items; 
	const BVH4mb::Triangle01* tris  = (BVH4mb::Triangle01*) curNode.leaf<8>(accel,items);

	prefetch<PFHINT_L1>((float16*)tris +  0); 
	prefetch<PFHINT_L2>((float16*)tris +  1); 
	prefetch<PFHINT_L2>((float16*)tris +  2); 
	prefetch<PFHINT_L2>((float16*)tris +  3); 
	prefetch<PFHINT_L2>((float16*)tris +  4); 
	prefetch<PFHINT_L2>((float16*)tris +  5); 
	prefetch<PFHINT_L2>((float16*)tris +  6); 
	prefetch<PFHINT_L2>((float16*)tris +  7); 

	const float16 zero = float16::zero();

	for (size_t i=0; i<items; i++) 
	  {
	    const Triangle1& tri_t0 = tris[i].t0;
	    const Triangle1& tri_t1 = tris[i].t1;

	    prefetch<PFHINT_L1>(&tris[i+1].t0); 
	    prefetch<PFHINT_L1>(&tris[i+1].t1); 

	    STAT3(normal.trav_prims,1,popcnt(m_valid_leaf_active),16);
        
	    /* load vertices and calculate edges */
	    const Vec3f16 v0_t0( broadcast1to16f(&tri_t0.v0.x), broadcast1to16f(&tri_t0.v0.y), broadcast1to16f(&tri_t0.v0.z) );
	    const Vec3f16 v0_t1( broadcast1to16f(&tri_t1.v0.x), broadcast1to16f(&tri_t1.v0.y), broadcast1to16f(&tri_t1.v0.z) );
	    const Vec3f16 v0 = v0_t0 * one_time + time * v0_t1;
	    const Vec3f16 v1_t0( broadcast1to16f(&tri_t0.v1.x), broadcast1to16f(&tri_t0.v1.y), broadcast1to16f(&tri_t0.v1.z) );
	    const Vec3f16 v1_t1( broadcast1to16f(&tri_t1.v1.x), broadcast1to16f(&tri_t1.v1.y), broadcast1to16f(&tri_t1.v1.z) );
	    const Vec3f16 v1 = v1_t0 * one_time + time * v1_t1;
	    const Vec3f16 v2_t0( broadcast1to16f(&tri_t0.v2.x), broadcast1to16f(&tri_t0.v2.y), broadcast1to16f(&tri_t0.v2.z) );
	    const Vec3f16 v2_t1( broadcast1to16f(&tri_t1.v2.x), broadcast1to16f(&tri_t1.v2.y), broadcast1to16f(&tri_t1.v2.z) );
	    const Vec3f16 v2 = v2_t0 * one_time + time * v2_t1;

	    const Vec3f16 e1 = v0-v1;
	    const Vec3f16 e2 = v2-v0;

	    const Vec3f16 Ng = cross(e1,e2);

	    /* calculate denominator */
	    const Vec3f16 C =  v0 - org;
	    
	    const float16 den = dot(Ng,dir);

	    bool16 valid = m_valid_leaf;

#if defined(RTCORE_BACKFACE_CULLING)
	    
	    valid &= den > zero;
#endif

	    /* perform edge tests */
	    const float16 rcp_den = rcp(den);
	    const Vec3f16 R = cross(dir,C);
	    const float16 u = dot(R,e2)*rcp_den;
	    const float16 v = dot(R,e1)*rcp_den;
	    valid = ge(valid,u,zero);
	    valid = ge(valid,v,zero);
	    valid = le(valid,u+v,one);
	    const float16 t = dot(C,Ng) * rcp_den;

	    if (unlikely(none(valid))) continue;
      
	    /* perform depth test */
	    valid = ge(valid, t,ray16.tnear);
	    valid = ge(valid,ray16.tfar,t);

	    /* ray masking test */
#if defined(RTCORE_RAY_MASK)
	    valid &= (int16(tri_t0.mask()) & ray16.mask) != 0;
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
