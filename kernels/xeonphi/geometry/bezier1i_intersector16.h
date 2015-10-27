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

#include "bezier1i.h"
#include "../../common/ray.h"
#include "filter.h"

namespace embree
{   
  /*! Intersector for a single ray from a ray packet with a bezier curve. */

  struct __aligned(64) Precalculations 
  {
    __forceinline Precalculations (const LinearSpace3vf16& ls16, const vfloat16 &rcp_length, const size_t k)
      : ray_space(ls16.vx.x[k],ls16.vy.x[k],ls16.vz.x[k],
		  ls16.vx.y[k],ls16.vy.y[k],ls16.vz.y[k],
		  ls16.vx.z[k],ls16.vy.z[k],ls16.vz.z[k]),
      inv_ray_length(rcp_length[k])
      {}
    __aligned(64) LinearSpace3fa ray_space;
    vfloat16 inv_ray_length;
  };

  template< bool ENABLE_INTERSECTION_FILTER>
  struct Bezier1iIntersector16
  {
    typedef Bezier1i Primitive;


    static __forceinline Vec4vf16 eval16(const vfloat16 &p0123,
				      const vfloat16 &c0,
				      const vfloat16 &c1,
				      const vfloat16 &c2,
				      const vfloat16 &c3)
    {
#if 1
      const vfloat16 p0 = shuffle4<0>(p0123);
      const vfloat16 p1 = shuffle4<1>(p0123);
      const vfloat16 p2 = shuffle4<2>(p0123);
      const vfloat16 p3 = shuffle4<3>(p0123);

      const vfloat16 x = c0 * swAAAA(p0) + c1 * swAAAA(p1) + c2 * swAAAA(p2) + c3 * swAAAA(p3);
      const vfloat16 y = c0 * swBBBB(p0) + c1 * swBBBB(p1) + c2 * swBBBB(p2) + c3 * swBBBB(p3);
      const vfloat16 z = c0 * swCCCC(p0) + c1 * swCCCC(p1) + c2 * swCCCC(p2) + c3 * swCCCC(p3);
      const vfloat16 w = c0 * swDDDD(p0) + c1 * swDDDD(p1) + c2 * swDDDD(p2) + c3 * swDDDD(p3);
#else
      const vfloat16 x = madd(c0,vfloat16(p0123[0]),madd(c1,vfloat16(p0123[4]),madd(c2,vfloat16(p0123[8]),c3* vfloat16(p0123[12]))));
      const vfloat16 y = madd(c0,vfloat16(p0123[1]),madd(c1,vfloat16(p0123[5]),madd(c2,vfloat16(p0123[9]),c3* vfloat16(p0123[13]))));
      const vfloat16 z = madd(c0,vfloat16(p0123[2]),madd(c1,vfloat16(p0123[6]),madd(c2,vfloat16(p0123[10]),c3* vfloat16(p0123[14]))));
      const vfloat16 w = madd(c0,vfloat16(p0123[3]),madd(c1,vfloat16(p0123[7]),madd(c2,vfloat16(p0123[11]),c3* vfloat16(p0123[15]))));
#endif
      return Vec4vf16(x,y,z,w);
    }

    static __forceinline void eval(const float t, const vfloat16 &p0123, vfloat16& point, vfloat16& tangent)
    {
      const vfloat16 t0 = vfloat16(1.0f) - vfloat16(t), t1 = vfloat16(t);

      const Vec3fa *__restrict__ const p = (Vec3fa*)&p0123;

      const vfloat16 p00 = broadcast4to16f((float*)&p[0]);
      const vfloat16 p01 = broadcast4to16f((float*)&p[1]);
      const vfloat16 p02 = broadcast4to16f((float*)&p[2]);
      const vfloat16 p03 = broadcast4to16f((float*)&p[3]);

      const vfloat16 p10 = p00 * t0 + p01 * t1;
      const vfloat16 p11 = p01 * t0 + p02 * t1;
      const vfloat16 p12 = p02 * t0 + p03 * t1;
      const vfloat16 p20 = p10 * t0 + p11 * t1;
      const vfloat16 p21 = p11 * t0 + p12 * t1;
      const vfloat16 p30 = p20 * t0 + p21 * t1;

      point = p30;
      tangent = p21-p20;
    }

    static __forceinline bool intersect(const vfloat16& pre_vx, 
					const vfloat16& pre_vy, 
					const vfloat16& pre_vz, 					
					const vfloat16& inv_ray_length,
					Ray16& ray, 
					const vfloat16 &dir_xyz,
					const vfloat16 &org_xyz,
					const size_t k, 
					const Bezier1i& curve_in, 
					const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

      const vfloat16 c0 = vfloat16::load((float*)&coeff01[0]);
      const vfloat16 c1 = vfloat16::load((float*)&coeff01[1]);
      const vfloat16 c2 = vfloat16::load((float*)&coeff01[2]);
      const vfloat16 c3 = vfloat16::load((float*)&coeff01[3]);

      const vfloat16 zero = vfloat16::zero();
      const vfloat16 one  = vfloat16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);


      const vfloat16 p0123 = vfloat16::loadu((float*)curve_in.p);

      const vfloat16 p0123_org = p0123 - org_xyz;

      const vfloat16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);



      const Vec4vf16 p0 = eval16(p0123_2D,c0,c1,c2,c3);
      
      const vfloat16 last_x = vfloat16(p0123_2D[12 + 0]);
      const vfloat16 last_y = vfloat16(p0123_2D[13 + 0]);
      const vfloat16 last_z = vfloat16(p0123_2D[14 + 0]);
      const vfloat16 last_w = vfloat16(p0123_2D[15 + 0]);

      const Vec4vf16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));


      /* approximative intersection with cone */
      const Vec4vf16 v = p1-p0;
      const Vec4vf16 w = -p0;
      const vfloat16 d0 = w.x*v.x + w.y*v.y;
      const vfloat16 d1 = v.x*v.x + v.y*v.y;
      const vfloat16 u = clamp(d0*rcp_nr(d1),zero,one);
      const Vec4vf16 p = p0 + u*v;
      const vfloat16 t = p.z * inv_ray_length;
      const vfloat16 d2 = p.x*p.x + p.y*p.y; 
      const vfloat16 r = p.w;
      const vfloat16 r2 = r*r;

      vbool16 valid = le(d2,r2);
      valid = lt(valid,vfloat16(ray.tnear[k]),t);
      valid = lt(valid,t,vfloat16(ray.tfar[k]));

      if (unlikely(none(valid))) return false;
      STAT3(normal.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(RTCORE_RAY_MASK)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(curve_in.geomID);
      if (unlikely(g->mask & ray.mask[k]) == 0) return false;
#endif  

      /* update hit information */
      if (ENABLE_INTERSECTION_FILTER) 
	{
	  const Geometry* const gg = ((Scene*)geom)->get(curve_in.geomID);
	  if (unlikely(gg->hasIntersectionFilter<vfloat16>())) 
	    {
	      while(any(valid)) 
		{
		  const float one_over_width = 1.0f/16.0f;
		  unsigned int i = select_min(valid,t);
		  float uu = (float(i)+u[i])*one_over_width; 
		  vfloat16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != vfloat16::zero() );

		  if (runIntersectionFilter16(gg,ray,k,vfloat16(uu),vfloat16(0.0f),vfloat16(t[i]),vfloat16(T[0]),vfloat16(T[1]),vfloat16(T[2]),(vbool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
		    break;
		  valid ^= (unsigned int)1 << i;
		}
	      if (unlikely(none(valid))) return false;
	    }
	}

      const float one_over_width = 1.0f/16.0f;
      unsigned int i = select_min(valid,t);
      float uu = (float(i)+u[i])*one_over_width; 
      vfloat16 P,T;
      eval(uu,p0123,P,T);
      assert( T != vfloat16::zero() );

      ray.update(1,k,vfloat16(t[i]),vfloat16(uu),vfloat16::zero(),swAAAA(T),swBBBB(T),swCCCC(T),curve_in.geomID,curve_in.primID);
      return true;
    }

    static __forceinline bool occluded(const vfloat16& pre_vx, 
				       const vfloat16& pre_vy, 
				       const vfloat16& pre_vz, 	
				       const vfloat16& inv_ray_length,				
				       const Ray16& ray, 
				       const vfloat16 &dir_xyz,
				       const vfloat16 &org_xyz,
				       const size_t k, 
				       const Bezier1i& curve_in, 
				       const void* geom) 
    {
      STAT3(shadow.trav_prims,1,1,1);
      const vfloat16 zero = vfloat16::zero();
      const vfloat16 one  = vfloat16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);

      const vfloat16 p0123 = vfloat16::loadu((float*)curve_in.p);
      const vfloat16 p0123_org = p0123 - org_xyz;

      const vfloat16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);


      const vfloat16 c0 = vfloat16::load((float*)&coeff01[0]);
      const vfloat16 c1 = vfloat16::load((float*)&coeff01[1]);
      const vfloat16 c2 = vfloat16::load((float*)&coeff01[2]);
      const vfloat16 c3 = vfloat16::load((float*)&coeff01[3]);

      const Vec4vf16 p0 = eval16(p0123_2D,c0,c1,c2,c3);

      const vfloat16 last_x = vfloat16(p0123_2D[12 + 0]);
      const vfloat16 last_y = vfloat16(p0123_2D[13 + 0]);
      const vfloat16 last_z = vfloat16(p0123_2D[14 + 0]);
      const vfloat16 last_w = vfloat16(p0123_2D[15 + 0]);

      const Vec4vf16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));


      const float one_over_width = 1.0f/16.0f;


      /* approximative intersection with cone */
      const Vec4vf16 v = p1-p0;
      const Vec4vf16 w = -p0;
      const vfloat16 d0 = w.x*v.x + w.y*v.y;
      const vfloat16 d1 = v.x*v.x + v.y*v.y;
      const vfloat16 u = clamp(d0*rcp(d1),zero,one);
      const Vec4vf16 p = p0 + u*v;
      const vfloat16 t = p.z * inv_ray_length;
      const vfloat16 d2 = p.x*p.x + p.y*p.y; 
      const vfloat16 r = p.w;
      const vfloat16 r2 = r*r;
      vbool16 valid = le(d2,r2);
      valid = lt(valid,vfloat16(ray.tnear[k]),t);
      valid = lt(valid,t,vfloat16(ray.tfar[k]));


      if (unlikely(none(valid))) return false;

      STAT3(shadow.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(RTCORE_RAY_MASK)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(curve_in.geomID);
      if (unlikely(g->mask & ray.mask[k]) == 0) return false;
#endif  


      if (ENABLE_INTERSECTION_FILTER) 
	{
	  const Geometry* const gg = ((Scene*)geom)->get(curve_in.geomID);
	  if (likely(gg->hasOcclusionFilter<vfloat16>())) 
	    {
	      while(any(valid)) 
		{
		  unsigned int i = select_min(valid,t);
		  float uu = (float(i)+u[i])*one_over_width; 
		  vfloat16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != vfloat16::zero() );

		  if (runOcclusionFilter16(gg,(Ray16&)ray,k,vfloat16(uu),vfloat16(0.0f),vfloat16(t[i]),vfloat16(T[0]),vfloat16(T[1]),vfloat16(T[2]),(vbool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
		    return true;
		  valid ^= (unsigned int)1 << i;
		}
	    }
	  return false;
	}
      return true;
    }


    // ==================================================================
    // ==================================================================
    // ==================================================================


    static __forceinline bool intersect(const vfloat16& pre_vx, 
					const vfloat16& pre_vy, 
					const vfloat16& pre_vz, 					
					const vfloat16& inv_ray_length,
					Ray& ray, 
					const vfloat16 &dir_xyz,
					const vfloat16 &org_xyz,
					const Bezier1i& curve_in, 
					const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

      const vfloat16 zero = vfloat16::zero();
      const vfloat16 one  = vfloat16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);

      const vfloat16 p0123 = vfloat16::loadu((float*)curve_in.p);
      const vfloat16 p0123_org = p0123 - org_xyz;

      const vfloat16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);


      const vfloat16 c0 = vfloat16::load((float*)&coeff01[0]);
      const vfloat16 c1 = vfloat16::load((float*)&coeff01[1]);
      const vfloat16 c2 = vfloat16::load((float*)&coeff01[2]);
      const vfloat16 c3 = vfloat16::load((float*)&coeff01[3]);

      const Vec4vf16 p0 = eval16(p0123_2D,c0,c1,c2,c3);

      const vfloat16 last_x = vfloat16(p0123_2D[12 + 0]);
      const vfloat16 last_y = vfloat16(p0123_2D[13 + 0]);
      const vfloat16 last_z = vfloat16(p0123_2D[14 + 0]);
      const vfloat16 last_w = vfloat16(p0123_2D[15 + 0]);

      const Vec4vf16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));

      const float one_over_width = 1.0f/16.0f;


      /* approximative intersection with cone */
      const Vec4vf16 v = p1-p0;
      const Vec4vf16 w = -p0;
      const vfloat16 d0 = w.x*v.x + w.y*v.y;
      const vfloat16 d1 = v.x*v.x + v.y*v.y;
      const vfloat16 u = clamp(d0*rcp(d1),zero,one);
      const Vec4vf16 p = p0 + u*v;
      const vfloat16 t = p.z * inv_ray_length;
      const vfloat16 d2 = p.x*p.x + p.y*p.y; 
      const vfloat16 r = p.w;
      const vfloat16 r2 = r*r;
      vbool16 valid = le(d2,r2);
      valid = lt(valid,vfloat16(ray.tnear),t);
      valid = lt(valid,t,vfloat16(ray.tfar));


      if (unlikely(none(valid))) return false;
      STAT3(normal.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(RTCORE_RAY_MASK)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(curve_in.geomID);
      if (unlikely(g->mask & ray.mask) == 0) return false;
#endif  

      if (ENABLE_INTERSECTION_FILTER) 
	{
	  const Geometry* const gg = ((Scene*)geom)->get(curve_in.geomID);
	  if (unlikely(gg->hasIntersectionFilter1())) 
	    {
	      while(any(valid)) 
		{
		  unsigned int i = select_min(valid,t);
		  float uu = (float(i)+u[i])*one_over_width; 
		  vfloat16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != vfloat16::zero() );

		  if (runIntersectionFilter1(gg,ray,vfloat16(uu),vfloat16(0.0f),vfloat16(t[i]),vfloat16(T[0]),vfloat16(T[1]),vfloat16(T[2]),(vbool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
		    break;
		  valid ^= (unsigned int)1 << i;
		}
	      if (unlikely(none(valid))) return false;
	    }
	}

      unsigned int i = select_min(valid,t);
      float uu = (float(i)+u[i])*one_over_width; 
      vfloat16 P,T;
      eval(uu,p0123,P,T);
      assert( T != vfloat16::zero() );

      ray.update((vbool16)1,vfloat16(t[i]),vfloat16(uu),vfloat16::zero(),swAAAA(T),swBBBB(T),swCCCC(T),curve_in.geomID,curve_in.primID);

      return true;
    }

    static __forceinline bool occluded(const vfloat16& pre_vx, 
				       const vfloat16& pre_vy, 
				       const vfloat16& pre_vz, 					
				       const vfloat16& inv_ray_length,
				       const Ray& ray, 
				       const vfloat16 &dir_xyz,
				       const vfloat16 &org_xyz,
				       const Bezier1i& curve_in, 
				       const void* geom) 
    {
      STAT3(shadow.trav_prims,1,1,1);
      const vfloat16 zero = vfloat16::zero();
      const vfloat16 one  = vfloat16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);

      const vfloat16 p0123 = vfloat16::loadu((float*)curve_in.p);
      const vfloat16 p0123_org = p0123 - org_xyz;

      const vfloat16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);


      const vfloat16 c0 = vfloat16::load((float*)&coeff01[0]);
      const vfloat16 c1 = vfloat16::load((float*)&coeff01[1]);
      const vfloat16 c2 = vfloat16::load((float*)&coeff01[2]);
      const vfloat16 c3 = vfloat16::load((float*)&coeff01[3]);

      const Vec4vf16 p0 = eval16(p0123_2D,c0,c1,c2,c3);

      const vfloat16 last_x = vfloat16(p0123_2D[12 + 0]);
      const vfloat16 last_y = vfloat16(p0123_2D[13 + 0]);
      const vfloat16 last_z = vfloat16(p0123_2D[14 + 0]);
      const vfloat16 last_w = vfloat16(p0123_2D[15 + 0]);

      const Vec4vf16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));

      /* approximative intersection with cone */
      const Vec4vf16 v = p1-p0;
      const Vec4vf16 w = -p0;
      const vfloat16 d0 = w.x*v.x + w.y*v.y;
      const vfloat16 d1 = v.x*v.x + v.y*v.y;
      const vfloat16 u = clamp(d0*rcp(d1),zero,one);
      const Vec4vf16 p = p0 + u*v;
      const vfloat16 t = p.z * inv_ray_length;
      const vfloat16 d2 = p.x*p.x + p.y*p.y; 
      const vfloat16 r = p.w;
      const vfloat16 r2 = r*r;
      vbool16 valid = le(d2,r2);
      valid = lt(valid,vfloat16(ray.tnear),t);
      valid = lt(valid,t,vfloat16(ray.tfar));


      if (unlikely(none(valid))) return false;

      STAT3(shadow.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(RTCORE_RAY_MASK)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(curve_in.geomID);
      if (unlikely(g->mask & ray.mask) == 0) return false;
#endif  

      if (ENABLE_INTERSECTION_FILTER) 
	{
	  const Geometry* const gg = ((Scene*)geom)->get(curve_in.geomID);
	  if (unlikely(gg->hasOcclusionFilter1())) 
	    {
	      while(any(valid)) 
		{
		  unsigned int i = select_min(valid,t);
		  const float one_over_width = 1.0f/16.0f;
		  float uu = (float(i)+u[i])*one_over_width; 
		  vfloat16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != vfloat16::zero() );

		  if (runOcclusionFilter1(gg,(Ray&)ray,vfloat16(uu),vfloat16(0.0f),vfloat16(t[i]),vfloat16(T[0]),vfloat16(T[1]),vfloat16(T[2]),(vbool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
		    return true;
		  valid ^= (unsigned int)1 << i;
		}
	    }
	  return false;
	}

      return true;
    }
  };
}
