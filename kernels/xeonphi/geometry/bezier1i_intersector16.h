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
#include "../../common/ray16.h"
#include "filter.h"

namespace embree
{
  typedef LinearSpace3<Vec3f16> LinearSpace_Vec3f16;
    
  /*! Intersector for a single ray from a ray packet with a bezier curve. */

  struct __aligned(64) Precalculations 
  {
     /* __forceinline Precalculations (const Ray16& ray, const size_t k)  */
     /*   : ray_space(frame(Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k])).transposed()) {} // FIXME: works only with normalized ray direction  */

    __forceinline Precalculations (const LinearSpace_Vec3f16& ls16, const float16 &rcp_length, const size_t k)
      : ray_space(ls16.vx.x[k],ls16.vy.x[k],ls16.vz.x[k],
		  ls16.vx.y[k],ls16.vy.y[k],ls16.vz.y[k],
		  ls16.vx.z[k],ls16.vy.z[k],ls16.vz.z[k]),
      inv_ray_length(rcp_length[k])
      {}
    __aligned(64) LinearSpace3fa ray_space;
    float16 inv_ray_length;
  };

  template< bool ENABLE_INTERSECTION_FILTER>
  struct Bezier1iIntersector16
  {
    typedef Bezier1i Primitive;


    static __forceinline Vec4f16 eval16(const float16 &p0123,
				      const float16 &c0,
				      const float16 &c1,
				      const float16 &c2,
				      const float16 &c3)
    {
#if 1
      const float16 p0 = permute<0>(p0123);
      const float16 p1 = permute<1>(p0123);
      const float16 p2 = permute<2>(p0123);
      const float16 p3 = permute<3>(p0123);

      const float16 x = c0 * swAAAA(p0) + c1 * swAAAA(p1) + c2 * swAAAA(p2) + c3 * swAAAA(p3);
      const float16 y = c0 * swBBBB(p0) + c1 * swBBBB(p1) + c2 * swBBBB(p2) + c3 * swBBBB(p3);
      const float16 z = c0 * swCCCC(p0) + c1 * swCCCC(p1) + c2 * swCCCC(p2) + c3 * swCCCC(p3);
      const float16 w = c0 * swDDDD(p0) + c1 * swDDDD(p1) + c2 * swDDDD(p2) + c3 * swDDDD(p3);
#else
      const float16 x = madd(c0,float16(p0123[0]),madd(c1,float16(p0123[4]),madd(c2,float16(p0123[8]),c3* float16(p0123[12]))));
      const float16 y = madd(c0,float16(p0123[1]),madd(c1,float16(p0123[5]),madd(c2,float16(p0123[9]),c3* float16(p0123[13]))));
      const float16 z = madd(c0,float16(p0123[2]),madd(c1,float16(p0123[6]),madd(c2,float16(p0123[10]),c3* float16(p0123[14]))));
      const float16 w = madd(c0,float16(p0123[3]),madd(c1,float16(p0123[7]),madd(c2,float16(p0123[11]),c3* float16(p0123[15]))));
#endif
      return Vec4f16(x,y,z,w);
    }

    static __forceinline void eval(const float t, const float16 &p0123, float16& point, float16& tangent)
    {
      const float16 t0 = float16(1.0f) - float16(t), t1 = float16(t);

      const Vec3fa *__restrict__ const p = (Vec3fa*)&p0123;

      const float16 p00 = broadcast4to16f((float*)&p[0]);
      const float16 p01 = broadcast4to16f((float*)&p[1]);
      const float16 p02 = broadcast4to16f((float*)&p[2]);
      const float16 p03 = broadcast4to16f((float*)&p[3]);

      const float16 p10 = p00 * t0 + p01 * t1;
      const float16 p11 = p01 * t0 + p02 * t1;
      const float16 p12 = p02 * t0 + p03 * t1;
      const float16 p20 = p10 * t0 + p11 * t1;
      const float16 p21 = p11 * t0 + p12 * t1;
      const float16 p30 = p20 * t0 + p21 * t1;

      point = p30;
      tangent = p21-p20;
    }

    static __forceinline bool intersect(const float16& pre_vx, 
					const float16& pre_vy, 
					const float16& pre_vz, 					
					const float16& inv_ray_length,
					Ray16& ray, 
					const float16 &dir_xyz,
					const float16 &org_xyz,
					const size_t k, 
					const Bezier1i& curve_in, 
					const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

      const float16 c0 = load16f(&coeff01[0]);
      const float16 c1 = load16f(&coeff01[1]);
      const float16 c2 = load16f(&coeff01[2]);
      const float16 c3 = load16f(&coeff01[3]);

      const float16 zero = float16::zero();
      const float16 one  = float16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);


      const float16 p0123 = uload16f((float*)curve_in.p);

      const float16 p0123_org = p0123 - org_xyz;

      const float16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);



      const Vec4f16 p0 = eval16(p0123_2D,c0,c1,c2,c3);
      
      const float16 last_x = float16(p0123_2D[12 + 0]);
      const float16 last_y = float16(p0123_2D[13 + 0]);
      const float16 last_z = float16(p0123_2D[14 + 0]);
      const float16 last_w = float16(p0123_2D[15 + 0]);

      const Vec4f16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));


      /* approximative intersection with cone */
      const Vec4f16 v = p1-p0;
      const Vec4f16 w = -p0;
      const float16 d0 = w.x*v.x + w.y*v.y;
      const float16 d1 = v.x*v.x + v.y*v.y;
      const float16 u = clamp(d0*rcp_nr(d1),zero,one);
      const Vec4f16 p = p0 + u*v;
      const float16 t = p.z * inv_ray_length;
      const float16 d2 = p.x*p.x + p.y*p.y; 
      const float16 r = p.w;
      const float16 r2 = r*r;

      bool16 valid = le(d2,r2);
      valid = lt(valid,float16(ray.tnear[k]),t);
      valid = lt(valid,t,float16(ray.tfar[k]));

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
	  if (unlikely(gg->hasIntersectionFilter<float16>())) 
	    {
	      while(any(valid)) 
		{
		  const float one_over_width = 1.0f/16.0f;
		  unsigned int i = select_min(valid,t);
		  float uu = (float(i)+u[i])*one_over_width; 
		  float16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != float16::zero() );

		  if (runIntersectionFilter16(gg,ray,k,float16(uu),float16(0.0f),float16(t[i]),float16(T[0]),float16(T[1]),float16(T[2]),(bool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
		    break;
		  valid ^= (unsigned int)1 << i;
		}
	      if (unlikely(none(valid))) return false;
	    }
	}

      const float one_over_width = 1.0f/16.0f;
      unsigned int i = select_min(valid,t);
      float uu = (float(i)+u[i])*one_over_width; 
      float16 P,T;
      eval(uu,p0123,P,T);
      assert( T != float16::zero() );

      ray.update(1,k,float16(t[i]),float16(uu),float16::zero(),swAAAA(T),swBBBB(T),swCCCC(T),curve_in.geomID,curve_in.primID);
      return true;
    }

    static __forceinline bool occluded(const float16& pre_vx, 
				       const float16& pre_vy, 
				       const float16& pre_vz, 	
				       const float16& inv_ray_length,				
				       const Ray16& ray, 
				       const float16 &dir_xyz,
				       const float16 &org_xyz,
				       const size_t k, 
				       const Bezier1i& curve_in, 
				       const void* geom) 
    {
      STAT3(shadow.trav_prims,1,1,1);
      const float16 zero = float16::zero();
      const float16 one  = float16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);

      const float16 p0123 = uload16f((float*)curve_in.p);
      const float16 p0123_org = p0123 - org_xyz;

      const float16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);


      const float16 c0 = load16f(&coeff01[0]);
      const float16 c1 = load16f(&coeff01[1]);
      const float16 c2 = load16f(&coeff01[2]);
      const float16 c3 = load16f(&coeff01[3]);

      const Vec4f16 p0 = eval16(p0123_2D,c0,c1,c2,c3);

      const float16 last_x = float16(p0123_2D[12 + 0]);
      const float16 last_y = float16(p0123_2D[13 + 0]);
      const float16 last_z = float16(p0123_2D[14 + 0]);
      const float16 last_w = float16(p0123_2D[15 + 0]);

      const Vec4f16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));


      const float one_over_width = 1.0f/16.0f;


      /* approximative intersection with cone */
      const Vec4f16 v = p1-p0;
      const Vec4f16 w = -p0;
      const float16 d0 = w.x*v.x + w.y*v.y;
      const float16 d1 = v.x*v.x + v.y*v.y;
      const float16 u = clamp(d0*rcp(d1),zero,one);
      const Vec4f16 p = p0 + u*v;
      const float16 t = p.z * inv_ray_length;
      const float16 d2 = p.x*p.x + p.y*p.y; 
      const float16 r = p.w;
      const float16 r2 = r*r;
      bool16 valid = le(d2,r2);
      valid = lt(valid,float16(ray.tnear[k]),t);
      valid = lt(valid,t,float16(ray.tfar[k]));


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
	  if (likely(gg->hasOcclusionFilter<float16>())) 
	    {
	      while(any(valid)) 
		{
		  unsigned int i = select_min(valid,t);
		  float uu = (float(i)+u[i])*one_over_width; 
		  float16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != float16::zero() );

		  if (runOcclusionFilter16(gg,(Ray16&)ray,k,float16(uu),float16(0.0f),float16(t[i]),float16(T[0]),float16(T[1]),float16(T[2]),(bool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
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


    static __forceinline bool intersect(const float16& pre_vx, 
					const float16& pre_vy, 
					const float16& pre_vz, 					
					const float16& inv_ray_length,
					Ray& ray, 
					const float16 &dir_xyz,
					const float16 &org_xyz,
					const Bezier1i& curve_in, 
					const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

      const float16 zero = float16::zero();
      const float16 one  = float16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);

      const float16 p0123 = uload16f((float*)curve_in.p);
      const float16 p0123_org = p0123 - org_xyz;

      const float16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);


      const float16 c0 = load16f(&coeff01[0]);
      const float16 c1 = load16f(&coeff01[1]);
      const float16 c2 = load16f(&coeff01[2]);
      const float16 c3 = load16f(&coeff01[3]);

      const Vec4f16 p0 = eval16(p0123_2D,c0,c1,c2,c3);

      const float16 last_x = float16(p0123_2D[12 + 0]);
      const float16 last_y = float16(p0123_2D[13 + 0]);
      const float16 last_z = float16(p0123_2D[14 + 0]);
      const float16 last_w = float16(p0123_2D[15 + 0]);

      const Vec4f16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));

      const float one_over_width = 1.0f/16.0f;


      /* approximative intersection with cone */
      const Vec4f16 v = p1-p0;
      const Vec4f16 w = -p0;
      const float16 d0 = w.x*v.x + w.y*v.y;
      const float16 d1 = v.x*v.x + v.y*v.y;
      const float16 u = clamp(d0*rcp(d1),zero,one);
      const Vec4f16 p = p0 + u*v;
      const float16 t = p.z * inv_ray_length;
      const float16 d2 = p.x*p.x + p.y*p.y; 
      const float16 r = p.w;
      const float16 r2 = r*r;
      bool16 valid = le(d2,r2);
      valid = lt(valid,float16(ray.tnear),t);
      valid = lt(valid,t,float16(ray.tfar));


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
		  float16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != float16::zero() );

		  if (runIntersectionFilter1(gg,ray,float16(uu),float16(0.0f),float16(t[i]),float16(T[0]),float16(T[1]),float16(T[2]),(bool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
		    break;
		  valid ^= (unsigned int)1 << i;
		}
	      if (unlikely(none(valid))) return false;
	    }
	}

      unsigned int i = select_min(valid,t);
      float uu = (float(i)+u[i])*one_over_width; 
      float16 P,T;
      eval(uu,p0123,P,T);
      assert( T != float16::zero() );

      ray.update((bool16)1,float16(t[i]),float16(uu),float16::zero(),swAAAA(T),swBBBB(T),swCCCC(T),curve_in.geomID,curve_in.primID);

      return true;
    }

    static __forceinline bool occluded(const float16& pre_vx, 
				       const float16& pre_vy, 
				       const float16& pre_vz, 					
				       const float16& inv_ray_length,
				       const Ray& ray, 
				       const float16 &dir_xyz,
				       const float16 &org_xyz,
				       const Bezier1i& curve_in, 
				       const void* geom) 
    {
      STAT3(shadow.trav_prims,1,1,1);
      const float16 zero = float16::zero();
      const float16 one  = float16::one();

      prefetch<PFHINT_L1>(curve_in.p + 0);
      prefetch<PFHINT_L1>(curve_in.p + 3);

      const float16 p0123 = uload16f((float*)curve_in.p);
      const float16 p0123_org = p0123 - org_xyz;

      const float16 p0123_2D = select(0x7777,pre_vx * swAAAA(p0123_org) + pre_vy * swBBBB(p0123_org) + pre_vz * swCCCC(p0123_org),p0123);


      const float16 c0 = load16f(&coeff01[0]);
      const float16 c1 = load16f(&coeff01[1]);
      const float16 c2 = load16f(&coeff01[2]);
      const float16 c3 = load16f(&coeff01[3]);

      const Vec4f16 p0 = eval16(p0123_2D,c0,c1,c2,c3);

      const float16 last_x = float16(p0123_2D[12 + 0]);
      const float16 last_y = float16(p0123_2D[13 + 0]);
      const float16 last_z = float16(p0123_2D[14 + 0]);
      const float16 last_w = float16(p0123_2D[15 + 0]);

      const Vec4f16 p1(align_shift_right<1>(last_x,p0[0]),  
       		     align_shift_right<1>(last_y,p0[1]), 
      		     align_shift_right<1>(last_z,p0[2]),  
       		     align_shift_right<1>(last_w,p0[3]));

      /* approximative intersection with cone */
      const Vec4f16 v = p1-p0;
      const Vec4f16 w = -p0;
      const float16 d0 = w.x*v.x + w.y*v.y;
      const float16 d1 = v.x*v.x + v.y*v.y;
      const float16 u = clamp(d0*rcp(d1),zero,one);
      const Vec4f16 p = p0 + u*v;
      const float16 t = p.z * inv_ray_length;
      const float16 d2 = p.x*p.x + p.y*p.y; 
      const float16 r = p.w;
      const float16 r2 = r*r;
      bool16 valid = le(d2,r2);
      valid = lt(valid,float16(ray.tnear),t);
      valid = lt(valid,t,float16(ray.tfar));


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
		  float16 P,T;
		  eval(uu,p0123,P,T);
		  assert( T != float16::zero() );

		  if (runOcclusionFilter1(gg,(Ray&)ray,float16(uu),float16(0.0f),float16(t[i]),float16(T[0]),float16(T[1]),float16(T[2]),(bool16)((unsigned int)1 << i),curve_in.geomID,curve_in.primID))
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
