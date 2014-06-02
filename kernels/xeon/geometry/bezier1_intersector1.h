// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "bezier1.h"
#include "common/ray.h"
#include "geometry/filter.h"
#include "geometry/mailbox.h"

namespace embree
{
  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1Intersector1
  {
    typedef Bezier1 Primitive;

    struct Precalculations 
    {
      __forceinline Precalculations (const Ray& ray)
	: ray_space(frame(ray.dir).transposed()) {} // FIXME: works only with normalized ray direction

      LinearSpace3fa ray_space;
      Mailbox mbox;
    };

    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Bezier1& bezier, const void* geom)
    {
      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa& v0 = bezier.p0;
      const Vec3fa& v1 = bezier.p1;
      const Vec3fa& v2 = bezier.p2;
      const Vec3fa& v3 = bezier.p3;

      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(pre.ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(pre.ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(pre.ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(pre.ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

#if defined (__AVX__)

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: can be calculated from p0 by shifting

      /* approximative intersection with cone */
      const avx4f v = p1-p0;
      const avx4f w = -p0;
      const avxf d0 = w.x*v.x + w.y*v.y;
      const avxf d1 = v.x*v.x + v.y*v.y;
      const avxf u = clamp(d0*rcp(d1),avxf(zero),avxf(one));
      const avx4f p = p0 + u*v;
      const avxf t = p.z;
      const avxf d2 = p.x*p.x + p.y*p.y; 
      const avxf r = p.w;
      const avxf r2 = r*r;
      avxb valid = d2 <= r2 & avxf(ray.tnear) < t & t < avxf(ray.tfar);
      const float one_over_width = 1.0f/8.0f;

#else

      /* subdivide 2 levels at once */ 
      const sse4f p0 = curve2D.eval(sse_coeff0[0],sse_coeff0[1],sse_coeff0[2],sse_coeff0[3]);
      const sse4f p1 = curve2D.eval(sse_coeff1[0],sse_coeff1[1],sse_coeff1[2],sse_coeff1[3]); // FIXME: can be calculated from p0 by shifting

      /* approximative intersection with cone */
      const sse4f v = p1-p0;
      const sse4f w = -p0;
      const ssef d0 = w.x*v.x + w.y*v.y;
      const ssef d1 = v.x*v.x + v.y*v.y;
      const ssef u = clamp(d0*rcp(d1),ssef(zero),ssef(one));
      const sse4f p = p0 + u*v;
      const ssef t = p.z;
      const ssef d2 = p.x*p.x + p.y*p.y; 
      const ssef r = p.w;
      const ssef r2 = r*r;
      sseb valid = d2 <= r2 & ssef(ray.tnear) < t & t < ssef(ray.tfar);
      const float one_over_width = 1.0f/4.0f;

#endif

    retry:
      if (unlikely(none(valid))) return;
      STAT3(normal.trav_prim_hits,1,1,1);
      size_t i = select_min(valid,t);

      /* ray masking test */
#if defined(__USE_RAY_MASK__)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(bezier.geomID);
      if (unlikely(g->mask & ray.mask) == 0) return;
#endif    
      
      /* intersection filter test */
#if defined(__INTERSECTION_FILTER__)
      int geomID = bezier.geomID;
      const Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (!likely(geometry->hasIntersectionFilter1())) 
      {
#endif
        /* update hit information */
        const float uu = (float(i)+u[i])*one_over_width; // FIXME: correct u range for subdivided segments
        const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(uu,P,T);
        if (T == Vec3fa(zero)) { valid[i] = 0; goto retry; } // ignore denormalized curves
        STAT3(normal.trav_prim_hits,1,1,1);
        ray.u = uu;
        ray.v = 0.0f;
        ray.tfar = t[i];
        ray.Ng = T;
        ray.geomID = bezier.geomID;
        ray.primID = bezier.primID;
#if defined(__INTERSECTION_FILTER__)
          return;
      }

      while (true) 
      {
        const float uu = (float(i)+u[i])*one_over_width;
        const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(uu,P,T);
        if (T != Vec3fa(zero))
            if (runIntersectionFilter1(geometry,ray,uu,0.0f,t[i],T,geomID,bezier.primID)) return;
        valid[i] = 0;
        if (none(valid)) return;
        i = select_min(valid,t);
        STAT3(normal.trav_prim_hits,1,1,1);
      }
#endif
    }

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Bezier1* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) 
      {
	//if (unlikely(pre.mbox.hit(curves[i].geomID,curves[i].primID))) continue;
	intersect(pre,ray,curves[i],geom);
	//pre.mbox.add(curves[i].geomID,curves[i].primID);
      }
    }

    static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Bezier1& bezier, const void* geom) 
    {
      /* load bezier curve control points */
      STAT3(shadow.trav_prims,1,1,1);
      const Vec3fa& v0 = bezier.p0;
      const Vec3fa& v1 = bezier.p1;
      const Vec3fa& v2 = bezier.p2;
      const Vec3fa& v3 = bezier.p3;

      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(pre.ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(pre.ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(pre.ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(pre.ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

#if defined (__AVX__)

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: use  _mm_alignr_epi8 (__m128i a, __m128i b, int n)

      /* approximative intersection with cone */
      const avx4f v = p1-p0;
      const avx4f w = -p0;
      const avxf d0 = w.x*v.x + w.y*v.y;
      const avxf d1 = v.x*v.x + v.y*v.y;
      const avxf u = clamp(d0*rcp(d1),avxf(zero),avxf(one));
      const avx4f p = p0 + u*v;
      const avxf t = p.z;
      const avxf d2 = p.x*p.x + p.y*p.y; 
      const avxf r = p.w;
      const avxf r2 = r*r;
      avxb valid = d2 <= r2 & avxf(ray.tnear) < t & t < avxf(ray.tfar);
      const float one_over_width = 1.0f/8.0f;

#else

      /* subdivide 2 levels at once */ 
      const sse4f p0 = curve2D.eval(sse_coeff0[0],sse_coeff0[1],sse_coeff0[2],sse_coeff0[3]);
      const sse4f p1 = curve2D.eval(sse_coeff1[0],sse_coeff1[1],sse_coeff1[2],sse_coeff1[3]); 

      /* approximative intersection with cone */
      const sse4f v = p1-p0;
      const sse4f w = -p0;
      const ssef d0 = w.x*v.x + w.y*v.y;
      const ssef d1 = v.x*v.x + v.y*v.y;
      const ssef u = clamp(d0*rcp(d1),ssef(zero),ssef(one));
      const sse4f p = p0 + u*v;
      const ssef t = p.z;
      const ssef d2 = p.x*p.x + p.y*p.y; 
      const ssef r = p.w;
      const ssef r2 = r*r;
      sseb valid = d2 <= r2 & ssef(ray.tnear) < t & t < ssef(ray.tfar); // FIXME: try to move t-test below (as well as calculations it depends on)
      const float one_over_width = 1.0f/4.0f;

#endif

      if (none(valid)) return false;
      STAT3(shadow.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(__USE_RAY_MASK__)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(bezier.geomID);
      if (unlikely(g->mask & ray.mask) == 0) return false;
#endif  

      /* intersection filter test */
#if defined(__INTERSECTION_FILTER__)
      size_t i = select_min(valid,t);
      int geomID = bezier.geomID;
      const Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (likely(!geometry->hasOcclusionFilter1())) return true;
      
      while (true) 
      {
        /* calculate hit information */
        const float uu = (float(i)+u[i])*one_over_width;
        const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(uu,P,T);
        if (T != Vec3fa(zero))
          if (runOcclusionFilter1(geometry,ray,uu,0.0f,t[i],T,geomID,bezier.primID)) break;
        valid[i] = 0;
        if (none(valid)) return false;
        i = select_min(valid,t);
        STAT3(shadow.trav_prim_hits,1,1,1);
      }
#endif
      return true;
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Bezier1* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (occluded(pre,ray,curves[i],geom))
          return true;

      return false;
    }
  };
}
