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

#include "bezier1i.h"
#include "common/ray.h"
#include "geometry/filter.h"
#include "geometry/mailbox.h"

namespace embree
{
  static __forceinline void intersectBezierCurve(Ray& ray, const LinearSpace3fa& ray_space, 
                                                 const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int& geomID, const int& primID, 
                                                 const void* geom)
    {
      /* transform control points into ray space */
      STAT3(normal.trav_prims,1,1,1);
      Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

#if defined (__AVX__)

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: can be calculated from p0 by shifting
      //const avx4f p1(shift_left1(p0.x,w3.x),shift_left1(p0.y,w3.y),shift_left1(p0.z,w3.z),shift_left1(p0.w,w3.w));

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
      //const sse4f p1(shift_left1(p0.x,w3.x),shift_left1(p0.y,w3.y),shift_left1(p0.z,w3.z),shift_left1(p0.w,w3.w));

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
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(geomID);
      if (unlikely(g->mask & ray.mask) == 0) return;
#endif  

      /* intersection filter test */
#if defined(__INTERSECTION_FILTER__)
      Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (!likely(geometry->hasIntersectionFilter1())) 
      {
#endif
        /* update hit information */
        const float uu = (float(i)+u[i])*one_over_width; // FIXME: correct u range for subdivided segments
        const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(uu,P,T);
        if (T == Vec3fa(zero)) { valid[i] = 0; goto retry; } // ignore denormalized curves
        ray.u = uu;
        ray.v = 0.0f;
        ray.tfar = t[i];
        ray.Ng = T;
        ray.geomID = geomID;
        ray.primID = primID;
#if defined(__INTERSECTION_FILTER__)
          return;
      }

      while (true) 
      {
        const float uu = (float(i)+u[i])*one_over_width;
        const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(uu,P,T);
        if (T != Vec3fa(zero))
            if (runIntersectionFilter1(geometry,ray,uu,0.0f,t[i],T,geomID,primID)) return;
        valid[i] = 0;
        if (none(valid)) return;
        i = select_min(valid,t);
        STAT3(normal.trav_prim_hits,1,1,1);
      }
#endif
    }

    static __forceinline bool occludedBezierCurve(Ray& ray, const LinearSpace3fa& ray_space, 
                                                  const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int& geomID, const int& primID, 
                                                  const void* geom) 
    {
      /* transform control points into ray space */
      STAT3(shadow.trav_prims,1,1,1);
      Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

#if defined (__AVX__)

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);

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
      sseb valid = d2 <= r2 & ssef(ray.tnear) < t & t < ssef(ray.tfar);
      const float one_over_width = 1.0f/4.0f;

#endif

      if (none(valid)) return false;
      STAT3(shadow.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(__USE_RAY_MASK__)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(geomID);
      if (unlikely(g->mask & ray.mask) == 0) return false;
#endif  

      /* intersection filter test */
#if defined(__INTERSECTION_FILTER__)

      size_t i = select_min(valid,t);
      Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (likely(!geometry->hasOcclusionFilter1())) return true;
      

      while (true) 
      {
        /* calculate hit information */
        const float uu = (float(i)+u[i])*one_over_width;
        const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(uu,P,T);
        if (T != Vec3fa(zero))
          if (runOcclusionFilter1(geometry,ray,uu,0.0f,t[i],T,geomID,primID)) break;
        valid[i] = 0;
        if (none(valid)) return false;
        i = select_min(valid,t);
        STAT3(shadow.trav_prim_hits,1,1,1);
      }
#endif
      return true;
    }

  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1iIntersector1
  {
    typedef Bezier1i Primitive;

    struct Precalculations 
    {
      __forceinline Precalculations (const Ray& ray)
	: ray_space(rcp(frame(ray.dir))) {}

      LinearSpace3fa ray_space;
      //Mailbox mbox;
    };

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Bezier1i* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
      {
	//if (unlikely(pre.mbox.hit(curves[i].geomID,curves[i].primID))) continue;
        intersectBezierCurve(ray,pre.ray_space,curves[i].p[0],curves[i].p[1],curves[i].p[2],curves[i].p[3],curves[i].geomID,curves[i].primID,geom);
	//pre.mbox.add(curves[i].geomID,curves[i].primID);
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Bezier1i* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (occludedBezierCurve(ray,pre.ray_space,curves[i].p[0],curves[i].p[1],curves[i].p[2],curves[i].p[3],curves[i].geomID,curves[i].primID,geom))
          return true;

      return false;
    }
  };

  /*! Intersector for a single ray with a bezier curve. */
  struct Bezier1iIntersector1MB
  {
    typedef Bezier1iMB Primitive;

    struct Precalculations 
    {
      __forceinline Precalculations (const Ray& ray)
	: ray_space(rcp(frame(ray.dir))) {}

      LinearSpace3fa ray_space;
      //Mailbox mbox;
    };

    static __forceinline void intersect(Precalculations& pre, Ray& ray, const Bezier1iMB* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
      {
	//if (unlikely(pre.mbox.hit(curves[i].geomID,curves[i].primID))) continue;
        const Vec3fa a0 = curves[i].p0[0], a1 = curves[i].p0[1], a2 = curves[i].p0[2], a3 = curves[i].p0[3];
        const Vec3fa b0 = curves[i].p1[0], b1 = curves[i].p1[1], b2 = curves[i].p1[2], b3 = curves[i].p1[3];
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;
        intersectBezierCurve(ray,pre.ray_space,p0,p1,p2,p3,curves[i].geomID,curves[i].primID,geom);
	//pre.mbox.add(curves[i].geomID,curves[i].primID);
      }
    }

    static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Bezier1iMB* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
      {
        const Vec3fa a0 = curves[i].p0[0], a1 = curves[i].p0[1], a2 = curves[i].p0[2], a3 = curves[i].p0[3];
        const Vec3fa b0 = curves[i].p1[0], b1 = curves[i].p1[1], b2 = curves[i].p1[2], b3 = curves[i].p1[3];
        const float t0 = 1.0f-ray.time, t1 = ray.time;
        const Vec3fa p0 = t0*a0 + t1*b0;
        const Vec3fa p1 = t0*a1 + t1*b1;
        const Vec3fa p2 = t0*a2 + t1*b2;
        const Vec3fa p3 = t0*a3 + t1*b3;

        if (occludedBezierCurve(ray,pre.ray_space,p0,p1,p2,p3,curves[i].geomID,curves[i].primID,geom))
          return true;
      }
      return false;
    }
  };
}
