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

#include "bezier1.h"
#include "common/ray.h"
#include "geometry/filter.h"

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
    };

    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Bezier1& bezier, const void* geom)
    {
      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa& v0 = bezier.p0;
      const Vec3fa& v1 = bezier.p1;
      const Vec3fa& v2 = bezier.p2;
      const Vec3fa& v3 = bezier.p3;

#if 0
      /* subdivide 3 levels at once */ 
      const BezierCurve3D curve2D(v0,v1,v2,v3,0.0f,1.0f,0);
      const avx4f a = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f b = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: can be calculated from p0 by shifting
      const avx3f a3(a.x,a.y,a.z);
      const avx3f b3(b.x,b.y,b.z);

      const avxf  rl0 = 1.0f/length(b3-a3); // FIXME: multiply equation with this
      const avx3f p0 = a3, d0 = (b3-a3)*rl0;
      const avxf  r0 = a.w, dr = (b.w-a.w)*rl0;
      const float rl1 = 1.0f/length(ray.dir); // FIXME: normalization not required
      const avx3f p1 = ray.org, d1 = ray.dir*rl1;

      const avx3f dp = p1-p0;
      const avxf dpdp = dot(dp,dp);
      const avxf d1d1 = dot(d1,d1);
      const avxf d0d1 = dot(d0,d1);
      const avxf d0dp = dot(d0,dp);
      const avxf d1dp = dot(d1,dp);
      const avxf R = r0 + d0dp*dr;
      const avxf A = d1d1 - sqr(d0d1) * (1.0f+dr*dr);
      const avxf B = 2.0f * (d1dp - d0d1*(d0dp + R*dr));
      const avxf C = dpdp - (sqr(d0dp) + sqr(R));
      const avxf D = B*B - 4.0f*A*C;
      avxb valid = D >= 0.0f;
      if (none(valid)) return;
      
      const avxf Q = sqrt(D);
      //const avxf t0 = (-B-Q)*rcp2A;
      //const avxf t1 = (-B+Q)*rcp2A;
      const avxf t0 = (-B-Q)/(2.0f*A);
      const avxf u0 = d0dp+t0*d0d1;
      const avxf t = t0*rl1;
      const avxf u = u0*rl0;
      valid &= (ray.tnear < t) & (t < ray.tfar) & (0.0f <= u) & (u <= 1.0f);

#else

      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(pre.ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(pre.ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(pre.ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(pre.ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

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
      const avxf r = p.w; //max(p.w,ray.org.w+ray.dir.w*t);
      const avxf r2 = r*r;
      avxb valid = d2 <= r2 & avxf(ray.tnear) < t & t < avxf(ray.tfar);

#endif

    retry:
      if (unlikely(none(valid))) return;
      const float one_over_8 = 1.0f/8.0f;
      size_t i = select_min(valid,t);
      STAT3(normal.trav_prim_hits,1,1,1);

      /* intersection filter test */
#if defined(__INTERSECTION_FILTER__)
      int geomID = bezier.geomID;
      const Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (!likely(geometry->hasIntersectionFilter1())) 
      {
#endif
        /* update hit information */
        const float uu = (float(i)+u[i])*one_over_8; // FIXME: correct u range for subdivided segments
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
        const float uu = (float(i)+u[i])*one_over_8;
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

    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Bezier1* curves, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(pre,ray,curves[i],geom);
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
      const avxf r = p.w; //max(p.w,ray.org.w+ray.dir.w*t);
      const avxf r2 = r*r;
      avxb valid = d2 <= r2 & avxf(ray.tnear) < t & t < avxf(ray.tfar);
      if (none(valid)) return false;
      STAT3(shadow.trav_prim_hits,1,1,1);

      /* intersection filter test */
#if defined(__INTERSECTION_FILTER__)

      size_t i = select_min(valid,t);
      int geomID = bezier.geomID;
      const Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (likely(!geometry->hasOcclusionFilter1())) return true;
      const float one_over_8 = 1.0f/8.0f;

      while (true) 
      {
        /* calculate hit information */
        const float uu = (float(i)+u[i])*one_over_8;
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

    static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Bezier1* curves, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (occluded(pre,ray,curves[i],geom))
          return true;

      return false;
    }
  };
}
