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

#include "../../common/ray.h"
#include "../../common/globals.h"
#include "filter.h"

namespace embree
{
  namespace isa
  {
    struct Bezier1Intersector1
    {
      struct Precalculations 
      {
        __forceinline Precalculations (const Ray& ray, const void *ptr)
          : depth_scale(rsqrt(dot(ray.dir,ray.dir))), ray_space(frame(depth_scale*ray.dir).transposed()) {}
        
        float depth_scale;
        LinearSpace3fa ray_space;
      };
      
      static __forceinline void intersect(Ray& ray, const Precalculations& pre, 
                                          const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int& geomID, const int& primID, 
                                          Scene* scene, const unsigned* geomID_to_instID)
      {
        /* transform control points into ray space */
        STAT3(normal.trav_prims,1,1,1);
        Vec3fa w0 = xfmVector(pre.ray_space,v0-ray.org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(pre.ray_space,v1-ray.org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(pre.ray_space,v2-ray.org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(pre.ray_space,v3-ray.org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
#if defined (__AVX__)
        
        /* subdivide 3 levels at once */ 
        const Vec4vf8 p0 = curve2D.eval8(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
        const Vec4vf8 p1 = curve2D.eval8(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: can be calculated from p0 by shifting
        //const Vec4vf8 p1(shift_left1(p0.x,w3.x),shift_left1(p0.y,w3.y),shift_left1(p0.z,w3.z),shift_left1(p0.w,w3.w));
        
        /* approximative intersection with cone */
        const Vec4vf8 v = p1-p0;
        const Vec4vf8 w = -p0;
        const vfloat8 d0 = w.x*v.x + w.y*v.y;
        const vfloat8 d1 = v.x*v.x + v.y*v.y;
        const vfloat8 u = clamp(d0*rcp(d1),vfloat8(zero),vfloat8(one));
        const Vec4vf8 p = p0 + u*v;
        const vfloat8 t = p.z*pre.depth_scale;
        const vfloat8 d2 = p.x*p.x + p.y*p.y; 
        const vfloat8 r = p.w;
        const vfloat8 r2 = r*r;
        vbool8 valid = d2 <= r2 & vfloat8(ray.tnear) < t & t < vfloat8(ray.tfar);
        const float one_over_width = 1.0f/8.0f;
        
#else
        
        /* subdivide 2 levels at once */ 
        const Vec4vf4 p0 = curve2D.eval4(sse_coeff0[0],sse_coeff0[1],sse_coeff0[2],sse_coeff0[3]);
        const Vec4vf4 p1 = curve2D.eval4(sse_coeff1[0],sse_coeff1[1],sse_coeff1[2],sse_coeff1[3]); // FIXME: can be calculated from p0 by shifting
        //const Vec4vf4 p1(shift_left1(p0.x,w3.x),shift_left1(p0.y,w3.y),shift_left1(p0.z,w3.z),shift_left1(p0.w,w3.w));
        
        /* approximative intersection with cone */
        const Vec4vf4 v = p1-p0;
        const Vec4vf4 w = -p0;
        const vfloat4 d0 = w.x*v.x + w.y*v.y;
        const vfloat4 d1 = v.x*v.x + v.y*v.y;
        const vfloat4 u = clamp(d0*rcp(d1),vfloat4(zero),vfloat4(one));
        const Vec4vf4 p = p0 + u*v;
        const vfloat4 t = p.z*pre.depth_scale;
        const vfloat4 d2 = p.x*p.x + p.y*p.y; 
        const vfloat4 r = p.w;
        const vfloat4 r2 = r*r;
        vbool4 valid = d2 <= r2 & vfloat4(ray.tnear) < t & t < vfloat4(ray.tfar);
        const float one_over_width = 1.0f/4.0f;
        
#endif
        int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
      retry:
        if (unlikely(none(valid))) return;
        STAT3(normal.trav_prim_hits,1,1,1);
        size_t i = select_min(valid,t);
        
        /* ray masking test */
#if defined(RTCORE_RAY_MASK)
        BezierCurves* g = scene->getBezierCurves(geomID);
        if (unlikely(g->mask & ray.mask) == 0) return;
#endif  
        
        /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
        Geometry* geometry = scene->get(geomID);
        if (!likely(geometry->hasIntersectionFilter1())) 
        {
#endif
          /* update hit information */
          const float uu = (float(i)+u[i])*one_over_width; // FIXME: correct u range for subdivided segments
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
          Vec3fa P,T; curve3D.eval(uu,P,T);
          if (T == Vec3fa(zero)) { valid[i] = 0; goto retry; } // ignore denormalized curves
          ray.u = uu;
          ray.v = 0.0f;
          ray.tfar = t[i];
          ray.Ng = T;
          ray.geomID = instID;
          ray.primID = primID;
#if defined(RTCORE_INTERSECTION_FILTER)
          return;
        }
        
        while (true) 
        {
          const float uu = (float(i)+u[i])*one_over_width;
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
          Vec3fa P,T; curve3D.eval(uu,P,T);
          if (T != Vec3fa(zero))
            if (runIntersectionFilter1(geometry,ray,uu,0.0f,t[i],T,instID,primID)) return;
          valid[i] = 0;
          if (none(valid)) return;
          i = select_min(valid,t);
          STAT3(normal.trav_prim_hits,1,1,1);
        }
#endif
      }
      
      static __forceinline bool occluded(Ray& ray, const Precalculations& pre,
                                         const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int& geomID, const int& primID, 
                                         Scene* scene, const unsigned* geomID_to_instID) 
      {
        /* transform control points into ray space */
        STAT3(shadow.trav_prims,1,1,1);
        Vec3fa w0 = xfmVector(pre.ray_space,v0-ray.org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(pre.ray_space,v1-ray.org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(pre.ray_space,v2-ray.org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(pre.ray_space,v3-ray.org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
#if defined (__AVX__)
        
        /* subdivide 3 levels at once */ 
        const Vec4vf8 p0 = curve2D.eval8(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
        const Vec4vf8 p1 = curve2D.eval8(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);
        
        /* approximative intersection with cone */
        const Vec4vf8 v = p1-p0;
        const Vec4vf8 w = -p0;
        const vfloat8 d0 = w.x*v.x + w.y*v.y;
        const vfloat8 d1 = v.x*v.x + v.y*v.y;
        const vfloat8 u = clamp(d0*rcp(d1),vfloat8(zero),vfloat8(one));
        const Vec4vf8 p = p0 + u*v;
        const vfloat8 t = p.z*pre.depth_scale;
        const vfloat8 d2 = p.x*p.x + p.y*p.y; 
        const vfloat8 r = p.w;
        const vfloat8 r2 = r*r;
        vbool8 valid = d2 <= r2 & vfloat8(ray.tnear) < t & t < vfloat8(ray.tfar);
        const float one_over_width = 1.0f/8.0f;
        
#else
        
        /* subdivide 2 levels at once */ 
        const Vec4vf4 p0 = curve2D.eval4(sse_coeff0[0],sse_coeff0[1],sse_coeff0[2],sse_coeff0[3]);
        const Vec4vf4 p1 = curve2D.eval4(sse_coeff1[0],sse_coeff1[1],sse_coeff1[2],sse_coeff1[3]);
        
        /* approximative intersection with cone */
        const Vec4vf4 v = p1-p0;
        const Vec4vf4 w = -p0;
        const vfloat4 d0 = w.x*v.x + w.y*v.y;
        const vfloat4 d1 = v.x*v.x + v.y*v.y;
        const vfloat4 u = clamp(d0*rcp(d1),vfloat4(zero),vfloat4(one));
        const Vec4vf4 p = p0 + u*v;
        const vfloat4 t = p.z*pre.depth_scale;
        const vfloat4 d2 = p.x*p.x + p.y*p.y; 
        const vfloat4 r = p.w;
        const vfloat4 r2 = r*r;
        vbool4 valid = d2 <= r2 & vfloat4(ray.tnear) < t & t < vfloat4(ray.tfar);
        const float one_over_width = 1.0f/4.0f;
        
#endif
        
        if (none(valid)) return false;
        STAT3(shadow.trav_prim_hits,1,1,1);
        
        /* ray masking test */
#if defined(RTCORE_RAY_MASK)
        BezierCurves* g = scene->getBezierCurves(geomID);
        if (unlikely(g->mask & ray.mask) == 0) return false;
#endif  
        
        /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
        
        size_t i = select_min(valid,t);
        Geometry* geometry = scene->get(geomID);
        if (likely(!geometry->hasOcclusionFilter1())) return true;
        
        const int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;
        while (true) 
        {
          /* calculate hit information */
          const float uu = (float(i)+u[i])*one_over_width;
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
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
    };


    /*! Intersector for a single ray from a ray packet with a bezier curve. */
    template<int K>
    struct Bezier1IntersectorK
    {
      typedef Vec3<vfloat<K>> Vec3vfK;
      
      struct Precalculations 
      {
        __forceinline Precalculations (const vbool<K>& valid, const RayK<K>& ray)
        {
          int mask = movemask(valid);
          depth_scale = rsqrt(dot(ray.dir,ray.dir));
          while (mask) {
            size_t k = __bscf(mask);
            ray_space[k] = frame(depth_scale[k]*Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k])).transposed();
          }
        }
        
        __forceinline Precalculations (const RayK<K>& ray, size_t k)
        {
          Vec3fa ray_dir = Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
          depth_scale[k] = rsqrt(dot(ray_dir,ray_dir));
          ray_space  [k] = frame(depth_scale[k]*ray_dir).transposed();
        }
        
        vfloat<K> depth_scale;
        LinearSpace3fa ray_space[vfloat<K>::size];
      };
      
      static __forceinline void intersect(const Precalculations& pre, RayK<K>& ray, size_t k,
                                          const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int& geomID, const int& primID, 
                                          Scene* scene)
      {
        STAT3(normal.trav_prims,1,1,1);
        
        /* load ray */
        const Vec3fa ray_org(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
        const Vec3fa ray_dir(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
        const float ray_tnear = ray.tnear[k];
        const float ray_tfar  = ray.tfar [k];
        
        /* transform control points into ray space */
        Vec3fa w0 = xfmVector(pre.ray_space[k],v0-ray_org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(pre.ray_space[k],v1-ray_org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(pre.ray_space[k],v2-ray_org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(pre.ray_space[k],v3-ray_org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
#if defined (__AVX__)
        
        /* subdivide 3 levels at once */ 
        const Vec4vf8 p0 = curve2D.eval8(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
        const Vec4vf8 p1 = curve2D.eval8(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: can be calculated from p0 by shifting
        
        /* approximative intersection with cone */
        const Vec4vf8 v = p1-p0;
        const Vec4vf8 w = -p0;
        const vfloat8 d0 = w.x*v.x + w.y*v.y;
        const vfloat8 d1 = v.x*v.x + v.y*v.y;
        const vfloat8 u = clamp(d0*rcp(d1),vfloat8(zero),vfloat8(one));
        const Vec4vf8 p = p0 + u*v;
        const vfloat8 t = p.z*pre.depth_scale[k];
        const vfloat8 d2 = p.x*p.x + p.y*p.y; 
        const vfloat8 r = p.w;
        const vfloat8 r2 = r*r;
        vbool8 valid = d2 <= r2 & vfloat8(ray_tnear) < t & t < vfloat8(ray_tfar);
        const float one_over_width = 1.0f/8.0f;
        
#else
        
        /* subdivide 2 levels at once */ 
        const Vec4vf4 p0 = curve2D.eval4(sse_coeff0[0],sse_coeff0[1],sse_coeff0[2],sse_coeff0[3]);
        const Vec4vf4 p1 = curve2D.eval4(sse_coeff1[0],sse_coeff1[1],sse_coeff1[2],sse_coeff1[3]); // FIXME: can be calculated from p0 by shifting
        
        /* approximative intersection with cone */
        const Vec4vf4 v = p1-p0;
        const Vec4vf4 w = -p0;
        const vfloat4 d0 = w.x*v.x + w.y*v.y;
        const vfloat4 d1 = v.x*v.x + v.y*v.y;
        const vfloat4 u = clamp(d0*rcp(d1),vfloat4(zero),vfloat4(one));
        const Vec4vf4 p = p0 + u*v;
        const vfloat4 t = p.z*pre.depth_scale[k];
        const vfloat4 d2 = p.x*p.x + p.y*p.y; 
        const vfloat4 r = p.w;
        const vfloat4 r2 = r*r;
        vbool4 valid = d2 <= r2 & vfloat4(ray_tnear) < t & t < vfloat4(ray_tfar);
        const float one_over_width = 1.0f/4.0f;
        
#endif
        
      retry:
        if (unlikely(none(valid))) return;
        STAT3(normal.trav_prim_hits,1,1,1);
        size_t i = select_min(valid,t);
        
        /* ray masking test */
#if defined(RTCORE_RAY_MASK)
        BezierCurves* g = scene->getBezierCurves(geomID);
        if (unlikely(g->mask & ray.mask[k]) == 0) return;
#endif    
        
        /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
        const Geometry* geometry = scene->get(geomID);
        if (!likely(geometry->hasIntersectionFilter<vfloat<K>>()))
        {
#endif
          /* update hit information */
          const float uu = (float(i)+u[i])*one_over_width; // FIXME: correct u range for subdivided segments
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
          Vec3fa P,T; curve3D.eval(uu,P,T);
          if (T == Vec3fa(zero)) { valid[i] = 0; goto retry; } // ignore denormalized curves
          STAT3(normal.trav_prim_hits,1,1,1);
          ray.u[k] = uu;
          ray.v[k] = 0.0f;
          ray.tfar[k] = t[i];
          ray.Ng.x[k] = T.x;
          ray.Ng.y[k] = T.y;
          ray.Ng.z[k] = T.z;
          ray.geomID[k] = geomID;
          ray.primID[k] = primID;
#if defined(RTCORE_INTERSECTION_FILTER)
          return;
        }
        
        while (true) 
        {
          const float uu = (float(i)+u[i])*one_over_width;
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
          Vec3fa P,T; curve3D.eval(uu,P,T);
          if (T != Vec3fa(zero))
            if (runIntersectionFilter(geometry,ray,k,uu,0.0f,t[i],T,geomID,primID)) return;
          valid[i] = 0;
          if (none(valid)) return;
          i = select_min(valid,t);
          STAT3(normal.trav_prim_hits,1,1,1);
        }
#endif
      }
      
      static __forceinline bool occluded(const Precalculations& pre, RayK<K>& ray, const size_t k,
                                         const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int& geomID, const int& primID, 
                                         Scene* scene) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        
        /* load ray */
        const Vec3fa ray_org(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
        const Vec3fa ray_dir(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
        const float ray_tnear = ray.tnear[k];
        const float ray_tfar  = ray.tfar [k];
        
        /* transform control points into ray space */
        Vec3fa w0 = xfmVector(pre.ray_space[k],v0-ray_org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(pre.ray_space[k],v1-ray_org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(pre.ray_space[k],v2-ray_org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(pre.ray_space[k],v3-ray_org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
#if defined (__AVX__)
        
        /* subdivide 3 levels at once */ 
        const Vec4vf8 p0 = curve2D.eval8(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
        const Vec4vf8 p1 = curve2D.eval8(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);
        
        /* approximative intersection with cone */
        const Vec4vf8 v = p1-p0;
        const Vec4vf8 w = -p0;
        const vfloat8 d0 = w.x*v.x + w.y*v.y;
        const vfloat8 d1 = v.x*v.x + v.y*v.y;
        const vfloat8 u = clamp(d0*rcp(d1),vfloat8(zero),vfloat8(one));
        const Vec4vf8 p = p0 + u*v;
        const vfloat8 t = p.z*pre.depth_scale[k];
        const vfloat8 d2 = p.x*p.x + p.y*p.y; 
        const vfloat8 r = p.w;
        const vfloat8 r2 = r*r;
        vbool8 valid = d2 <= r2 & vfloat8(ray_tnear) < t & t < vfloat8(ray_tfar);
        const float one_over_width = 1.0f/8.0f;
        
#else
        
        /* subdivide 2 levels at once */ 
        const Vec4vf4 p0 = curve2D.eval4(sse_coeff0[0],sse_coeff0[1],sse_coeff0[2],sse_coeff0[3]);
        const Vec4vf4 p1 = curve2D.eval4(sse_coeff1[0],sse_coeff1[1],sse_coeff1[2],sse_coeff1[3]);
        
        /* approximative intersection with cone */
        const Vec4vf4 v = p1-p0;
        const Vec4vf4 w = -p0;
        const vfloat4 d0 = w.x*v.x + w.y*v.y;
        const vfloat4 d1 = v.x*v.x + v.y*v.y;
        const vfloat4 u = clamp(d0*rcp(d1),vfloat4(zero),vfloat4(one));
        const Vec4vf4 p = p0 + u*v;
        const vfloat4 t = p.z*pre.depth_scale[k];
        const vfloat4 d2 = p.x*p.x + p.y*p.y; 
        const vfloat4 r = p.w;
        const vfloat4 r2 = r*r;
        vbool4 valid = d2 <= r2 & vfloat4(ray_tnear) < t & t < vfloat4(ray_tfar);
        const float one_over_width = 1.0f/4.0f;
        
#endif
        
        if (none(valid)) return false;
        STAT3(shadow.trav_prim_hits,1,1,1);
        
        /* ray masking test */
#if defined(RTCORE_RAY_MASK)
        BezierCurves* g = scene->getBezierCurves(geomID);
        if (unlikely(g->mask & ray.mask[k]) == 0) return false;
#endif  
        
        /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
        size_t i = select_min(valid,t);
        const Geometry* geometry = scene->get(geomID);
        if (likely(!geometry->hasOcclusionFilter<vfloat<K>>())) return true;
        
        while (true) 
        {
          /* calculate hit information */
          const float uu = (float(i)+u[i])*one_over_width;
          const BezierCurve3fa curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
          Vec3fa P,T; curve3D.eval(uu,P,T);
          if (T != Vec3fa(zero))
            if (runOcclusionFilter(geometry,ray,k,uu,0.0f,t[i],T,geomID,primID)) break;
          valid[i] = 0;
          if (none(valid)) return false;
          i = select_min(valid,t);
          STAT3(shadow.trav_prim_hits,1,1,1);
        }
#endif
        return true;
      }
    };
  }
}
