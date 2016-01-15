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
    template<int M>
      struct BezierHit
    {
      __forceinline BezierHit() {}

      __forceinline BezierHit(const vbool<M>& valid, const vfloat<M>& U, const vfloat<M>& V, const vfloat<M>& T, 
                              const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3)
        : valid(valid), U(U), V(V), T(T), p0(p0), p1(p1), p2(p2), p3(p3) {}
      
      __forceinline void finalize() 
      {
        vu = (vfloat<M>(step)+U)*(1.0f/float(M));
        vv = 0.0f;
        vt = T;
      }
      
      __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
      __forceinline float t  (const size_t i) const { return vt[i]; }
      __forceinline Vec3fa Ng(const size_t i) const 
      { 
        const BezierCurve3fa curve3D(p0,p1,p2,p3,0.0f,1.0f,0);
        Vec3fa P,T; curve3D.eval(vu[i],P,T);
        return T == Vec3fa(zero) ? Vec3fa(one) : T; 
      }
      
    public:
      vfloat<M> U;
      vfloat<M> V;
      vfloat<M> T;
      Vec3fa p0,p1,p2,p3;
      
    public:
      vbool<M> valid;
      vfloat<M> vu;
      vfloat<M> vv;
      vfloat<M> vt;
    };
    
    struct Bezier1Intersector1
    {
      float depth_scale;
      LinearSpace3fa ray_space;

      __forceinline Bezier1Intersector1(const Ray& ray, const void* ptr) 
         : depth_scale(rsqrt(dot(ray.dir,ray.dir))), ray_space(frame(depth_scale*ray.dir).transposed()) {}

      template<typename Epilog>
      __forceinline bool intersect(Ray& ray,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, 
                                   const Epilog& epilog) const
      {
       /* transform control points into ray space */
        STAT3(normal.trav_prims,1,1,1);
        Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
        /* subdivide 2 levels at once */ 
        const Vec4vf4 p0 = curve2D.eval0(vbool4(true),0,4);
        const Vec4vf4 p1 = curve2D.eval1(vbool4(true),0,4);
        
        /* approximative intersection with cone */
        const Vec4vf4 v = p1-p0;
        const Vec4vf4 w = -p0;
        const vfloat4 d0 = w.x*v.x + w.y*v.y;
        const vfloat4 d1 = v.x*v.x + v.y*v.y;
        const vfloat4 u = clamp(d0*rcp(d1),vfloat4(zero),vfloat4(one));
        const Vec4vf4 p = p0 + u*v;
        const vfloat4 t = p.z*depth_scale;
        const vfloat4 d2 = p.x*p.x + p.y*p.y; 
        const vfloat4 r = p.w;
        const vfloat4 r2 = r*r;
        vbool4 valid = d2 <= r2 & vfloat4(ray.tnear) < t & t < vfloat4(ray.tfar);
        if (likely(none(valid))) return false;
        
        /* update hit information */
        BezierHit<4> hit(valid,u,0.0f,t,v0,v1,v2,v3);
        return epilog(valid,hit);
      }
    };

    template<int K>
    struct Bezier1IntersectorK
    {
      vfloat<K> depth_scale;
      LinearSpace3fa ray_space[K];

      __forceinline Bezier1IntersectorK(const vbool<K>& valid, const RayK<K>& ray) 
      {
        int mask = movemask(valid);
        depth_scale = rsqrt(dot(ray.dir,ray.dir));
        while (mask) {
          size_t k = __bscf(mask);
          ray_space[k] = frame(depth_scale[k]*Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k])).transposed();
        }
      }

      __forceinline Bezier1IntersectorK (const RayK<K>& ray, size_t k)
      {
        Vec3fa ray_dir = Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
        depth_scale[k] = rsqrt(dot(ray_dir,ray_dir));
        ray_space  [k] = frame(depth_scale[k]*ray_dir).transposed();
      }
      
      template<typename Epilog>
      __forceinline bool intersect(RayK<K>& ray, size_t k,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3,
                                   const Epilog& epilog) const
      {
        /* load ray */
        const Vec3fa ray_org(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
        const Vec3fa ray_dir(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
        const float ray_tnear = ray.tnear[k];
        const float ray_tfar  = ray.tfar [k];
        
        /* transform control points into ray space */
        Vec3fa w0 = xfmVector(ray_space[k],v0-ray_org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(ray_space[k],v1-ray_org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(ray_space[k],v2-ray_org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(ray_space[k],v3-ray_org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
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
        const vfloat4 t = p.z*depth_scale[k];
        const vfloat4 d2 = p.x*p.x + p.y*p.y; 
        const vfloat4 r = p.w;
        const vfloat4 r2 = r*r;
        vbool4 valid = d2 <= r2 & vfloat4(ray_tnear) < t & t < vfloat4(ray_tfar);
        if (likely(none(valid))) return false;
        
        /* update hit information */
        BezierHit<4> hit(valid,u,0.0f,t,v0,v1,v2,v3);
        return epilog(valid,hit);
      }
    };
  }
}
