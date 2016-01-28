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
#include "line_intersector.h" // FIXME: remove later

namespace embree
{
  namespace isa
  {
    template<int M>
      struct BezierHit
    {
      __forceinline BezierHit() {}

      __forceinline BezierHit(const vbool<M>& valid, const vfloat<M>& U, const vfloat<M>& V, const vfloat<M>& T, const int i, const int N,
                              const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3)
        : valid(valid), U(U), V(V), T(T), i(i), N(N), p0(p0), p1(p1), p2(p2), p3(p3) {}
      
      __forceinline void finalize() 
      {
        vu = (vfloat<M>(step)+U+vfloat<M>(i))*(1.0f/float(N));
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
      int i, N;
      Vec3fa p0,p1,p2,p3;
      
    public:
      vbool<M> valid;
      vfloat<M> vu;
      vfloat<M> vv;
      vfloat<M> vt;
    };
    
#if 1

    struct Bezier1Intersector1
    {
      __forceinline Bezier1Intersector1(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
      __forceinline bool intersect(Ray& ray,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int N0,
                                   const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);
        bool ishit = false;
        BezierCurve3fa curve2D(v0,v1,v2,v3,0.0f,1.0f,4);
        const int N = 3; // FIXME

        /* process SIMD-size-1 many segments per iteration */
        for (int i=0; i<N; i+=VSIZEX-1)
        {
          /* evaluate the bezier curve */
          vboolx valid = vintx(i)+vintx(step) < vintx(N);
          const Vec4vfx p  = curve2D.eval0(valid,i,N);
          const Vec4vfx p1  = curve2D.eval1(valid,i,N); // FIXME: optimize away
          const Vec4vfx dp_ = curve2D.derivative(valid,i,N);
          const Vec3vfx dp = normalize(Vec3vfx(dp_.x,dp_.y,dp_.z));

          /* early exit */
          const Vec3vfx Q1(p.x,p.y,p.z);
          const Vec3vfx Q2(p1.x,p1.y,p1.z);
          valid &= abs(dot(Vec3vfx(ray.org)-Q1,normalize_safe(cross(Q2-Q1,Vec3vfx(ray.dir))))) <= max(p.w,p1.w);
          if (none(valid)) continue;
         
          /* intersect each bezier segment */
          vboolx valid_o = false;
          LineIntersectorHitM<VSIZEX> hit;

          for (size_t j=0; j<min(VSIZEX-1,N-i); j++)
          {
            const Vec3fa p1( p.x[j+0], p.y[j+0] ,p.z[j+0]);
            const Vec3fa p2( p.x[j+1], p.y[j+1] ,p.z[j+1]);
            const Vec3fa n1(dp.x[j+0],dp.y[j+0],dp.z[j+0]);
            const Vec3fa n2(dp.x[j+1],dp.y[j+1],dp.z[j+1]);
            const float  r1 =  p.w[j+0];
            const float  r2 =  p.w[j+1];
            float u = 0.0f;
            float t = 0.0f;
            Vec3fa Ng = zero;
            if (!intersect_fill_cone(ray,p1,n1,r1,p2,n2,r2,u,t,Ng)) continue;
            hit.vu[j] = u;
            hit.vv[j] = 0.0f;
            hit.vt[j] = t;
            hit.vNg.x[j] = Ng.x;
            hit.vNg.y[j] = Ng.y;
            hit.vNg.z[j] = Ng.z;
            valid_o[j] = 0xFFFFFFFF;
          }
          
          /* update hit information */
          if (unlikely(none(valid_o))) continue;
          ishit |= epilog(valid_o,hit);
        }
        return ishit;
      }
    };

#else

    struct Bezier1Intersector1
    {
      float depth_scale;
      LinearSpace3fa ray_space;

      __forceinline Bezier1Intersector1(const Ray& ray, const void* ptr) 
         : depth_scale(rsqrt(dot(ray.dir,ray.dir))), ray_space(frame(depth_scale*ray.dir).transposed()) {}

      template<typename Epilog>
      __forceinline bool intersect(Ray& ray,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int N,
                                   const Epilog& epilog) const
      {
        /* transform control points into ray space */
        STAT3(normal.trav_prims,1,1,1);
        Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
        Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
        Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
        Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
        BezierCurve3fa curve2D(w0,w1,w2,w3,0.0f,1.0f,4);
        
        /* evaluate the bezier curve */
        vboolx valid = vfloatx(step) < vfloatx(N);
        const Vec4vfx p0 = curve2D.eval0(valid,0,N);
        const Vec4vfx p1 = curve2D.eval1(valid,0,N);
        
        /* approximative intersection with cone */
        const Vec4vfx v = p1-p0;
        const Vec4vfx w = -p0;
        const vfloatx d0 = w.x*v.x + w.y*v.y;
        const vfloatx d1 = v.x*v.x + v.y*v.y;
        const vfloatx u = clamp(d0*rcp(d1),vfloatx(zero),vfloatx(one));
        const Vec4vfx p = p0 + u*v;
        const vfloatx t = p.z*depth_scale;
        const vfloatx d2 = p.x*p.x + p.y*p.y; 
        const vfloatx r = p.w;
        const vfloatx r2 = r*r;
        valid &= d2 <= r2 & vfloatx(ray.tnear) < t & t < vfloatx(ray.tfar);

        /* update hit information */
         bool ishit = false;
        if (unlikely(any(valid))) {
          BezierHit<VSIZEX> hit(valid,u,0.0f,t,0,N,v0,v1,v2,v3);
          ishit |= epilog(valid,hit);
        }

        if (unlikely(VSIZEX < N)) 
        {
          /* process SIMD-size many segments per iteration */
          for (int i=VSIZEX; i<N; i+=VSIZEX)
          {
            /* evaluate the bezier curve */
            vboolx valid = vintx(i)+vintx(step) < vintx(N);
            const Vec4vfx p0 = curve2D.eval0(valid,i,N);
            const Vec4vfx p1 = curve2D.eval1(valid,i,N);
            
            /* approximative intersection with cone */
            const Vec4vfx v = p1-p0;
            const Vec4vfx w = -p0;
            const vfloatx d0 = w.x*v.x + w.y*v.y;
            const vfloatx d1 = v.x*v.x + v.y*v.y;
            const vfloatx u = clamp(d0*rcp(d1),vfloatx(zero),vfloatx(one));
            const Vec4vfx p = p0 + u*v;
            const vfloatx t = p.z*depth_scale;
            const vfloatx d2 = p.x*p.x + p.y*p.y; 
            const vfloatx r = p.w;
            const vfloatx r2 = r*r;
            valid &= d2 <= r2 & vfloatx(ray.tnear) < t & t < vfloatx(ray.tfar);

             /* update hit information */
            if (unlikely(any(valid))) {
              BezierHit<VSIZEX> hit(valid,u,0.0f,t,i,N,v0,v1,v2,v3);
              ishit |= epilog(valid,hit);
            }
          }
        }
        return ishit;
      }
    };
#endif

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
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int N,
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
        
        /* process SIMD-size many segments per iteration */
        bool ishit = false;
        for (int i=0; i<N; i+=VSIZEX)
        {
          /* evaluate the bezier curve */
          vboolx valid = vintx(i)+vintx(step) < vintx(N);
          const Vec4vfx p0 = curve2D.eval0(valid,i,N);
          const Vec4vfx p1 = curve2D.eval1(valid,i,N);
          
          /* approximative intersection with cone */
          const Vec4vfx v = p1-p0;
          const Vec4vfx w = -p0;
          const vfloatx d0 = w.x*v.x + w.y*v.y;
          const vfloatx d1 = v.x*v.x + v.y*v.y;
          const vfloatx u = clamp(d0*rcp(d1),vfloatx(zero),vfloatx(one));
          const Vec4vfx p = p0 + u*v;
          const vfloatx t = p.z*depth_scale[k];
          const vfloatx d2 = p.x*p.x + p.y*p.y; 
          const vfloatx r = p.w;
          const vfloatx r2 = r*r;
          valid &= d2 <= r2 & vfloatx(ray_tnear) < t & t < vfloatx(ray_tfar);
          if (likely(none(valid))) continue;
        
          /* update hit information */
          BezierHit<VSIZEX> hit(valid,u,0.0f,t,i,N,v0,v1,v2,v3);
          ishit |= epilog(valid,hit);
        }
        return ishit;
      }
    };
  }
}
