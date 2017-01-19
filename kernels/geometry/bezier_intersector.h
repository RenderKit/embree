// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/ray.h"
#include "filter.h"

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
        : U(U), V(V), T(T), i(i), N(N), p0(p0), p1(p1), p2(p2), p3(p3), valid(valid) {}
      
      __forceinline void finalize() 
      {
        vu = (vfloat<M>(step)+U+vfloat<M>(float(i)))*(1.0f/float(N));
        vv = V;
        vt = T;
      }
      
      __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
      __forceinline float t  (const size_t i) const { return vt[i]; }
      __forceinline Vec3fa Ng(const size_t i) const 
      { 
        Vec3fa T = BezierCurve3fa(p0,p1,p2,p3,0.0f,1.0f,0).eval_du(vu[i]);
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
    
    struct Bezier1Intersector1
    {
      float depth_scale;
      LinearSpace3fa ray_space;

      __forceinline Bezier1Intersector1() {}

      __forceinline Bezier1Intersector1(const Ray& ray, const void* ptr) 
         : depth_scale(rsqrt(dot(ray.dir,ray.dir))), ray_space(frame(depth_scale*ray.dir).transposed()) {}

      __forceinline vboolx intersect_triangle(const vboolx& valid0,
                                             const Vec3fa& ray_org,
                                             const Vec3fa& ray_dir,
                                             const float ray_tnear,
                                             const float ray_tfar,
                                             const Vec3vfx& tri_v0, 
                                             const Vec3vfx& tri_v1, 
                                             const Vec3vfx& tri_v2, 
                                             vfloatx& vu, 
                                             vfloatx& vv,
                                             vfloatx& vt) const
      {
        vboolx valid = valid0;
        /* calculate vertices relative to ray origin */
          const Vec3vfx O = Vec3vfx(ray_org);
          const Vec3vfx D = Vec3vfx(ray_dir);
          const Vec3vfx v0 = tri_v0-O;
          const Vec3vfx v1 = tri_v1-O;
          const Vec3vfx v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const Vec3vfx e0 = v2-v0;
          const Vec3vfx e1 = v0-v1;
          const Vec3vfx e2 = v1-v2;
          
          /* perform edge tests */
          const vfloatx U = dot(cross(v0,e0),D);
          const vfloatx V = dot(cross(v1,e1),D);
          const vfloatx W = dot(cross(v2,e2),D);

          const vfloatx maxUVW = max(U,V,W);
          valid &= maxUVW <= 0.0f;

          if (unlikely(none(valid))) return false;
          
          /* calculate geometry normal and denominator */
          const Vec3vfx Ng1 = cross(e1,e0);
          const Vec3vfx Ng = Ng1;
          const vfloatx den = dot(Ng,D);
          const vfloatx absDen = abs(den);
          const vfloatx sgnDen = signmsk(den);
          
          /* perform depth test */
          const vfloatx T = dot(v0,Ng);
          valid &= ((T^sgnDen) >= absDen*vfloatx(ray_tnear));
          valid &=(absDen*vfloatx(ray_tfar) >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloatx(zero);
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          const vfloatx rcpDen = rcp(den);
          vt = T * rcpDen;
          vu = U * rcpDen;
          vv = V * rcpDen;
          return valid;
      }

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
        vboolx valid = vfloatx(step) < vfloatx(float(N));
        const Vec4vfx p0 = curve2D.eval0(valid,0,N);
        const Vec4vfx p1 = curve2D.eval1(valid,0,N);
      
#if 1
        const Vec3vfx dp0dt = curve2D.derivative0(valid,0,N);
        const Vec3vfx dp1dt = curve2D.derivative1(valid,0,N);
        const Vec3vfx n0(dp0dt.y,-dp0dt.x,0.0f);
        const Vec3vfx n1(dp1dt.y,-dp1dt.x,0.0f);
        const Vec3vfx nn0 = normalize(n0);
        const Vec3vfx nn1 = normalize(n1);
        const Vec3vfx up0 = Vec3vfx(p0)+nn0*p0.w;
        const Vec3vfx up1 = Vec3vfx(p1)+nn1*p1.w;
        const Vec3vfx lp0 = Vec3vfx(p0)-nn0*p0.w;
        const Vec3vfx lp1 = Vec3vfx(p1)-nn1*p1.w;

        const Vec2vfx uv_lp0(0.0f,-1.0f);
        const Vec2vfx uv_lp1(1.0f,-1.0f);
        const Vec2vfx uv_up0(0.0f,+1.0f);
        const Vec2vfx uv_up1(1.0f,+1.0f);

        bool ishit = false;

        vfloatx vu,vv,vt;
        vboolx valid0 = intersect_triangle(valid,zero,Vec3fa(0,0,1),ray.tnear*depth_scale,ray.tfar*depth_scale,lp0,up0,lp1,vu,vv,vt);
        if (any(valid0))
        {
          const Vec2vfx uv = vu*uv_up0 + vv*uv_lp1 + (vfloatx(1.0f)-vu-vv)*uv_lp0;
          BezierHit<VSIZEX> bhit(valid0,uv.x,uv.y,depth_scale*vt,0,N,v0,v1,v2,v3);
          ishit |= epilog(bhit.valid,bhit);
        }

        vboolx valid1 = intersect_triangle(valid,zero,Vec3fa(0,0,1),ray.tnear*depth_scale,ray.tfar*depth_scale,up1,lp1,up0,vu,vv,vt);
        if (any(valid1))
        {
          const Vec2vfx uv = vu*uv_lp1 + vv*uv_up0 + (vfloatx(1.0f)-vu-vv)*uv_up1;
          BezierHit<VSIZEX> bhit(valid1,uv.x,uv.y,depth_scale*vt,0,N,v0,v1,v2,v3);
          ishit |= epilog(bhit.valid,bhit);
        }
        return ishit;
  
#else
        /* approximative intersection with cone */
        const Vec4vfx v = p1-p0;
        const Vec4vfx w = -p0;
        const vfloatx d0 = madd(w.x,v.x,w.y*v.y);
        const vfloatx d1 = madd(v.x,v.x,v.y*v.y);
        const vfloatx u = clamp(d0*rcp(d1),vfloatx(zero),vfloatx(one));
        const Vec4vfx p = madd(u,v,p0);
        const vfloatx t = p.z*depth_scale;
        const vfloatx d2 = madd(p.x,p.x,p.y*p.y); 
        const vfloatx r = p.w;
        const vfloatx r2 = r*r;
        valid &= (d2 <= r2) & (vfloatx(ray.tnear) < t) & (t < vfloatx(ray.tfar));

#if 0
        const Vec3vfx dp0dt = curve2D.derivative0(valid,0,N);
        const Vec3vfx dp1dt = curve2D.derivative1(valid,0,N);
        valid &= dot(Vec2vfx(p0.x,p0.y),Vec2vfx(dp0dt.x,dp0dt.y)) <= 0.0f;
        valid &= dot(Vec2vfx(p1.x,p1.y),Vec2vfx(dp1dt.x,dp1dt.y)) >= 0.0f;
#endif

#if 0
        const Vec4vfx p1p0 = p1-p0;
        const vfloatx side = p1p0.x*p0.y - p1p0.y*p0.x;
        const vfloatx sd2 = select(side<0.0f,vfloatx(1.0f),vfloatx(-1.0f));
#endif

#if 0
        for (size_t i=0; i<min(VSIZEX,N)-1; i++)
          if (d2[i] < d2[i+1]) valid[i+1] = 0;
#endif

        /* update hit information */
        bool ishit = false;
        if (unlikely(any(valid))) {
          //BezierHit<VSIZEX> hit(valid,u,sd2*sqrt(d2*rcp(r2)),t,0,N,v0,v1,v2,v3);
          BezierHit<VSIZEX> hit(valid,u,0.0f,t,0,N,v0,v1,v2,v3);
          ishit = ishit | epilog(valid,hit);
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
            const vfloatx d0 = madd(w.x,v.x,w.y*v.y);
            const vfloatx d1 = madd(v.x,v.x,v.y*v.y);
            const vfloatx u = clamp(d0*rcp(d1),vfloatx(zero),vfloatx(one));
            const Vec4vfx p = madd(u,v,p0);
            const vfloatx t = p.z*depth_scale;
            const vfloatx d2 = madd(p.x,p.x,p.y*p.y); 
            const vfloatx r = p.w;
            const vfloatx r2 = r*r;
            valid &= (d2 <= r2) & (vfloatx(ray.tnear) < t) & (t < vfloatx(ray.tfar));

             /* update hit information */
            if (unlikely(any(valid))) {
              BezierHit<VSIZEX> hit(valid,u,0.0f,t,i,N,v0,v1,v2,v3);
              ishit = ishit | epilog(valid,hit);
            }
          }
        }
        return ishit;
#endif
      }
    };

    template<int K>
    struct Bezier1IntersectorK
    {
      vfloat<K> depth_scale;
      LinearSpace3fa ray_space[K];

      __forceinline Bezier1IntersectorK(const vbool<K>& valid, const RayK<K>& ray) 
      {
        size_t mask = movemask(valid);
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
          const vfloatx d0 = madd(w.x,v.x,w.y*v.y);
          const vfloatx d1 = madd(v.x,v.x,v.y*v.y);
          const vfloatx u = clamp(d0*rcp(d1),vfloatx(zero),vfloatx(one));
          const Vec4vfx p = madd(u,v,p0);
          const vfloatx t = p.z*depth_scale[k];
          const vfloatx d2 = madd(p.x,p.x,p.y*p.y); 
          const vfloatx r = p.w;
          const vfloatx r2 = r*r;
          valid &= (d2 <= r2) & (vfloatx(ray_tnear) < t) & (t < vfloatx(ray_tfar));
          if (likely(none(valid))) continue;
        
          /* update hit information */
          BezierHit<VSIZEX> hit(valid,u,0.0f,t,i,N,v0,v1,v2,v3);
          ishit = ishit | epilog(valid,hit);
        }
        return ishit;
      }
    };
  }
}
