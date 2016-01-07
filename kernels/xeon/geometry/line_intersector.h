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
#include "filter.h"

namespace embree
{
  namespace isa
  {
    template<int M>
      struct LineIntersectorHitM
      {
        __forceinline LineIntersectorHitM(const vfloat<M>& u, const vfloat<M>& v, const vfloat<M>& t, const Vec3<vfloat<M>>& Ng)
          : vu(u), vv(v), vt(t), vNg(Ng) {}
        
        __forceinline void finalize() {}
        
        __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
        __forceinline float t  (const size_t i) const { return vt[i]; }
        __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }
        
      public:
        vfloat<M> vu;
        vfloat<M> vv;
        vfloat<M> vt;
        Vec3<vfloat<M>> vNg;
      };
    
    template<int M>
      struct LineIntersector1
      {
        typedef Vec3<vfloat<M>> Vec3vfM;
        typedef Vec4<vfloat<M>> Vec4vfM;
        
        struct Precalculations
        {
          __forceinline Precalculations (const Ray& ray, const void* ptr)
          {
            const float s = rsqrt(dot(ray.dir,ray.dir));
            depth_scale = s;
            ray_space = frame(s*ray.dir).transposed();
          }
          
          vfloat<M> depth_scale;
          LinearSpace3<Vec3vfM> ray_space;
        };

        static __forceinline vbool<M> intersect_sphere(vbool<M> valid, 
                                                       Ray& ray, const Vec4vfM& v0,
                                                       vfloat<M>& t_o,
                                                       Vec3<vfloat<M>>& Ng_o)
        {
          Vec3<vfloat<M>> P0 = Vec3<vfloat<M>>(v0.x,v0.y,v0.z);
          vfloat<M> r0 = v0.w;

          const Vec3<vfloat<M>> O = Vec3<vfloat<M>>(ray.org)-P0, dO = ray.dir;
          
          const vfloat<M> A = dot(dO,dO);
          const vfloat<M> B = 2.0f * dot(dO,O);
          const vfloat<M> C = dot(O,O) - sqr(r0);
          
          const vfloat<M> D = B*B - 4.0f*A*C;
          valid &= D >= 0.0f;
          if (none(valid)) return false;
          
          const vfloat<M> Q = sqrt(D);
          //const vfloat<M> t0 = (-B-Q)*rcp2A;
          //const vfloat<M> t1 = (-B+Q)*rcp2A;
          const vfloat<M> t0 = (-B-Q)/(2.0f*A);
          const vfloat<M> t = t0;
          valid &= (ray.tnear < t) & (t < ray.tfar);
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          const Vec3<vfloat<M>> Pr = Vec3<vfloat<M>>(ray.org) + t*Vec3<vfloat<M>>(ray.dir);
          t_o = t;
          Ng_o = Pr-P0;
          return valid;
        }

        static __forceinline vbool<M> intersect_cone(vbool<M> valid, 
                                                     Ray& ray, const Vec4vfM& v0, const Vec4vfM& v1,
                                                     vfloat<M>& t_o,
                                                     vfloat<M>& u_o,
                                                     Vec3<vfloat<M>>& Ng_o)
        {
          Vec4<vfloat<M>> a = v0;
          Vec4<vfloat<M>> b = v1;
          Vec3<vfloat<M>> a3(v0.x,v0.y,v0.z);
          Vec3<vfloat<M>> b3(v1.x,v1.y,v1.z);

          const vfloat<M> rl = rcp_length(b3-a3);
          const Vec3<vfloat<M>> P0 = a3, dP = (b3-a3)*rl;
          const vfloat<M> r0 = a.w, dr = (b.w-a.w)*rl;
          const Vec3<vfloat<M>> O = Vec3<vfloat<M>>(ray.org)-P0, dO = ray.dir;
          
          const vfloat<M> dOdO = dot(dO,dO);
          const vfloat<M> OdO = dot(dO,O);
          const vfloat<M> OO = dot(O,O);
          const vfloat<M> dOz = dot(dP,dO);
          const vfloat<M> Oz = dot(dP,O);

          const vfloat<M> R = r0 + Oz*dr;          
          const vfloat<M> A = dOdO - sqr(dOz) * (1.0f+sqr(dr));
          const vfloat<M> B = 2.0f * (OdO - dOz*(Oz + R*dr));
          const vfloat<M> C = OO - (sqr(Oz) + sqr(R));
          
          const vfloat<M> D = B*B - 4.0f*A*C;
          valid &= D >= 0.0f;
          if (none(valid)) return false;
          
          const vfloat<M> Q = sqrt(D);
          //const vfloat<M> t0 = (-B-Q)*rcp2A;
          //const vfloat<M> t1 = (-B+Q)*rcp2A;
          const vfloat<M> t0 = (-B-Q)/(2.0f*A);
          const vfloat<M> u0 = Oz+t0*dOz;
          const vfloat<M> t = t0;
          const vfloat<M> u = u0*rl;
          valid &= (ray.tnear < t) & (t < ray.tfar) & (0.0f <= u) & (u <= 1.0f); // FIXME: should not do uv test
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          const Vec3<vfloat<M>> Pr = Vec3<vfloat<M>>(ray.org) + t*Vec3<vfloat<M>>(ray.dir);
          const Vec3<vfloat<M>> Pl = a3 + u*(b3-a3);
          t_o = t;
          u_o = u;
          Ng_o = Pr-Pl;
          return valid;
        }

        static __forceinline std::pair<vfloat<M>,vfloat<M>> intersect_half_plane(Ray& ray, const Vec3<vfloat<M>>& N, const Vec3<vfloat<M>>& P)
        {
          Vec3<vfloat<M>> O = Vec3<vfloat<M>>(ray.org) - P;
          Vec3<vfloat<M>> D = Vec3<vfloat<M>>(ray.dir);
          vfloat<M> ON = dot(O,N);
          vfloat<M> DN = dot(D,N);
          vfloat<M> t = -ON*rcp(DN);
          vfloat<M> lower = select(DN < 0.0f, float(neg_inf), t);
          vfloat<M> upper = select(DN < 0.0f, t, float(pos_inf));
          return std::make_pair(lower,upper);
        }

        template<typename Epilog>
        static __forceinline bool intersect(Ray& ray, const Precalculations& pre,
                                            const vbool<M>& valid_i, const Vec4vfM& v0, const Vec4vfM& v1, const Vec4vfM& v2, const Vec4vfM& v3, 
                                            const Epilog& epilog)
        {
#if 0
          /* transform end points into ray space */
          Vec4vfM p0(xfmVector(pre.ray_space,v1.xyz()-Vec3vfM(ray.org)), v1.w);
          Vec4vfM p1(xfmVector(pre.ray_space,v2.xyz()-Vec3vfM(ray.org)), v2.w);

          /* approximative intersection with cone */
          const Vec4vfM v = p1-p0;
          const Vec4vfM w = -p0;
          const vfloat<M> d0 = w.x*v.x + w.y*v.y;
          const vfloat<M> d1 = v.x*v.x + v.y*v.y;
          const vfloat<M> u = clamp(d0*rcp(d1),vfloat<M>(zero),vfloat<M>(one));
          const Vec4vfM p = p0 + u*v;
          const vfloat<M> t = p.z*pre.depth_scale;
          const vfloat<M> d2 = p.x*p.x + p.y*p.y;
          const vfloat<M> r = p.w;
          const vfloat<M> r2 = r*r;
          vbool<M> valid = valid_i & d2 <= r2 & vfloat<M>(ray.tnear) < t & t < vfloat<M>(ray.tfar);
          if (unlikely(none(valid))) return false;
          
          /* ignore denormalized segments */
          const Vec3vfM T = v2.xyz()-v1.xyz();
          valid &= T.x != vfloat<M>(zero) | T.y != vfloat<M>(zero) | T.z != vfloat<M>(zero);
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          LineIntersectorHitM<M> hit(u,zero,t,T);
          return epilog(valid,hit);
#else
          Vec3<vfloat<M>> q0(v0.x,v0.y,v0.z);
          Vec3<vfloat<M>> q1(v1.x,v1.y,v1.z);
          Vec3<vfloat<M>> q2(v2.x,v2.y,v2.z);
          Vec3<vfloat<M>> q3(v3.x,v3.y,v3.z);
          
          auto tpl0 = intersect_half_plane(ray,q2-q0,q1);
          auto tpl1 = intersect_half_plane(ray,q1-q2,q1);

          auto tpr0 = intersect_half_plane(ray,q1-q3,q2);
          auto tpr1 = intersect_half_plane(ray,q2-q1,q2);

          vfloat<M> tl,ul; Vec3<vfloat<M>> Ngl; vbool<M> validl = intersect_sphere(valid_i,ray,v1,tl,Ngl); ul = 0.0f;
          validl &= max(tpl0.first,tpl1.first) <= tl & tl <= min(tpl0.second,tpl1.second);

          vfloat<M> t0,u0; Vec3<vfloat<M>> Ng0; vbool<M> valid0 = intersect_cone(valid_i,ray,v1,v2,t0,u0,Ng0);

          vfloat<M> tr,ur; Vec3<vfloat<M>> Ngr; vbool<M> validr = intersect_sphere(valid_i,ray,v2,tr,Ngr); ur = 1.0f;
          validr &= max(tpr0.first,tpr1.first) <= tr & tr <= min(tpr0.second,tpr1.second);

          //if (none(validr)) return false;
          //LineIntersectorHitM<M> hitr(ur,zero,tr,Ngr);
          //return epilog(validr,hitr);

          vbool<M> valid;
          vfloat<M> t,u; Vec3<vfloat<M>> Ng;
          valid = valid0; t = select(valid0,t0,float(inf)); u = u0; Ng = Ng0;

          vbool<M> va = !valid0 & validl & (tl < t);
          t = select(va, tl, t);
          vbool<M> vb = !valid0 & validr & (tr < t);
          t = select(vb, tr, t);

          u = select(va, ul, u);
          u = select(vb, ur, u);

          Ng.x = select(va, Ngl.x, Ng.x);
          Ng.x = select(vb, Ngr.x, Ng.x);

          Ng.y = select(va, Ngl.y, Ng.y);
          Ng.y = select(vb, Ngr.y, Ng.y);

          Ng.z = select(va, Ngl.z, Ng.z);
          Ng.z = select(vb, Ngr.z, Ng.z);

          valid = select(va, validl, valid);
          valid = select(vb, validr, valid);
          if (none(valid)) return false;
          
          LineIntersectorHitM<M> hit(u,zero,t,Ng);
          return epilog(valid,hit);

#endif
        }
      };
    
    template<int M, int K>
      struct LineIntersectorK
      {
        typedef Vec3<vfloat<M>> Vec3vfM;
        typedef Vec4<vfloat<M>> Vec4vfM;
        
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
          
          vfloat<K> depth_scale;
          LinearSpace3<Vec3vfM> ray_space[K];
        };
        
        template<typename Epilog>
        static __forceinline bool intersect(RayK<K>& ray, size_t k, const Precalculations& pre,
                                            const vbool<M>& valid_i, const Vec4vfM& v0, const Vec4vfM& v1,
                                            const Epilog& epilog)
        {
          /* transform end points into ray space */
          const Vec3vfM ray_org(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
          const Vec3vfM ray_dir(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
          Vec4vfM p0(xfmVector(pre.ray_space[k],v0.xyz()-ray_org), v0.w);
          Vec4vfM p1(xfmVector(pre.ray_space[k],v1.xyz()-ray_org), v1.w);
          
          /* approximative intersection with cone */
          const Vec4vfM v = p1-p0;
          const Vec4vfM w = -p0;
          const vfloat<M> d0 = w.x*v.x + w.y*v.y;
          const vfloat<M> d1 = v.x*v.x + v.y*v.y;
          const vfloat<M> u = clamp(d0*rcp(d1),vfloat<M>(zero),vfloat<M>(one));
          const Vec4vfM p = p0 + u*v;
          const vfloat<M> t = p.z*pre.depth_scale[k];
          const vfloat<M> d2 = p.x*p.x + p.y*p.y;
          const vfloat<M> r = p.w;
          const vfloat<M> r2 = r*r;
          vbool<M> valid = valid_i & d2 <= r2 & vfloat<M>(ray.tnear[k]) < t & t < vfloat<M>(ray.tfar[k]);
          if (unlikely(none(valid))) return false;
          
          /* ignore denormalized segments */
          const Vec3vfM T = v1.xyz()-v0.xyz();
          valid &= T.x != vfloat<M>(zero) | T.y != vfloat<M>(zero) | T.z != vfloat<M>(zero);
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          LineIntersectorHitM<M> hit(u,zero,t,T);
          return epilog(valid,hit);
        }
      };
  }
}
