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
        __forceinline LineIntersectorHitM() {}

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
          //valid &= (ray.tnear < t) & (t < ray.tfar);// & (0.0f <= u) & (u <= 1.0f); 
          //if (unlikely(none(valid))) return false;
          
          /* update hit information */
          const Vec3<vfloat<M>> Pr = Vec3<vfloat<M>>(ray.org) + t*Vec3<vfloat<M>>(ray.dir);
          const Vec3<vfloat<M>> Pl = a3 + u*(b3-a3);
          t_o = t;
          u_o = u;
          Ng_o = Pr-Pl;
          return valid;
        }

        static __forceinline bool intersect_cone(const Vec3fa& org, const Vec3fa& dir, 
                                                 const Vec3fa& v0, const float r0, 
                                                 const Vec3fa& v1, const float r1,
                                                 float& t0_o, float& t1_o)

        {
          const float rl = 1.0f/length(v1-v0);
          const Vec3fa P0 = v0, dP = (v1-v0)*rl;
          const float dr = (r1-r0)*rl;
          const Vec3fa O = org-P0, dO = dir;
          
          const float dOdO = dot(dO,dO);
          const float OdO = dot(dO,O);
          const float OO = dot(O,O);
          const float dOz = dot(dP,dO);
          const float Oz = dot(dP,O);

          const float R = r0 + Oz*dr;          
          const float A = dOdO - sqr(dOz) * (1.0f+sqr(dr));
          const float B = 2.0f * (OdO - dOz*(Oz + R*dr));
          const float C = OO - (sqr(Oz) + sqr(R));
          
          const float D = B*B - 4.0f*A*C;
          if (D < 0.0f) return false;
          
          const float Q = sqrt(D);
          t0_o = (-B-Q)/(2.0f*A);
          t1_o = (-B+Q)/(2.0f*A);
          return true;
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

        static __forceinline std::pair<float,float> intersect_half_plane(const Vec3fa& ray_org, const Vec3fa& ray_dir, const Vec3fa& N, const Vec3fa& P)
        {
          Vec3fa O = Vec3fa(ray_org) - P;
          Vec3fa D = Vec3fa(ray_dir);
          float ON = dot(O,N);
          float DN = dot(D,N);
          float t = -ON*rcp(DN);
          float lower = select(DN < 0.0f, float(neg_inf), t);
          float upper = select(DN < 0.0f, t, float(pos_inf));
          return std::make_pair(lower,upper);
        }

        static __forceinline Vec3fa distributator(const Vec3fa& A, const Vec3fa& B, const Vec3fa& C) {
          return dot(A,B)*C - A*dot(B,C);
        }

        static __forceinline float f(const float u, 
                                     const Vec3fa& d,
                                     const Vec3fa& p0, const Vec3fa& n0, const float r0,
                                     const Vec3fa& p1, const Vec3fa& n1, const float r1)
        {
          const Vec3fa ps = (1.0f-u)*p0 + u*p1;
          const Vec3fa ns = (1.0f-u)*n0 + u*n1;
          const float  rs = (1.0f-u)*r0 + u*r1;
          const Vec3fa A = distributator(ps,ns,d);
          const float  B = rs*dot(ns,d);
          return dot(A,A) - sqr(B);
        }

        static __forceinline float dfds(const float u,
                                        const Vec3fa& d,
                                        const Vec3fa& p0, const Vec3fa& n0, const float r0,
                                        const Vec3fa& p1, const Vec3fa& n1, const float r1)
        {
          const Vec3fa ps = (1.0f-u)*p0 + u*p1, dps = p1-p0;
          const Vec3fa ns = (1.0f-u)*n0 + u*n1, dns = n1-n0;
          const float  rs = (1.0f-u)*r0 + u*r1, drs = r1-r0;
          const Vec3fa A  = distributator(ps,ns,d);
          const Vec3fa dA = distributator(dps,ns,d)+distributator(ps,dns,d);
          const float  B  = rs*dot(ns,d);
          const float  dB = drs*dot(ns,d) + rs*dot(dns,d);
          return 2.0f*dot(dA,A) - 2.0f*dB*B;
        }

        static __forceinline bool intersect_iterative1(const Vec3fa& d,
                                                      const Vec3fa& p0, const Vec3fa& n0, const float r0,
                                                      const Vec3fa& p1, const Vec3fa& n1, const float r1,
                                                      float& u)
        {
          float u0 = 0.0f;
          for (int i=0; i<20; i++) {
            const float fu = f(u0,d,p0,n0,r0,p1,n1,r1);
            const float dfu = dfds(u0,d,p0,n0,r0,p1,n1,r1);
            u0 = u0 - fu/dfu;
          }
          if (f(u0,d,p0,n0,r0,p1,n1,r1) > 0.01f) u0 = 2;
          const Vec3fa ps0 = (1.0f-u0)*p0 + u0*p1;
          const Vec3fa ns0 = (1.0f-u0)*n0 + u0*n1;
          float t0 = dot(ps0,ns0)/dot(d,ns0);
          if (u0 < 0.0f || u0 > 1.0f) t0 = inf;
          
          float u1 = 1.0f;
          for (int i=0; i<20; i++) {
            const float fu = f(u1,d,p0,n0,r0,p1,n1,r1);
            const float dfu = dfds(u1,d,p0,n0,r0,p1,n1,r1);
            u1 = u1 - fu/dfu;
          }
          if (f(u1,d,p0,n0,r0,p1,n1,r1) > 0.01f) u1 = 2;
          const Vec3fa ps1 = (1.0f-u1)*p0 + u1*p1;
          const Vec3fa ns1 = (1.0f-u1)*n0 + u1*n1;
          float t1 = dot(ps1,ns1)/dot(d,ns1);
          if (u1 < 0.0f || u1 > 1.0f) t1 = inf;

          if (t0 < t1) {
            u = u0;
          } else {
            u = u1;
          }
          
          return u >= 0.0f && u <= 1.0f;
        }

        static __forceinline bool intersect_iterative2(const Ray& ray,
                                                       const Vec3fa& p0_i, const Vec3fa& n0, const float r0,
                                                       const Vec3fa& p1_i, const Vec3fa& n1, const float r1,
                                                       float& u, float& t, Vec3fa& Ng)
        {
          const Vec3fa p0 = p0_i-ray.org;
          const Vec3fa p1 = p1_i-ray.org;
          const Vec3fa d = ray.dir;
          auto tp0 = intersect_half_plane(zero,d,+n0,p0);
          auto tp1 = intersect_half_plane(zero,d,-n1,p1);

          float t_term = 0.01f;
          const float r01 = max(r0,r1)+t_term;
          float tc_lower,tc_upper;
          if (!intersect_cone(zero,d,p0,r01,p1,r01,tc_lower,tc_upper))
            return false;

          tc_lower = max(tc_lower,tp0.first ,tp1.first );
          tc_upper = min(tc_upper,tp0.second,tp1.second);

          t = tc_lower; float dt = inf;
          Vec3fa p = t*d;
          for (size_t i=0; i<200; i++) 
          {
            if (t > tc_upper) break;
            const Vec3fa N = cross(p-p0,p1-p0);
            const Vec3fa q0 = p0+r0*normalize(cross(n0,N));
            const Vec3fa q1 = p1+r1*normalize(cross(n1,N));
            dt = dot(p-q0,normalize(cross(q1-q0,N)));
            t += dt;
            p = t*d;
            if (unlikely(dt < t_term)) {
              u = dot(p-q0,normalize(q1-q0))/length(q1-q0);
              Ng = cross(q1-q0,N);
              break;
            }
          }
          if (t < tc_lower  || t > tc_upper) return false;
          if (t < ray.tnear || t > ray.tfar) return false;
          return true;
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
#endif

#if 0
          Vec3<vfloat<M>> q0(v0.x,v0.y,v0.z);
          Vec3<vfloat<M>> q1(v1.x,v1.y,v1.z);
          Vec3<vfloat<M>> q2(v2.x,v2.y,v2.z);
          Vec3<vfloat<M>> q3(v3.x,v3.y,v3.z);

          auto Hl = normalize_safe(q1-q0) + normalize_safe(q2-q1);
          auto Hr = normalize_safe(q1-q2) + normalize_safe(q2-q3);
          
          vfloat<M> t1,u1; Vec3<vfloat<M>> Ng1; vbool<M> valid1 = intersect_cone(valid_i,ray,v1,v2,t1,u1,Ng1);
          valid1 &= (ray.tnear < t1) & (t1 < ray.tfar);

          vfloat<M> tl,ul; Vec3<vfloat<M>> Ngl; vbool<M> validl = intersect_sphere(valid_i,ray,v1,tl,Ngl); ul = 0.0f;
          auto left = u1 < 0.0f;
          valid1 = select(left,validl,valid1);
          t1 = select(left, tl, t1);
          u1 = select(left, ul, u1);
          Ng1.x = select(left, Ngl.x, Ng1.x);
          Ng1.y = select(left, Ngl.y, Ng1.y);
          Ng1.z = select(left, Ngl.z, Ng1.z);

          vfloat<M> tr,ur; Vec3<vfloat<M>> Ngr; vbool<M> validr = intersect_sphere(valid_i,ray,v2,tr,Ngr); ur = 1.0f;
          auto right = u1 > 1.0f;
          valid1 = select(right,validr,valid1);
          t1 = select(right, tr, t1);
          u1 = select(right, ur, u1);
          Ng1.x = select(right, Ngr.x, Ng1.x);
          Ng1.y = select(right, Ngr.y, Ng1.y);
          Ng1.z = select(right, Ngr.z, Ng1.z);

          auto thl = intersect_half_plane(ray,Hl,q1);
          valid1 &= thl.first <= t1 & t1 <= thl.second;

          auto thr = intersect_half_plane(ray,Hr,q2);
          valid1 &= thr.first <= t1 & t1 <= thr.second;
          
          if (none(valid1)) return false;
          LineIntersectorHitM<M> hit1(u1,zero,t1,Ng1);
          return epilog(valid1,hit1);

#endif

#if 0

          Vec3<vfloat<M>> q0(v0.x,v0.y,v0.z);
          Vec3<vfloat<M>> q1(v1.x,v1.y,v1.z);
          Vec3<vfloat<M>> q2(v2.x,v2.y,v2.z);
          Vec3<vfloat<M>> q3(v3.x,v3.y,v3.z);

          auto Hl = normalize_safe(q1-q0) + normalize_safe(q2-q1);
          auto Hr = normalize_safe(q1-q2) + normalize_safe(q2-q3);
          
          auto tpl0 = intersect_half_plane(ray,Hl,q1);
          auto tpl1 = intersect_half_plane(ray,q1-q2,q1);

          auto tpr0 = intersect_half_plane(ray,Hr,q2);
          auto tpr1 = intersect_half_plane(ray,q2-q1,q2);

          vfloat<M> tl,ul; Vec3<vfloat<M>> Ngl; vbool<M> validl = intersect_sphere(valid_i,ray,v1,tl,Ngl); ul = 0.0f;
          validl &= max(tpl0.first,tpl1.first) <= tl & tl <= min(tpl0.second,tpl1.second);

          //vfloat<M> t0,u0; Vec3<vfloat<M>> Ng0; vbool<M> valid0 = intersect_cone(valid_i,ray,v0,v1,t0,u0,Ng0);
          //valid0 &= (0.0f <= u0) & (u0 <= 1.0f);
          vfloat<M> t1,u1; Vec3<vfloat<M>> Ng1; vbool<M> valid1 = intersect_cone(valid_i,ray,v1,v2,t1,u1,Ng1);
          valid1 &= (0.0f <= u1) & (u1 <= 1.0f);
          //vfloat<M> t2,u2; Vec3<vfloat<M>> Ng2; vbool<M> valid2 = intersect_cone(valid_i,ray,v2,v3,t2,u2,Ng2);
          //valid2 &= (0.0f <= u2) & (u2 <= 1.0f);

          //valid1 &= !valid0 | (t0 > t1);
          //valid1 &= !valid2 | (t2 > t1);
          valid1 &= (ray.tnear < t1) & (t1 < ray.tfar);

          vfloat<M> tr,ur; Vec3<vfloat<M>> Ngr; vbool<M> validr = intersect_sphere(valid_i,ray,v2,tr,Ngr); ur = 1.0f;
          validr &= max(tpr0.first,tpr1.first) <= tr & tr <= min(tpr0.second,tpr1.second);

          //if (none(validr)) return false;
          //LineIntersectorHitM<M> hitr(ur,zero,tr,Ngr);
          //return epilog(validr,hitr);

          vbool<M> valid;
          vfloat<M> t,u; Vec3<vfloat<M>> Ng;
          valid = valid1; t = select(valid1,t1,float(inf)); u = u1; Ng = Ng1;

          vbool<M> va = !valid1 & validl & (tl < t);
          t = select(va, tl, t);
          vbool<M> vb = !valid1 & validr & (tr < t);
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

#if 1
          vbool<M> valid = false;
          LineIntersectorHitM<M> hit;
          
          for (size_t i=0; i<M; i++)
          {
            if (!valid_i[i]) continue;
            const Vec3fa p0(v0.x[i],v0.y[i],v0.z[i]);
            const Vec3fa p1(v1.x[i],v1.y[i],v1.z[i]);
            const Vec3fa p2(v2.x[i],v2.y[i],v2.z[i]);
            const Vec3fa p3(v3.x[i],v3.y[i],v3.z[i]);
            const Vec3fa n1 = normalize_safe(p1-p0) + normalize_safe(p2-p1);
            const Vec3fa n2 = normalize_safe(p2-p1) + normalize_safe(p3-p2);
            float u = 0.0f;
#if 1
            float t = 0.0f;
            Vec3fa Ng = zero;
            if (!intersect_iterative2(ray,p1,n1,v1.w[i],p2,n2,v2.w[i],u,t,Ng)) continue;
#else
            if (!intersect_iterative1(ray.dir,p1-ray.org,n1,v1.w[i],p2-ray.org,n2,v2.w[i],u)) continue;
            const Vec3fa ps = (1.0f-u)*p1 + u*p2;
            const Vec3fa ns = (1.0f-u)*n1 + u*n2;
            const float t = dot(ps-ray.org,ns)/dot(ray.dir,ns);
            const Vec3fa os = ray.org+t*ray.dir;
            const Vec3fa Ng = os-ps;
#endif
            hit.vu[i] = u;
            hit.vv[i] = 0.0f;
            hit.vt[i] = t;
            hit.vNg.x[i] = Ng.x;
            hit.vNg.y[i] = Ng.y;
            hit.vNg.z[i] = Ng.z;
            valid[i] = 0xFFFFFFFF;
          }
          if (none(valid)) return false;
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
