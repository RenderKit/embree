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
#include "cone.h"

namespace embree
{
  namespace isa
  {
    static __forceinline BBox1f intersect_half_plane(const Vec3fa& ray_org, const Vec3fa& ray_dir, const Vec3fa& N, const Vec3fa& P)
    {
      Vec3fa O = Vec3fa(ray_org) - P;
      Vec3fa D = Vec3fa(ray_dir);
      float ON = dot(O,N);
      float DN = dot(D,N);
      float t = -ON*rcp(abs(DN) < min_rcp_input ? min_rcp_input : DN );
      float lower = select(DN < 0.0f, float(neg_inf), t);
      float upper = select(DN < 0.0f, t, float(pos_inf));
      return BBox1f(lower,upper);
    }

    struct FillCone
    {
      const Vec3fa p0_i; //!< start location
      const Vec3fa p1_i; //!< end position
      const Vec3fa n0; //!< start direction
      const Vec3fa n1; //!< end direction
      const float r0;  //!< start radius
      const float r1;  //!< end radius

      __forceinline FillCone(const Vec3fa& p0, const Vec3fa& n0, const float r0, const Vec3fa& p1, const Vec3fa& n1, const float r1) 
        : p0_i(p0), n0(n0), r0(r0), p1_i(p1), n1(n1), r1(r1) {}

      __forceinline float distance_f(const Vec3fa& p,Vec3fa& q0, Vec3fa& q1, Vec3fa& Ng,
                                     const Vec3fa& p0, const Vec3fa& n0, const float r0,
                                     const Vec3fa& p1, const Vec3fa& n1, const float r1) const
      {
        const Vec3fa N = cross(p-p0,p1-p0);
#if defined (__AVX__)
        const vfloat<8> p0p1 = vfloat<8>(vfloat<4>(p0),vfloat<4>(p1));
        const vfloat<8> n0n1 = vfloat<8>(vfloat<4>(n0),vfloat<4>(n1));
        const vfloat<8> r0r1 = vfloat<8>(vfloat<4>(r0),vfloat<4>(r1));
        const vfloat<8> NN   = vfloat<8>(vfloat<4>(N));
        const vfloat<8> q0q1 = p0p1 + r0r1*normalize(cross(n0n1,NN));
        q0 = (Vec3fa)extract4<0>(q0q1);
        q1 = (Vec3fa)extract4<1>(q0q1);
#else
        q0 = p0+r0*normalize(cross(n0,N));
        q1 = p1+r1*normalize(cross(n1,N));
#endif
        Ng = normalize(cross(q1-q0,N));
        return dot(p-q0,Ng);
      }
      
      __forceinline float distance_f(const Vec3fa& p,
                                     const Vec3fa& p0, const Vec3fa& n0, const float r0,
                                     const Vec3fa& p1, const Vec3fa& n1, const float r1) const
      {
        Vec3fa q0; Vec3fa q1; Vec3fa Ng;
        return distance_f(p,q0,q1,Ng,p0,n0,r0,p1,n1,r1);
      }

      __forceinline Vec3fa grad_distance_f(const Vec3fa& p, 
                                           const Vec3fa& p0, const Vec3fa& n0, const float r0,
                                           const Vec3fa& p1, const Vec3fa& n1, const float r1) const
      {
        const Vec3fa N    = cross(p-p0,p1-p0);
        const Vec3fa dNdx = cross(Vec3fa(1,0,0),p1-p0);
        const Vec3fa dNdy = cross(Vec3fa(0,1,0),p1-p0);
        const Vec3fa dNdz = cross(Vec3fa(0,0,1),p1-p0);
        
        const Vec3fa N0    = cross(n0,N);
        const Vec3fa dN0dx = cross(n0,dNdx); 
        const Vec3fa dN0dy = cross(n0,dNdy); 
        const Vec3fa dN0dz = cross(n0,dNdz); 
        
        const Vec3fa N1    = cross(n1,N);
        const Vec3fa dN1dx = cross(n1,dNdx); 
        const Vec3fa dN1dy = cross(n1,dNdy); 
        const Vec3fa dN1dz = cross(n1,dNdz); 
        
        const Vec3fa q0    = p0+r0*normalize(N0);
        const Vec3fa dq0dx = r0*dnormalize(N0,dN0dx);
        const Vec3fa dq0dy = r0*dnormalize(N0,dN0dy);
        const Vec3fa dq0dz = r0*dnormalize(N0,dN0dz);
        
        const Vec3fa q1    = p1+r1*normalize(N1);
        const Vec3fa dq1dx = r1*dnormalize(N1,dN1dx);
        const Vec3fa dq1dy = r1*dnormalize(N1,dN1dy);
        const Vec3fa dq1dz = r1*dnormalize(N1,dN1dz);
        
        const Vec3fa K    = cross(q1-q0,N);
        const Vec3fa dKdx = cross(dq1dx-dq0dx,N) + cross(q1-q0,dNdx);
        const Vec3fa dKdy = cross(dq1dy-dq0dy,N) + cross(q1-q0,dNdy);
        const Vec3fa dKdz = cross(dq1dz-dq0dz,N) + cross(q1-q0,dNdz);
        
        const Vec3fa Ng    = normalize(K);
        const Vec3fa dNgdx = dnormalize(K,dKdx); 
        const Vec3fa dNgdy = dnormalize(K,dKdy); 
        const Vec3fa dNgdz = dnormalize(K,dKdz); 
        
        const float f    = dot(p-q0,Ng);
        const float dfdx = dot(Vec3fa(1,0,0)-dq0dx,Ng) + dot(p-q0,dNgdx); 
        const float dfdy = dot(Vec3fa(0,1,0)-dq0dy,Ng) + dot(p-q0,dNgdy); 
        const float dfdz = dot(Vec3fa(0,0,1)-dq0dz,Ng) + dot(p-q0,dNgdz); 
        
        return Vec3fa(dfdx,dfdy,dfdz);
      }
      
      __forceinline bool intersect(const Ray& ray, float& u, float& t, Vec3fa& Ng) const
      {
        //PING;
        STAT(Stat::get().user[0]++); 
        
        /* move closer to geometry to make intersection stable */
        const Vec3fa C = 0.5f*(p0_i+p1_i);
        const float tb = dot(C-ray.org,normalize(ray.dir));
        const Vec3fa org = ray.org+tb*normalize(ray.dir);
        const float dirlen = length(ray.dir);
        const Vec3fa dir = ray.dir;
        const Vec3fa p0 = p0_i-org;
        const Vec3fa p1 = p1_i-org;
        const float ray_tnear = ray.tnear-tb;
        const float ray_tfar = ray.tfar-tb;
        
        ////PRINT(ray_tnear);
        ////PRINT(ray_tfar);
        //PRINT(r0);
        //PRINT(r1);
        float maxR = max(r0,r1);
        //PRINT(maxR);
        float t_term = 128.0f*1.19209e-07f*abs(tb);
        
        const float r01 = maxR;
        float tc_lower,tc_upper;
        Vec3fa NgA; float uA;
        const Cone cone(p0,r01,p1,r01);
        if (!cone.intersect(dir,tc_lower,tc_upper,uA,NgA)) {
          STAT(Stat::get().user[1]++); 
          return false;
        }
        //PRINT(tc_lower);
        //PRINT(tc_upper);
        
        BBox1f tp0 = intersect_half_plane(zero,dir,+n0,p0);
        BBox1f tp1 = intersect_half_plane(zero,dir,-n1,p1);
        ////PRINT(tp0.first);
        ////PRINT(tp0.second);
        ////PRINT(tp1.first);
        ////PRINT(tp1.second);
        
        float td_lower = max(ray_tnear,tc_lower,tp0.lower ,tp1.lower );
        float td_upper = min(ray_tfar,tc_upper,tp0.upper,tp1.upper);
        //PRINT(td_lower);
        //PRINT(td_upper);
        if (td_lower > td_upper) {
          STAT(Stat::get().user[2]++); 
          return false;
        }
        
#if 0
        if (uA >= 0.0f && uA <= 1.0f) {
          u = uA;
          t = tb+tc_lower;
          Ng = NgA;
          return true;
        }
        return false;
#endif
        
        tc_lower = max(ray_tnear,tp0.lower ,tp1.lower );
        tc_upper = min(ray_tfar,tc_upper+0.1f*maxR,tp0.upper,tp1.upper);
        
        const Vec3fa p1p0 = p1-p0;
        const Vec3fa norm_p1p0 = normalize(p1p0);
        
        float A0 = abs(dot(norm_p1p0,normalize(n0)));
        float A1 = abs(dot(norm_p1p0,normalize(n1)));
        float rcpMaxDerivative = max(0.01f,min(A0,A1))/dirlen;
        
        //PRINT(tc_lower);
        //PRINT(tc_upper);
        
        STAT(Stat::get().user[3]++);
        t = td_lower; float dt = inf;
        Vec3fa p = t*dir;
        float inout = 1.0f;
        
        for (size_t i=0;; i++) 
        {
          //PRINT(i);
          STAT(Stat::get().user[4]++); 
          if (unlikely(i == 2000)) {
            STAT(Stat::get().user[5]++); 
            //PRINT("miss1");
            return false;
          }
          if (unlikely(t > tc_upper)) {
            STAT(Stat::get().user[6]++); 
            //PRINT("break1");
            break;
          }
          Vec3fa q0,q1,Ng;
          dt = distance_f(p,q0,q1,Ng,p0,n0,r0,p1,n1,r1);
          if (i == 0 && dt < 0.0f) inout = -1.0f;
          dt *= inout;
          //PRINT(dt);
          if (unlikely(std::isnan(dt))) {
            //PRINT("miss2");
            dt = 1.0f;
            //return false;
          }
          t += rcpMaxDerivative*dt;
          //if (p == t*d) break;
          p = t*dir;
          //PRINT(t);
          //PRINT(p);
          if (unlikely(abs(dt) < t_term)) {
            //PRINT("break2");
            STAT(Stat::get().user[7]++); 
            u = dot(p-q0,q1-q0)*rcp_length2(q1-q0);
            break;
          }
        }
        if (t < tc_lower || t > tc_upper) {
          //PRINT("miss3");
          return false;
        }
        
        /*const Vec3fa N = cross(p-p0,p1p0);
        const Vec3fa Ng0 = normalize(cross(n0,N));
        const Vec3fa Ng1 = normalize(cross(n1,N));
        const Vec3fa q0 = p0+r0*Ng0;
        const Vec3fa q1 = p1+r1*Ng1;
        Ng = normalize(cross(q1-q0,N));
        const Vec3fa P = (1.0f-u)*q0 + u*q1;
        t = tb+dot(q0,Ng)/dot(dir,Ng);*/
        t += tb;
        Ng = grad_distance_f(p,p0,n0,r0,p1,n1,r1);
        if (std::isnan(Ng.x) || std::isnan(Ng.y) || std::isnan(Ng.z)) return false;

        //PRINT("hit");
        //PRINT(t);
        //PRINT(Ng);
        //PRINT(P);
        return true;
      }
    };
  }
}
