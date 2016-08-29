// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
#include "plane.h"
#include "cone.h"

namespace embree
{
  namespace isa
  {
    struct FillCone
    {
      const Vec3fa p0_i; //!< start location
      const Vec3fa p1_i; //!< end position
      const Vec3fa n0;   //!< start direction
      const Vec3fa n1;   //!< end direction
      const float r0;    //!< start radius
      const float r1;    //!< end radius

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
        float dt = dot(p-q0,Ng);
        if (unlikely(std::isnan(dt))) return 1.0f;
        return dt;
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
        STAT(Stat::get().user[0]++); 
        
        /* move closer to geometry to make intersection stable */
        const Vec3fa C = 0.5f*(p0_i+p1_i);
        const Vec3fa ndir = normalize(ray.dir);
        const float dirlen = length(ray.dir);
        const float tb = dot(C-ray.org,ndir);
        const Vec3fa org = ray.org+tb*ndir;
        const Vec3fa dir = ray.dir;
        const Vec3fa p0 = p0_i-org;
        const Vec3fa p1 = p1_i-org;
        const float r01 = max(r0,r1);
        const float eps = 128.0f*1.19209e-07f*abs(tb);
        
        /* intersect with bounding cone */
        BBox1f tc;
        const Cone cone(p0,r01,p1,r01);
        if (!cone.intersect(dir,tc)) {
          STAT(Stat::get().user[1]++); 
          return false;
        }
        
        /* intersect with cap-planes */
        BBox1f tp(ray.tnear-tb,ray.tfar-tb);
        tp = embree::intersect(tp,HalfPlane(p0,+n0).intersect(zero,dir));
        tp = embree::intersect(tp,HalfPlane(p1,-n1)intersect(zero,dir));
        const BBox1f td = embree::intersect(tc,tp);
        if (td.lower > td.upper) {
          STAT(Stat::get().user[2]++); 
          return false;
        }
        
        tc.lower = tp.lower;
        tc.upper = min(tc.upper+0.1f*r01,tp.upper);

        /* shrink stepsize for distorted fill-cones */
        const Vec3fa p1p0 = p1-p0;
        const Vec3fa norm_p1p0 = normalize(p1p0);
        float A0 = abs(dot(norm_p1p0,normalize(n0)));
        float A1 = abs(dot(norm_p1p0,normalize(n1)));
        float rcpMaxDerivative = max(0.01f,min(A0,A1))/dirlen;
        
        STAT(Stat::get().user[3]++);
        t = td.lower;
        Vec3fa p = t*dir;
        float inout = 1.0f;
        
        for (size_t i=0; i<2000; i++) 
        {
          STAT(Stat::get().user[4]++); 
          
          if (unlikely(t > tc.upper)) {
            STAT(Stat::get().user[6]++); 
            break;
          }
          Vec3fa q0,q1,Ng;
          float dt = distance_f(p,q0,q1,Ng,p0,n0,r0,p1,n1,r1);
          if (i == 0 && dt < 0.0f) inout = -1.0f;
          t += rcpMaxDerivative*inout*dt;
          //if (p == t*d) break;
          p = t*dir;
          if (unlikely(abs(dt) < eps)) {
            STAT(Stat::get().user[7]++); 
            u = dot(p-q0,q1-q0)*rcp_length2(q1-q0);
            break;
          }
        }
        if (t < tc.lower || t > tc.upper) {
          return false;
        }

        t += tb;
        Ng = grad_distance_f(p,p0,n0,r0,p1,n1,r1);
        if (std::isnan(Ng.x) || std::isnan(Ng.y) || std::isnan(Ng.z)) return false;
        return true;
      }
    };
  }
}
