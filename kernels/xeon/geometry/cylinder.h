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

namespace embree
{
  namespace isa
  {
    struct Cylinder
    {
      const Vec3fa p0; //!< start location
      const Vec3fa p1; //!< end position
      const float r;   //!< radius of cylinder

      __forceinline Cylinder(const Vec3fa& p0, const Vec3fa& p1, const float r) 
        : p0(p0), p1(p1), r(r) {}

      __forceinline bool intersect(const Vec3fa& org_i, const Vec3fa& dir, BBox1f& t_o, float& u0_o, Vec3fa& Ng0_o) const
      {
        const float tb = dot(0.5f*(p0+p1)-org_i,normalize(dir));
        const Vec3fa org = org_i+tb*dir;
        const Vec3fa v0 = p0-org;
        const Vec3fa v1 = p1-org;
        
        const float rl = rcp_length(v1-v0);
        const Vec3fa P0 = v0, dP = (v1-v0)*rl;
        const Vec3fa O = -P0, dO = dir;
        
        const float dOdO = dot(dO,dO);
        const float OdO = dot(dO,O);
        const float OO = dot(O,O);
        const float dOz = dot(dP,dO);
        const float Oz = dot(dP,O);
        
        const float A = dOdO - sqr(dOz);
        const float B = 2.0f * (OdO - dOz*Oz);
        const float C = OO - sqr(Oz) - sqr(r0);
        
        const float D = B*B - 4.0f*A*C;
        if (D < 0.0f) return false;
        
        const float Q = sqrt(D);
        const float rcp_2A = rcp(2.0f*A);
        t_o.lower = (-B-Q)*rcp_2A;
        t_o.upper = (-B+Q)*rcp_2A;
        
        u0_o = (Oz+t_o.lower*dOz)*rl;
        const Vec3fa Pr = t_o.lower*dir;
        const Vec3fa Pl = v0 + u0_o*(v1-v0);
        Ng0_o = Pr-Pl;
        t_o.lower += tb;
        t_o.upper += tb;
        return true;
      }

      __forceinline bool intersect(const Vec3fa& dir, 
                                   BBox1f& t_o
                                   //float& u0_o,
                                   //Vec3fa& Ng_o
        ) const
      {
        const Vec3fa v0 = p0;
        const Vec3fa v1 = p1;
        const float rl = rcp_length(v1-v0);
        const Vec3fa P0 = v0, dP = (v1-v0)*rl;
        const Vec3fa O = -P0, dO = dir;
       
        const float dOdO = dot(dO,dO);
        const float OdO = dot(dO,O);
        const float OO = dot(O,O);
        const float dOz = dot(dP,dO);
        const float Oz = dot(dP,O);
        
        const float A = dOdO - sqr(dOz);
        const float B = 2.0f * (OdO - dOz*Oz);
        const float C = OO - sqr(Oz) - sqr(r0);
        
        const float D = B*B - 4.0f*A*C;
        if (D < 0.0f) return false;
        
        const float Q = sqrt(D);
        if (unlikely(A < min_rcp_input)) {
          t_o.lower = float(neg_inf);
          t_o.upper = float(pos_inf);
          return true;
        }
        
        const float rcp_2A = 0.5f*rcp(A);
        t_o.lower = (-B-Q)*rcp_2A;
        t_o.upper = (-B+Q)*rcp_2A;
        
        //u0_o = (Oz+t_o.lower*dOz)*rl;
        //const Vec3fa Pr = t_o.lower*dir;
        //const Vec3fa Pl = v0 + u0_o*(v1-v0);
        //Ng_o = Pr-Pl;
        return true;
      }
    };
  }
}

