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
    struct Cone
    {
      const Vec3fa p0; //!< start location of cone
      const Vec3fa p1; //!< end position of cone
      const float r0;  //!< start radius of cone
      const float r1;  //!< end radius of cone

      __forceinline Cone(const Vec3fa& p0, const float r0, const Vec3fa& p1, const float r1) 
        : p0(p0), r0(r0), p1(p1), r1(r1) {}

      __forceinline bool intersect(const Vec3fa& org_i, const Vec3fa& dir, BBox1f& t_o, float& u0_o, Vec3fa& Ng0_o) const
        
      {
        const float tb = dot(0.5f*(p0+p1)-org_i,normalize(dir));
        const Vec3fa org = org_i+tb*dir;
        const Vec3fa v0 = p0-org;
        const Vec3fa v1 = p1-org;
        
        const float rl = rcp_length(v1-v0);
        const Vec3fa P0 = v0, dP = (v1-v0)*rl;
        const float dr = (r1-r0)*rl;
        const Vec3fa O = -P0, dO = dir;
        
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

        const float eps = float(1<<12)*float(ulp)*max(abs(dOdO),abs(sqr(dOz)));
        if (abs(A) < eps) 
        {
          const float t = -C/B;
          const float z0 = Oz+t*dOz;
          const float z0r = r0+z0*dr;
          if (z0r < 0.0f) return false;

          if (dOz > 0.0f) t_o = BBox1f(t,pos_inf);
          else            t_o = BBox1f(neg_inf,t);
        }
        else
        {
          const float Q = sqrt(D);
          const float rcp_2A = rcp(2.0f*A);
          t_o.lower = (-B-Q)*rcp_2A;
          t_o.upper = (-B+Q)*rcp_2A;
          
          if (A > 0.0f) {
            const float z0 = Oz+t_o.lower*dOz;
            const float z0r = r0+z0*dr;
            if (z0r < 0.0f) return false;
          } else {
            if (dOz > 0) t_o.upper = pos_inf;
            else         t_o.lower = neg_inf;
          }
        }

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
        const float dr = (r1-r0)*rl;
        const Vec3fa O = -P0, dO = dir;
       
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

      static void verify()
      {
        const Cone cone(Vec3fa(0.0f,0.0f,0.0f),0.0f,Vec3fa(1.0f,0.0f,0.0f),1.0f);
        const Ray ray0(Vec3fa(-2.0f,1.0f,0.0f),Vec3fa(+1.0f,+0.0f,+0.0f),0.0f,float(inf));
        const Ray ray1(Vec3fa(+2.0f,1.0f,0.0f),Vec3fa(-1.0f,+0.0f,+0.0f),0.0f,float(inf));
        const Ray ray2(Vec3fa(-1.0f,0.0f,2.0f),Vec3fa(+0.0f,+0.0f,-1.0f),0.0f,float(inf));
        const Ray ray3(Vec3fa(+1.0f,0.0f,2.0f),Vec3fa(+0.0f,+0.0f,-1.0f),0.0f,float(inf));
        const Ray ray4(Vec3fa(-1.0f,0.0f,0.0f),Vec3fa(+1.0f,+0.0f,+0.0f),0.0f,float(inf));
        const Ray ray5(Vec3fa(+1.0f,0.0f,0.0f),Vec3fa(-1.0f,+0.0f,+0.0f),0.0f,float(inf));
        const Ray ray6(Vec3fa(+0.0f,0.0f,1.0f),Vec3fa(+0.0f,+0.0f,-1.0f),0.0f,float(inf));
        const Ray ray7(Vec3fa(+0.0f,1.0f,0.0f),Vec3fa(-1.0f,-1.0f,+0.0f),0.0f,float(inf));
        const Ray ray8(Vec3fa(+0.0f,1.0f,0.0f),Vec3fa(+1.0f,-1.0f,+0.0f),0.0f,float(inf));
        const Ray ray9(Vec3fa(+0.0f,1.0f,0.0f),Vec3fa(-1.0f,+1.0f,+0.0f),0.0f,float(inf));

        float eps = 0.001f;

        BBox1f t; float u; Vec3fa Ng; bool hit;

        PRINT(0);
        hit = cone.intersect(ray0.org,ray0.dir,t,u,Ng);
        if (hit != true || abs(3.0f-t.lower) > eps || t.upper !=  float(inf)) {
          PRINT("error0");
          PRINT2(hit,t); 
        }
        std::cout << std::endl; 

        PRINT(1);
        hit = cone.intersect(ray1.org,ray1.dir,t,u,Ng);
        if (hit != true || t.lower != float(neg_inf) || abs(1.0f-t.upper) > eps)  {
          PRINT("error1");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 

        PRINT(2);
        hit = cone.intersect(ray2.org,ray2.dir,t,u,Ng);
        if (hit != false) {
          PRINT("error2");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 

        PRINT(3);
        hit = cone.intersect(ray3.org,ray3.dir,t,u,Ng);
        if (hit != true || abs(1.0f-t.lower) > eps || abs(3.0f-t.upper) > eps)  {
          PRINT("error3");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 

        PRINT(4);
        hit = cone.intersect(ray4.org,ray4.dir,t,u,Ng);
        if (hit != true || abs(1.0f-t.lower) > eps || t.upper != float(inf))  {
          PRINT("error4");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 
        
        PRINT(5);
        hit = cone.intersect(ray5.org,ray5.dir,t,u,Ng);
        if (hit != true || t.lower != float(neg_inf) || abs(1.0f-t.upper) > eps)  {
          PRINT("error5");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 
        
        PRINT(6);
        hit = cone.intersect(ray6.org,ray6.dir,t,u,Ng);
        if (hit != true || abs(1.0f-t.lower) > eps || abs(1.0f-t.upper) > eps)  {
          PRINT("error6");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 

        PRINT(7);
        hit = cone.intersect(ray7.org,ray7.dir,t,u,Ng);
        if (hit != false) {
          PRINT("error7");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 

        PRINT(8);
        hit = cone.intersect(ray8.org,ray8.dir,t,u,Ng);
        if (hit != true || abs(0.5f-t.lower) > eps || t.upper < 1e6f)  {
          PRINT("error8");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 

        PRINT(9);
        hit = cone.intersect(ray9.org,ray9.dir,t,u,Ng);
        if (hit != true || t.lower > -1e6f || abs(-0.5f-t.upper) > eps)  {
          PRINT("error9");
          PRINT2(hit,t);
        }
        std::cout << std::endl; 
      }
    };

    template<int N>
      struct ConeN
    {
      typedef Vec3<vfloat<N>> Vec3vfN;
      
      const Vec3vfN p0; //!< start location
      const Vec3vfN p1; //!< end position
      const vfloat<N> r0;   //!< start radius of cone
      const vfloat<N> r1;   //!< end radius of cone

      __forceinline ConeN(const Vec3vfN& p0, const vfloat<N>& r0, const Vec3vfN& p1, const vfloat<N>& r1) 
        : p0(p0), p1(p1), r0(r0), r1(r1) {}

      __forceinline Cone operator[] (const size_t i) 
      {
        assert(i<N);
        return Cone(Vec3fa(p0.x[i],p0.y[i].p0.z[i]),r0[i],Vec3fa(p1.x[i],p1.y[i].p1.z[i]),r1[i]);
      }

      __forceinline vbool<N> intersect(const Vec3fa& org_i, const Vec3fa& dir, BBox<vfloat<N>>& t_o, vfloat<N>& u0_o, Vec3vfN& Ng0_o) const
      {
        const vfloat<N> tb = dot(vfloat<N>(0.5f)*(p0+p1)-Vec3vfN(org_i),Vec3vfN(normalize(dir)));
        const Vec3vfN org = Vec3vfN(org_i)+tb*Vec3vfN(dir);
        const Vec3vfN v0 = p0-org;
        const Vec3vfN v1 = p1-org;

        const vfloat<N> rl = rcp_length(v1-v0);
        const Vec3vfN P0 = v0, dP = (v1-v0)*rl;
        const vfloat<N> dr = (r1-r0)*rl;
        const Vec3vfN O = -P0, dO = dir;
       
        const vfloat<N> dOdO = dot(dO,dO);
        const vfloat<N> OdO = dot(dO,O);
        const vfloat<N> OO = dot(O,O);
        const vfloat<N> dOz = dot(dP,dO);
        const vfloat<N> Oz = dot(dP,O);
        
        const vfloat<N> R = r0 + Oz*dr;          
        const vfloat<N> A = dOdO - sqr(dOz) * (vfloat<N>(1.0f)+sqr(dr));
        const vfloat<N> B = 2.0f * (OdO - dOz*(Oz + R*dr));
        const vfloat<N> C = OO - (sqr(Oz) + sqr(R));
        
        const vfloat<N> D = B*B - 4.0f*A*C;
        vbool<N> valid = D >= 0.0f;
        if (none(valid)) return valid;

        const vfloat<N> eps = float(1<<12)*float(ulp)*max(abs(dOdO),abs(sqr(dOz)));
        const vbool<N> validt = valid &  (abs(A) < eps);
        const vbool<N> validf = valid & !(abs(A) < eps);
        if (any(validt))
        {
          const vfloat<N> t = -C/B;
          const vfloat<N> z0 = Oz+t*dOz;
          const vfloat<N> z0r = r0+z0*dr;
          valid &= z0r >= 0.0f;
          t_o.lower = select(validt, select(dOz > 0.0f, t, vfloat<N>(neg_inf)), t_o.lower);
          t_o.upper = select(validt, select(dOz > 0.0f, vfloat<N>(pos_inf), t), t_o.upper);
        }

        if (any(validf))
        {
          const vfloat<N> Q = sqrt(D);
          const vfloat<N> rcp_2A = 0.5f*rcp(A);
          t_o.lower = (-B-Q)*rcp_2A;
          t_o.upper = (-B+Q)*rcp_2A;
          
          const vbool<N> validft = validf &   A>0.0f;
          const vbool<N> validff = validf & !(A>0.0f);
          if (any(validft)) {
            const vfloat<N> z0 = Oz+t_o.lower*dOz;
            const vfloat<N> z0r = r0+z0*dr;
            valid &= z0r >= 0.0f;
          } 

          if (any(validff)) {
            t_o.lower = select(validff, select(dOz > 0.0f, t_o.lower, float(neg_inf)), t_o.lower);
            t_o.upper = select(validff, select(dOz > 0.0f, float(pos_inf), t_o.upper), t_o.upper);
          }
        }

        u0_o = (Oz+t_o.lower*dOz)*rl;
        const Vec3vfN Pr = t_o.lower*Vec3vfN(dir);
        const Vec3vfN Pl = v0 + u0_o*(v1-v0);
        Ng0_o = Pr-Pl;
        t_o.lower += tb;
        t_o.upper += tb;
        return valid;
      }
    };
  }
}

