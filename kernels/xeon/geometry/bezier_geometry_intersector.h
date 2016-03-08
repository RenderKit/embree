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
#include "cylinder.h"
#include "cone.h"
#include "plane.h"
#include "line_intersector.h"

namespace embree
{
  namespace isa
  {
    static const size_t numJacobianIterations = 5;
#if defined(__AVX512F__)
    static const size_t numBezierSubdivisions = 2;
#elif defined(__AVX__)
    static const size_t numBezierSubdivisions = 2;
#else
    static const size_t numBezierSubdivisions = 3;
#endif

    struct BezierGeometryHit
    {
      __forceinline BezierGeometryHit() {}

      __forceinline BezierGeometryHit(const float t, const float u, const Vec3fa& Ng)
        : t(t), u(u), v(0.0f), Ng(Ng) {}
      
      __forceinline void finalize() {}
      
    public:
      float t;
      float u;
      float v; 
      Vec3fa Ng;
    };
    
    template<typename Epilog>
    __forceinline bool intersect_bezier_iterative_debug(const Ray& ray, const float dt, const BezierCurve3fa& curve, size_t i, 
                                                        const vfloatx& u, const BBox<vfloatx>& tp, const BBox<vfloatx>& h0, const BBox<vfloatx>& h1, 
                                                        const Vec3vfx& Ng, const Vec4vfx& dP0du, const Vec4vfx& dP3du,
                                                        const Epilog& epilog)
    {
      if (tp.lower[i]+dt >= ray.tfar) return false;
      Vec3fa Ng_o = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
      if (h0.lower[i] == tp.lower[i]) Ng_o = -Vec3fa(dP0du.x[i],dP0du.y[i],dP0du.z[i]);
      if (h1.lower[i] == tp.lower[i]) Ng_o = +Vec3fa(dP3du.x[i],dP3du.y[i],dP3du.z[i]);
      BezierGeometryHit hit(tp.lower[i]+dt,u[i],Ng_o);
      return epilog(hit);
    }

   template<typename Epilog> 
     __forceinline bool intersect_bezier_iterative_jacobian(const Ray& ray, const float dt, const BezierCurve3fa& curve, float u, float t, const Epilog& epilog)
    {
      const Vec3fa org = zero;
      const Vec3fa dir = ray.dir;

      const float length_ray_dir = length(dir);
      for (size_t i=0; i<numJacobianIterations; i++) 
      {
        const Vec3fa Q = org + t*dir;
        //const Vec3fa dQdu = zero;
        const Vec3fa dQdt = dir;

        Vec3fa P,dPdu,ddPdu; curve.eval(u,P,dPdu,ddPdu);
        //const Vec3fa dPdt = zero;

        const Vec3fa R = Q-P;
        const Vec3fa dRdu = /*dQdu*/-dPdu;
        const Vec3fa dRdt = dQdt;//-dPdt;

        const Vec3fa T = normalize(dPdu);
        const Vec3fa dTdu = dnormalize(dPdu,ddPdu);
        //const Vec3fa dTdt = zero;

        const float f = dot(R,T);
        const float dfdu = dot(dRdu,T) + dot(R,dTdu);
        const float dfdt = dot(dRdt,T);// + dot(R,dTdt);

        const float K = dot(R,R)-sqr(f);
        const float dKdu = /*2.0f*/(dot(R,dRdu)-f*dfdu);
        const float dKdt = /*2.0f*/(dot(R,dRdt)-f*dfdt);
        const float rsqrt_K = rsqrt(K);

        const float g = sqrt(K)-P.w;
        const float dgdu = /*0.5f*/dKdu*rsqrt_K-dPdu.w;
        const float dgdt = /*0.5f*/dKdt*rsqrt_K;//-dPdt.w;

        const LinearSpace2f J = LinearSpace2f(dfdu,dfdt,dgdu,dgdt);
        const Vec2f dut = rcp(J)*Vec2f(f,g);
        const Vec2f ut = Vec2f(u,t) - dut;
        u = ut.x; t = ut.y;
        
        const bool converged_u = abs(f) < 16.0f*float(ulp)*reduce_max(abs(dPdu));
        const bool converged_t = abs(g) < 16.0f*float(ulp)*length_ray_dir;
        if (converged_u && converged_t) 
        {
          t+=dt;
          if (t < ray.tnear || t > ray.tfar) return false;
          if (u < 0.0f || u > 1.0f) return false;
          const Vec3fa R = normalize(Q-P);
          const Vec3fa U = dPdu+dPdu.w*R;
          const Vec3fa V = cross(dPdu,R);
          BezierGeometryHit hit(t,u,cross(V,U));
          return epilog(hit);
        }
      }
      return false;
    }

    template<typename Epilog>
      __forceinline bool intersect_bezier_recursive_jacobian(const Ray& ray, const float dt, const BezierCurve3fa& curve, const float u0, const float u1, const size_t depth, const Epilog& epilog)
    {
      int maxDepth = numBezierSubdivisions;
      //int maxDepth = Device::debug_int1+1;
      const Vec3fa org = zero;
      const Vec3fa dir = ray.dir;

      /* subdivide curve */
      const float dscale = (u1-u0)*(1.0f/(3.0f*(VSIZEX-1)));
      const vfloatx vu0 = lerp(u0,u1,vfloatx(step)*(1.0f/(VSIZEX-1)));
      Vec4vfx P0, dP0du; curve.evalN(vu0,P0,dP0du); dP0du = dP0du * Vec4vfx(dscale);
      const Vec4vfx P3 = shift_right_1(P0);
      const Vec4vfx dP3du = Vec4vfx(dscale)*shift_right_1(dP0du); 
      const Vec4vfx P1 = P0 + dP0du; 
      const Vec4vfx P2 = P3 - dP3du;

      /* calculate bounding cylinders */
      const vfloatx rr1 = sqr_point_to_line_distance(Vec3vfx(dP0du),Vec3vfx(P3-P0));
      const vfloatx rr2 = sqr_point_to_line_distance(Vec3vfx(dP3du),Vec3vfx(P3-P0));
      const vfloatx maxr12 = sqrt(max(rr1,rr2));
      const vfloatx r_outer = max(P0.w,P1.w,P2.w,P3.w)+maxr12;
      const vfloatx r_inner = max(0.0f,min(P0.w,P1.w,P2.w,P3.w)-maxr12);
      const CylinderN<VSIZEX> cylinder_outer(Vec3vfx(P0),Vec3vfx(P3),r_outer);
      const CylinderN<VSIZEX> cylinder_inner(Vec3vfx(P0),Vec3vfx(P3),r_inner);
      vboolx valid = true; clear(valid,VSIZEX-1);

      /* intersect with outer cylinder */
      BBox<vfloatx> tc_outer; vfloatx u_outer0; Vec3vfx Ng_outer0; vfloatx u_outer1; Vec3vfx Ng_outer1;
      valid &= cylinder_outer.intersect(org,dir,tc_outer,u_outer0,Ng_outer0,u_outer1,Ng_outer1);
      if (none(valid)) return false;
     
      /* intersect with cap-planes */
      BBox<vfloatx> tp(ray.tnear-dt,ray.tfar-dt);
      tp = embree::intersect(tp,tc_outer);
      BBox<vfloatx> h0 = HalfPlaneN<VSIZEX>(Vec3vfx(P0),+Vec3vfx(dP0du)).intersect(org,dir);
      tp = embree::intersect(tp,h0);
      BBox<vfloatx> h1 = HalfPlaneN<VSIZEX>(Vec3vfx(P3),-Vec3vfx(dP3du)).intersect(org,dir);
      tp = embree::intersect(tp,h1);
      valid &= tp.lower <= tp.upper;
      if (none(valid)) return false;

      /* clamp and correct u parameter */
      u_outer0 = clamp(u_outer0,vfloatx(0.0f),vfloatx(1.0f));
      u_outer1 = clamp(u_outer1,vfloatx(0.0f),vfloatx(1.0f));
      u_outer0 = lerp(u0,u1,(vfloatx(step)+u_outer0)*(1.0f/float(VSIZEX)));
      u_outer1 = lerp(u0,u1,(vfloatx(step)+u_outer1)*(1.0f/float(VSIZEX)));

      /* intersect with inner cylinder */
      BBox<vfloatx> tc_inner;
      vfloatx u_inner0; Vec3vfx Ng_inner0; vfloatx u_inner1; Vec3vfx Ng_inner1;
      const vboolx valid_inner = cylinder_inner.intersect(org,dir,tc_inner,u_inner0,Ng_inner0,u_inner1,Ng_inner1);

      /* at the unstable area we subdivide deeper */
      const vboolx unstable0 = !valid_inner | abs(dot(Vec3vfx(normalize(ray.dir)),normalize(Ng_inner0))) < 0.3f;
      const vboolx unstable1 = !valid_inner | abs(dot(Vec3vfx(normalize(ray.dir)),normalize(Ng_inner1))) < 0.3f;
      
      /* subtract the inner interval from the current hit interval */
      BBox<vfloatx> tp0, tp1;
      subtract(tp,tc_inner,tp0,tp1);
      vboolx valid0 = valid & (tp0.lower <= tp0.upper);
      vboolx valid1 = valid & (tp1.lower <= tp1.upper);
      if (none(valid0 | valid1)) return false;

      /* iterate over all first hits front to back */
      bool found = false;
      while (any(valid0))
      {
        const size_t i = select_min(valid0,tp0.lower); clear(valid0,i);
        const size_t termDepth = unstable0[i] ? maxDepth+1 : maxDepth;
        if (depth >= termDepth) found |= intersect_bezier_iterative_jacobian(ray,dt,curve,u_outer0[i],tp0.lower[i],epilog);
        //if (depth >= termDepth) found |= intersect_bezier_iterative_debug   (ray,dt,curve,i,u_outer0,tp0,h0,h1,Ng_outer0,dP0du,dP3du,epilog);
        else                   found |= intersect_bezier_recursive_jacobian(ray,dt,curve,vu0[i+0],vu0[i+1],depth+1,epilog);
        valid0 &= tp0.lower+dt < ray.tfar;
      }
      valid1 &= tp1.lower+dt < ray.tfar;

      /* iterate over all second hits front to back */
      while (any(valid1))
      {
        const size_t i = select_min(valid1,tp1.lower); clear(valid1,i);
        const size_t termDepth = unstable1[i] ? maxDepth+1 : maxDepth;
        if (depth >= termDepth) found |= intersect_bezier_iterative_jacobian(ray,dt,curve,u_outer1[i],tp1.upper[i],epilog);
        //if (depth >= termDepth) found |= intersect_bezier_iterative_debug   (ray,dt,curve,i,u_outer1,tp1,h0,h1,Ng_outer1,dP0du,dP3du,epilog);
        else                   found |= intersect_bezier_recursive_jacobian(ray,dt,curve,vu0[i+0],vu0[i+1],depth+1,epilog);
        valid1 &= tp1.lower+dt < ray.tfar;
      }
      return found;
    }

#if 0
    __forceinline bool intersect_bezier_recursive_cone(const Ray& ray, const BezierCurve3fa& curve, const float u0, const float u1, const size_t depth, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      const int maxDepth = Device::debug_int0;

      /* subdivide curve */
      const float dscale = (u1-u0)*(1.0f/(3.0f*(VSIZEX-1)));
      const vfloatx vu0 = lerp(u0,u1,vfloatx(step)*(1.0f/(VSIZEX-1)));
      Vec4vfx P0, dP0du; curve.evalN(vu0,P0,dP0du); dP0du = dP0du * Vec4vfx(dscale);
      const Vec4vfx P3 = shift_right_1(P0);
      const Vec4vfx dP3du = Vec4vfx(dscale)*shift_right_1(dP0du); 
      const Vec4vfx P1 = P0 + dP0du; 
      const Vec4vfx P2 = P3 - dP3du;

      /* calculate bounding cylinders */
      const vfloatx rr1 = sqr_point_to_line_distance(Vec3vfx(dP0du),Vec3vfx(P3-P0));
      const vfloatx rr2 = sqr_point_to_line_distance(Vec3vfx(dP3du),Vec3vfx(P3-P0));
      const vfloatx maxr12 = sqrt(max(rr1,rr2));
      const vfloatx r_outer = max(P0.w,P1.w,P2.w,P3.w)+maxr12;
      const vfloatx r_inner = max(0.0f,min(P0.w,P1.w,P2.w,P3.w)-maxr12);
      const CylinderN<VSIZEX> cylinder_outer(Vec3vfx(P0),Vec3vfx(P3),r_outer);
      const CylinderN<VSIZEX> cylinder_inner(Vec3vfx(P0),Vec3vfx(P3),r_inner);
      vboolx valid = true; clear(valid,VSIZEX-1);

      /* put cones at the leaf level */
      if (depth == maxDepth) 
      {
        const ConeN<VSIZEX> cone(Vec3vfx(P0),P0.w,Vec3vfx(P3),P3.w);
        BBox<vfloatx> tc; vfloatx uc0; Vec3vfx Ng0; vfloatx uc1; Vec3vfx Ng1;
        valid &= cone.intersect(ray.org,ray.dir,tc,uc0,Ng0,uc1,Ng1);
        if (none(valid)) return false;

        /* intersect with cap-planes */
        BBox<vfloatx> tp(ray.tnear,min(t_o,ray.tfar));
        tp = embree::intersect(tp,tc);
        BBox<vfloatx> h0 = HalfPlaneN<VSIZEX>(Vec3vfx(P0),+Vec3vfx(dP0du)).intersect(ray.org,ray.dir);
        tp = embree::intersect(tp,h0);
        BBox<vfloatx> h1 = HalfPlaneN<VSIZEX>(Vec3vfx(P3),-Vec3vfx(dP3du)).intersect(ray.org,ray.dir);
        tp = embree::intersect(tp,h1);
        valid &= tp.lower <= tp.upper;
        if (none(valid)) return false;
        
        /* clamp and correct u parameter */
        uc0 = clamp(uc0,vfloatx(0.0f),vfloatx(1.0f));
        uc1 = clamp(uc1,vfloatx(0.0f),vfloatx(1.0f));
        uc0 = lerp(u0,u1,(vfloatx(step)+uc0)*(1.0f/float(VSIZEX)));
        uc1 = lerp(u0,u1,(vfloatx(step)+uc1)*(1.0f/float(VSIZEX)));

        const vboolx valid0 = valid & (tp.lower < t_o);
        if (any(valid0)) 
        {
          const size_t i = select_min(valid0,tp.lower); 
          u_o = uc0[i];
          t_o = tp.lower[i];
          Ng_o = Vec3fa(Ng0.x[i],Ng0.y[i],Ng0.z[i]);
          if (h0.lower[i] == t_o) Ng_o = -Vec3fa(dP0du.x[i],dP0du.y[i],dP0du.z[i]);
          if (h1.lower[i] == t_o) Ng_o = +Vec3fa(dP3du.x[i],dP3du.y[i],dP3du.z[i]);
        }

        const vboolx valid1 = valid & (tp.upper < t_o);
        if (any(valid1)) 
        {
          const size_t i = select_min(valid1,tp.upper); 
          u_o = uc1[i];
          t_o = tp.upper[i];
          Ng_o = Vec3fa(Ng1.x[i],Ng1.y[i],Ng1.z[i]);
          if (h0.lower[i] == t_o) Ng_o = -Vec3fa(dP0du.x[i],dP0du.y[i],dP0du.z[i]);
          if (h1.lower[i] == t_o) Ng_o = +Vec3fa(dP3du.x[i],dP3du.y[i],dP3du.z[i]);
        }
        return any(valid0 | valid1);
      }

      /* intersect with outer cylinder */
      BBox<vfloatx> tc_outer; vfloatx u_outer0; Vec3vfx Ng_outer0; vfloatx u_outer1; Vec3vfx Ng_outer1;
      valid &= cylinder_outer.intersect(ray.org,ray.dir,tc_outer,u_outer0,Ng_outer0,u_outer1,Ng_outer1);
      if (none(valid)) return false;

      /* intersect with cap-planes */
      BBox<vfloatx> tp(ray.tnear,ray.tfar);
      tp = embree::intersect(tp,tc_outer);
      BBox<vfloatx> h0 = HalfPlaneN<VSIZEX>(Vec3vfx(P0),+Vec3vfx(dP0du)).intersect(ray.org,ray.dir);
      tp = embree::intersect(tp,h0);
      BBox<vfloatx> h1 = HalfPlaneN<VSIZEX>(Vec3vfx(P3),-Vec3vfx(dP3du)).intersect(ray.org,ray.dir);
      tp = embree::intersect(tp,h1);
      valid &= tp.lower <= tp.upper;
      if (none(valid)) return false;

      /* clamp and correct u parameter */
      u_outer0 = clamp(u_outer0,vfloatx(0.0f),vfloatx(1.0f));
      u_outer1 = clamp(u_outer1,vfloatx(0.0f),vfloatx(1.0f));
      u_outer0 = lerp(u0,u1,(vfloatx(step)+u_outer0)*(1.0f/float(VSIZEX)));
      u_outer1 = lerp(u0,u1,(vfloatx(step)+u_outer1)*(1.0f/float(VSIZEX)));

      /* intersect with inner cylinder */
      BBox<vfloatx> tc_inner;
      cylinder_inner.intersect(ray.org,ray.dir,tc_inner);

      /* subtract the inner interval from the current hit interval */
      BBox<vfloatx> tp0, tp1;
      subtract(tp,tc_inner,tp0,tp1);
      vboolx valid0 = valid & (tp0.lower <= tp0.upper);
      vboolx valid1 = valid & (tp1.lower <= tp1.upper);
      if (none(valid0 | valid1)) return false;

      /* iterate over all first hits front to back */
      bool found = false;
      while (any(valid0))
      {
        const size_t i = select_min(valid0,tp0.lower); clear(valid0,i);
        found |= intersect_bezier_recursive_cone(ray,curve,vu0[i+0],vu0[i+1],depth+1,epilog);
        valid0 &= tp0.lower < ray.tfar;
      }

      /* iterate over all second hits front to back */
      while (any(valid1))
      {
        const size_t i = select_min(valid1,tp1.lower); clear(valid1,i);
        found |= intersect_bezier_recursive_cone(ray,curve,vu0[i+0],vu0[i+1],depth+1,epilog);
        valid1 &= tp1.lower < ray.tfar;
      }
      return found;
    }
#endif

    struct BezierGeometry1Intersector1
    {
      __forceinline BezierGeometry1Intersector1(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
      __noinline bool intersect(Ray& ray,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3,
                                   const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);

        /* move ray closer to make intersection stable */
        const float dt = dot(0.25f*(v0+v1+v2+v3)-ray.org,ray.dir)*rcp(dot(ray.dir,ray.dir));
        const Vec3fa ref(ray.org+dt*ray.dir,0.0f);
        const Vec3fa p0 = v0-ref;
        const Vec3fa p1 = v1-ref;
        const Vec3fa p2 = v2-ref;
        const Vec3fa p3 = v3-ref;

        const BezierCurve3fa curve(p0,p1,p2,p3,0.0f,1.0f,1);
        return intersect_bezier_recursive_jacobian(ray,dt,curve,0.0f,1.0f,1,epilog);
        //return intersect_bezier_recursive_cone(ray1,curve,0.0f,1.0f,1,u,t,Ng);
      }
    };
  }
}
