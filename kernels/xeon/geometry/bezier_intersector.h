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
#include "fillcone.h"
#include "line_intersector.h"

extern "C" int g_debug_int0;
extern "C" int g_debug_int1;

namespace embree
{
  namespace isa
  {
    __forceinline bool intersect_bezier_iterative2(const Ray& ray, const BezierCurve3fa& curve, float u0, float u1, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      //PRINT("intersecting");
      //Vec3fa p0,n0,ddp0; curve.eval(u0,p0,n0,ddp0);
      Vec3fa p0 = curve.eval(u0);
      Vec3fa n0 = curve.eval_du(u0);
      Vec3fa ddp0 = curve.eval_dudu(u0);

      //Vec3fa p1,n1,ddp1; curve.eval(u1,p1,n1,ddp1);
      Vec3fa p1 = curve.eval(u1);
      Vec3fa n1 = curve.eval_du(u1);
      Vec3fa ddp1 = curve.eval_dudu(u1);

      Vec3fa q0 = p0+n0*(1.0f/3.0f);
      Vec3fa q1 = p1-n1*(1.0f/3.0f);
      float rq0 = length(cross(p0-q0,p1-q0))/length(p1-p0)+q0.w;
      float rq1 = length(cross(p0-q1,p1-q1))/length(p1-p0)+q1.w;
      float r01 = max(p0.w,rq0,rq1,p1.w);

      /* intersect with bounding cone */
      BBox1f tc;
      const Cone cone(p0,r01,p1,r01);
      if (!cone.intersect(ray.org,ray.dir,tc,u_o,Ng_o)) {
        //PRINT("missed cone");
        return false;
      }
      //PRINT(tc);

      /* intersect with cap-planes */
      BBox1f tp(ray.tnear,ray.tfar);
      tp = embree::intersect(tp,tc);
      tp = embree::intersect(tp,intersect_half_plane(ray.org,ray.dir,+n0,p0));
      tp = embree::intersect(tp,intersect_half_plane(ray.org,ray.dir,-n1,p1));
      if (tp.lower > tp.upper) {
        //PRINT("missed culling");
        return false;
      }

      //PRINT(tp);
      //t_o = tc.lower;
      //return true;
      
      /* calculate bounds on curvature */
      float minN = min(length(n0),length(n1));
      float maxN = max(length(n0),length(n1));
      float maxR = max(p0.w,q0.w,q1.w,p1.w);
      //PRINT2(maxN,length(p1-p0));
      float maxC = length(max(abs(ddp0),abs(ddp1)))/minN; //length(ddPdu)/length(dPdu); // FIXME: is this the proper curvature?
      //PRINT(minN);
      //PRINT(maxN);
      //PRINT(maxC);
      //PRINT(maxR);
      //PRINT(1.0f/maxC-maxR);
      float minDT = max(0.001f,1.0f/maxC-maxR);

      /* iterative double step solver */
      float eps = 128.0f*float(ulp);
      float rcpLenP0P1 = rcp_length(p1-p0);
      float rcpLenDir = rcp_length(ray.dir);
      float t = tp.lower;
      float u = u0 + (u1-u0)*dot(ray.org+t*ray.dir-p0,normalize(p1-p0))*rcpLenP0P1;
      for (size_t i=0; i<10000; i++)
      {
        //PRINT(i);
        Vec3fa Q = ray.org + t*ray.dir;
        Vec3fa P = curve.eval(u);
        Vec3fa dPdu = curve.eval_du(u);
        Vec3fa ddPdu = curve.eval_dudu(u);
        //PRINT(P);
        //PRINT(dPdu);
        //PRINT(ddPdu);
        //PRINT(length(dnormalize(dPdu,ddPdu)));
        Vec3fa T = normalize(dPdu); // can be optimized away
        //float C = length(ddPdu)/length(dPdu); // FIXME: is this the proper curvature?
        float du = dot(Q-P,T);
        float dt = sqrt(dot(Q-P,Q-P)-sqr(du))-P.w;
        u += du*rcpLenP0P1*(u1-u0)/(1.0f+P.w*maxC);
        if (du < 10.0f*eps)
          t += min(minDT,dt*rcpLenDir);
        //PRINT(du);
        //PRINT(dt);
        //PRINT(u);
        //PRINT(t);
        if (t > tc.upper) {
          //PRINT("miss1");
          return false;
        }
        if (max(abs(du),abs(dt)) < eps) 
        {
          //PRINT("hit");
          //PRINT(eps*rcpLenDir);
          //PRINT(t+eps*rcpLenDir);
          //PRINT(t-eps*rcpLenDir);
          if (t+eps*rcpLenDir < tp.lower || t-eps*rcpLenDir > tp.upper) {
            //PRINT("miss2");
            return false;
          }
          //if (t < ray.tnear || t > t_o) {
          ////PRINT("miss2");
          //  return false;
          //}
          u_o = u;
          t_o = t;
          Ng_o = Q-P;
          return true;
        }
      }
      //PRINT("miss3");
      return false;
    }

    __forceinline bool intersect_bezier_iterative3(const Ray& ray, const BezierCurve3fa& curve, float u0, float u1, float t0, float t1, float t2, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      //PRINT("intersecting");
      //Vec3fa p0,n0,ddp0; curve.eval(u0,p0,n0,ddp0);
      Vec3fa p0 = curve.eval(u0);
      Vec3fa n0 = curve.eval_du(u0);
      Vec3fa ddp0 = curve.eval_dudu(u0);

      //Vec3fa p1,n1,ddp1; curve.eval(u1,p1,n1,ddp1);
      Vec3fa p1 = curve.eval(u1);
      Vec3fa n1 = curve.eval_du(u1);
      Vec3fa ddp1 = curve.eval_dudu(u1);

      Vec3fa q0 = p0+n0*(1.0f/3.0f);
      Vec3fa q1 = p1-n1*(1.0f/3.0f);
      float rq0 = length(cross(p0-q0,p1-q0))/length(p1-p0)+q0.w;
      float rq1 = length(cross(p0-q1,p1-q1))/length(p1-p0)+q1.w;
      float r01 = max(p0.w,rq0,rq1,p1.w);

      /* calculate bounds on curvature */
      float minN = min(length(n0),length(n1));
      float maxN = max(length(n0),length(n1));
      float maxR = max(p0.w,q0.w,q1.w,p1.w);
      //PRINT2(maxN,length(p1-p0));
      float maxC = length(max(abs(ddp0),abs(ddp1)))/minN; //length(ddPdu)/length(dPdu); // FIXME: is this the proper curvature?
      //PRINT(minN);
      //PRINT(maxN);
      //PRINT(maxC);
      //PRINT(maxR);
      //PRINT(1.0f/maxC-maxR);
      float minDT = max(0.001f,1.0f/maxC-maxR);

      /* iterative double step solver */
      float eps = 128.0f*float(ulp);
      float rcpLenP0P1 = rcp_length(p1-p0);
      float rcpLenDir = rcp_length(ray.dir);
      float t = t0;
      float u = u0 + (u1-u0)*dot(ray.org+t*ray.dir-p0,normalize(p1-p0))*rcpLenP0P1;
      for (size_t i=0; i<10000; i++)
      {
        //PRINT(i);
        Vec3fa Q = ray.org + t*ray.dir;
        //Vec3fa P,dPdu,ddPdu; curve.eval(u,P,dPdu,ddPdu);
        Vec3fa P = curve.eval(u);
        Vec3fa dPdu = curve.eval_du(u);
        Vec3fa ddPdu = curve.eval_dudu(u);

        //PRINT(P);
        //PRINT(dPdu);
        //PRINT(ddPdu);
        //PRINT(length(dnormalize(dPdu,ddPdu)));
        Vec3fa T = normalize(dPdu); // can be optimized away
        //float C = length(ddPdu)/length(dPdu); // FIXME: is this the proper curvature?
        float du = dot(Q-P,T);
        float dt = sqrt(dot(Q-P,Q-P)-sqr(du))-P.w;
        u += du*rcpLenP0P1*(u1-u0)/(1.0f+P.w*maxC);
        //if (du < 10.0f*eps)
        t += min(minDT,dt*rcpLenDir);
        //PRINT(du);
        //PRINT(dt);
        //PRINT(u);
        //PRINT(t);
        if (t > t2) {
          //PRINT("miss1");
          return false;
        }
        if (max(abs(du),abs(dt)) < eps) 
        {
          //PRINT("hit");
          //PRINT(eps*rcpLenDir);
          //PRINT(t+eps*rcpLenDir);
          //PRINT(t-eps*rcpLenDir);
          //if (t+eps*rcpLenDir < t0 || t-eps*rcpLenDir > t1) {
          //  PRINT("miss2");
          //  return false;
          //}
          if (t < ray.tnear || t > t_o) {
            //PRINT("miss2");
            return false;
          }
          u_o = u;
          t_o = t;
          Ng_o = Q-P;
          return true;
        }
      }
      //PRINT("miss3");
      return false;
    }

    
    __forceinline bool intersect_bezier_iterative(const Ray& ray, const BezierCurve3fa& curve, float u, float& u_o, float& t_o, Vec3fa& Ng)
    {
      float t = 0.0f, d = 0.0f;
      for (size_t i=0; i<100; i++) 
      {
        //const float du = 0.0001f;
        //Vec3fa P,dPdu,dPdu2; curve.eval(u,P,dPdu,dPdu2);
        Vec3fa P = curve.eval(u);
        Vec3fa dPdu = curve.eval_du(u);
        Vec3fa dPdu2 = curve.eval_dudu(u);

        //Vec3fa _P,_dPdu,_dPdu2; curve.eval(u+du,_P,_dPdu,_dPdu2);
        //PRINT2(dPdu,(_P-P)/du);
        //PRINT2(dPdu2,(_dPdu-dPdu)/du);
        float A = dot(P-ray.org,dPdu);
        //float _A = dot(_P-ray.org,_dPdu);
        float dAdu = dot(dPdu,dPdu) + dot(P-ray.org,dPdu2);
        //PRINT2(dAdu,(_A-A)/du);
        float B = dot(ray.dir,dPdu);
        //float _B = dot(ray.dir,_dPdu);
        float dBdu = dot(ray.dir,dPdu2);
        //PRINT2(dBdu,(_B-B)/du);
        t = A/B;
        //float _t = _A/_B;
        float dtdu = dAdu/B - A*dBdu/sqr(B);
        //PRINT2(dtdu,(_t-t)/du);
        Ng = ray.org+t*ray.dir-P;
        //Vec3fa _Ng = ray.org+_t*ray.dir-P;
        Vec3fa dNgdu = dtdu*ray.dir;
        //PRINT2(dNgdu,(_Ng-Ng)/du);
        d = length(Ng)-P.w;
        //float _d = length(_Ng)-_P.w;
        float ddu = dot(Ng,dNgdu)/length(Ng)-dPdu.w;
        //PRINT2(ddu,(_d-d)/du);
        //u += 0.1f*abs(d);
        u -= d/ddu;
        if (abs(d) < 0.001f) {
          u_o = u;
          t_o = t;
          return true;
        }
      }
      return false;
    }

    __forceinline bool intersect_bezier_iterative_jacobian(const Ray& ray, const BezierCurve3fa& curve, float u, float t, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      //PRINT("jacobian solver");
      //PRINT2(u,t);
      for (size_t i=0; i<5; i++) 
      {
        //PRINT(i);
        Vec3fa Q = ray.org + t*ray.dir;
        Vec3fa dQdu = zero;
        Vec3fa dQdt = ray.dir;

        //Vec3fa P,dPdu,ddPdu; curve.eval(u,P,dPdu,ddPdu);
        Vec3fa P = curve.eval(u);
        Vec3fa dPdu = curve.eval_du(u);
        Vec3fa ddPdu = curve.eval_dudu(u);
        Vec3fa dPdt = zero;

        Vec3fa R = Q-P;
        Vec3fa dRdu = dQdu-dPdu;
        Vec3fa dRdt = dQdt-dPdt;

        Vec3fa T = normalize(dPdu);
        Vec3fa dTdu = dnormalize(dPdu,ddPdu);
        Vec3fa dTdt = zero;

        float f = dot(R,T);
        float dfdu = dot(dRdu,T) + dot(R,dTdu);
        float dfdt = dot(dRdt,T) + dot(R,dTdt);

        float K = dot(R,R)-sqr(f);
        float dKdu = 2.0f*dot(R,dRdu)-2.0f*f*dfdu;
        float dKdt = 2.0f*dot(R,dRdt)-2.0f*f*dfdt;

        float g = sqrt(K)-P.w;
        float dgdu = 0.5f*dKdu*rsqrt(K)-dPdu.w;
        float dgdt = 0.5f*dKdt*rsqrt(K)-dPdt.w;

        LinearSpace2f rcp_jacobian = rcp(LinearSpace2f(dfdu,dfdt,dgdu,dgdt));
        Vec2f dut = rcp_jacobian*Vec2f(f,g);
        Vec2f ut = Vec2f(u,t) - dut;
        u = ut.x; t = ut.y;
        //PRINT4(u,t,f,g);

        if (abs(f) < 16.0f*float(ulp)*length(dPdu) && abs(g) < 16.0f*float(ulp)*length(ray.dir)) 
        {
          //PRINT("converged");
          //PRINT2(f,g);
          //if (std::isnan(t) || std::isinf(t)) return false;
          //if (std::isnan(u) || std::isinf(t)) return false;
          if (t > t_o) return false;
          if (t < ray.tnear || t > ray.tfar) return false;
          if (u < 0.0f || u_o > 1.0f) return false;
          //PRINT("hit");
          u_o = u;
          t_o = t;
          Vec3fa R = normalize(Q-P);
          Vec3fa U = dPdu+dPdu.w*R;
          Vec3fa V = cross(dPdu,R);
          Ng_o = cross(V,U);
          //PRINT(u_o);
          //PRINT(t_o);
          //PRINT(Ng_o);
          return true;
        }
      }
      return false;
    }

    template<int M>
      __forceinline vfloat<M> sqr_point_to_line_distance(const Vec3<vfloat<M>>& P, const Vec3<vfloat<M>>& Q0, const Vec3<vfloat<M>>& Q1) 
    {
      const Vec3<vfloat<M>> N = cross(P-Q0,P-Q1);
      const Vec3<vfloat<M>> D = Q1-Q0;
      return dot(N,N)*rcp(dot(D,D));
    }

    __forceinline void subtract(const BBox<vfloatx>& a, const BBox<vfloatx>& b, BBox<vfloatx>& c, BBox<vfloatx>& d)
    {
      //c = intersect(a,BBox<vfloatx>(vfloatx(neg_inf),b.lower));
      //d = intersect(a,BBox<vfloatx>(b.upper,vfloatx(pos_inf)));
      c.lower = a.lower;
      c.upper = min(a.upper,b.lower);
      d.lower = max(a.lower,b.upper);
      d.upper = a.upper;
    }

    __forceinline size_t select_min(const vboolx& valid0, const vfloatx& v0, const vboolx& valid1, const vfloatx& v1) 
    { 
      const vfloat4 a0 = select(valid0,v0,vfloat4(pos_inf)); 
      const vfloat4 a1 = select(valid1,v1,vfloat4(pos_inf)); 
      const vfloatx m = vreduce_min(min(a0,a1));
      const vboolx valid_min0 = valid0 & (a0 == m);
      const vboolx valid_min1 = valid1 & (a1 == m);
      size_t m0 = movemask(any(valid_min0) ? valid_min0 : valid0);
      size_t m1 = movemask(any(valid_min1) ? valid_min1 : valid1);
      return __bsf(m0 | (m1 << VSIZEX)); 
    }

    __forceinline bool intersect_bezier_recursive(const Ray& ray, const BezierCurve3fa& curve, const float u0, const float u1, const size_t depth, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      //PRINT(depth);
      //Cone::verify();
      //exit(1);

      //PRINT(curve);
      int maxDepth = g_debug_int0;
      bool found = false;

      /* subdivide curve */
      const vfloatx lu = vfloatx(step)*(1.0f/(VSIZEX-1));
      const vfloatx vu0 = (vfloatx(one)-lu)*u0 + lu*u1;
      Vec4vfx P0, dP0du; curve.evalN(vu0,P0,dP0du);
      const Vec4vfx  P3   = Vec4vfx(shift_right_1(P0.x   ),shift_right_1(P0.y   ),shift_right_1(P0.z   ),shift_right_1(P0.w)   );
      const Vec4vfx dP3du = Vec4vfx(shift_right_1(dP0du.x),shift_right_1(dP0du.y),shift_right_1(dP0du.z),shift_right_1(dP0du.w));
      const Vec4vfx P1 = P0 + Vec4vfx((u1-u0)/(3.0f*(VSIZEX-1)))*dP0du; 
      const Vec4vfx P2 = P3 - Vec4vfx((u1-u0)/(3.0f*(VSIZEX-1)))*dP3du;
      const vfloatx r1 = sqrt(sqr_point_to_line_distance(Vec3vfx(P1),Vec3vfx(P0),Vec3vfx(P3)));
      const vfloatx r2 = sqrt(sqr_point_to_line_distance(Vec3vfx(P2),Vec3vfx(P0),Vec3vfx(P3)));
      const vfloatx r_outer = max(P0.w,P1.w,P2.w,P3.w)+max(r1,r2);
      const vfloatx r_inner = max(0.0f,min(P0.w,P1.w,P2.w,P3.w)-max(r1,r2));
      const CylinderN<VSIZEX> cylinder_outer(Vec3vfx(P0.x,P0.y,P0.z),Vec3vfx(P3.x,P3.y,P3.z),r_outer);
      const CylinderN<VSIZEX> cylinder_inner(Vec3vfx(P0.x,P0.y,P0.z),Vec3vfx(P3.x,P3.y,P3.z),r_inner);
      const ConeN<VSIZEX> cone(Vec3vfx(P0.x,P0.y,P0.z),P0.w,Vec3vfx(P3.x,P3.y,P3.z),P3.w);
      vboolx valid = true; clear(valid,VSIZEX-1);
      //if (depth == 1) { valid = false; set(valid,1); }
      //if (depth == 2) { valid = false; set(valid,0); }

      /* intersect with cylinder */
      BBox<vfloatx> tc_outer; vfloatx u_outer; Vec3vfx Ng_outer;
      BBox<vfloatx> tc_inner; vfloatx u_inner; Vec3vfx Ng_inner;
#if 0
      if (depth == maxDepth) {
        valid &= cone    .intersect(ray.org,ray.dir,tc,u,Ng);
        valid &= tc.lower > ray.tnear;
      }
      else
#endif
      {
        valid &= cylinder_outer.intersect(ray.org,ray.dir,tc_outer,u_outer,Ng_outer);
        vboolx valid_inner = cylinder_inner.intersect(ray.org,ray.dir,tc_inner,u_inner,Ng_inner);
        tc_inner.lower = select(valid_inner,tc_inner.lower,float(pos_inf));
        tc_inner.upper = select(valid_inner,tc_inner.upper,float(neg_inf));
      }
      //PRINT(valid);
      if (none(valid)) return false;
      //PRINT(valid);
      u_outer = clamp(u_outer,vfloatx(0.0f),vfloatx(1.0f));
      u_inner = clamp(u_inner,vfloatx(0.0f),vfloatx(1.0f));
      u_outer = (vfloatx(step)+u_outer)*(1.0f/float(VSIZEX));
      u_inner = (vfloatx(step)+u_inner)*(1.0f/float(VSIZEX));
      //PRINT(valid);
      //PRINT(tc);

      /* intersect with cap-planes */
      BBox<vfloatx> tp(ray.tnear,ray.tfar);
      //PRINT(tp);
      tp = embree::intersect(tp,tc_outer);
      //PRINT(tp);
      auto h0 = intersect_half_planeN(ray.org,ray.dir,+Vec3vfx(dP0du),Vec3vfx(P0));
      //PRINT(h0);
      tp = embree::intersect(tp,h0);
      //PRINT(tp);
      auto h1 = intersect_half_planeN(ray.org,ray.dir,-Vec3vfx(dP3du),Vec3vfx(P3));
      //PRINT(h1);
      tp = embree::intersect(tp,h1);
      //PRINT(tp);

      valid &= tp.lower <= tp.upper;
      valid &= tp.lower < t_o;
      if (none(valid)) return false;
      //PRINT(valid);

      BBox<vfloatx> tp0, tp1;
      subtract(tp,tc_inner,tp0,tp1);
      vboolx valid0 = valid & (tp0.lower <= tp0.upper);
      vboolx valid1 = valid & (tp1.lower <= tp1.upper);

      float tp_lower[2*VSIZEX]; vfloatx::storeu(&tp_lower[0*VSIZEX],tp0.lower ); vfloatx::storeu(&tp_lower[1*VSIZEX],tp1.lower );
      float tp_upper[2*VSIZEX]; vfloatx::storeu(&tp_upper[0*VSIZEX],tp0.upper ); vfloatx::storeu(&tp_upper[1*VSIZEX],tp1.upper );
      float u       [2*VSIZEX]; vfloatx::storeu(&u       [0*VSIZEX],u_outer   ); vfloatx::storeu(&u       [1*VSIZEX],u_inner   );
      float Ng_x    [2*VSIZEX]; vfloatx::storeu(&Ng_x    [0*VSIZEX],Ng_outer.x); vfloatx::storeu(&Ng_x    [1*VSIZEX],Ng_inner.x);
      float Ng_y    [2*VSIZEX]; vfloatx::storeu(&Ng_y    [0*VSIZEX],Ng_outer.y); vfloatx::storeu(&Ng_y    [1*VSIZEX],Ng_inner.y);
      float Ng_z    [2*VSIZEX]; vfloatx::storeu(&Ng_z    [0*VSIZEX],Ng_outer.z); vfloatx::storeu(&Ng_z    [1*VSIZEX],Ng_inner.z);

      /* iterate over all hits front to back */
      while (any(valid0 | valid1))
      {
        const size_t i = select_min(valid0,tp0.lower,valid1,tp1.lower);
        const size_t j = i & (VSIZEX-1);
        if (i < VSIZEX) clear(valid0,j);
        else            clear(valid1,j);
        
        //PRINT2(curve.depth,i);

        if (depth == maxDepth) 
        {
          if (g_debug_int1 % 2)
          {
            float uu = u[i];
            float ru = (1.0f-uu)*u0 + uu*u1;
            //bool h = intersect_bezier_iterative(ray,curve, ru, u_o, t_o, Ng_o);
            //bool h = intersect_bezier_iterative3(ray,curve, vu0[i], vu0[i+1], tp.lower[i], tp.upper[i], tc.upper[i], u_o, t_o, Ng_o);

            if (!intersect_bezier_iterative_jacobian(ray,curve,ru,tp_lower[i],u_o,t_o,Ng_o))
              continue;
            
            //if (u_o < 0.0f || u_o > 1.0f)
            //continue;

            found = true;
            valid0 &= tp0.lower < t_o;
            valid1 &= tp1.lower < t_o;
            continue;
          }
          else
          {
            if (tp_lower[i] < t_o) {
              float uu = u[i]; //(float(i)+u[i])/float(VSIZEX);
              u_o = (1.0f-uu)*u0 + uu*u1;
              t_o = tp_lower[i];
              Ng_o = Vec3fa(Ng_x[i],Ng_y[i],Ng_z[i]);
              //u_o = float(i+1)/float(VSIZEX);
              if (h0.lower[j] == tp_lower[i]) Ng_o = -Vec3fa(dP0du.x[j],dP0du.y[j],dP0du.z[j]);
              if (h1.lower[j] == tp_lower[i]) Ng_o = +Vec3fa(dP3du.x[j],dP3du.y[j],dP3du.z[j]);
              //return true;
              found = true;
              valid0 &= tp0.lower < t_o;
              valid1 &= tp1.lower < t_o;
            } else {
              //PRINT("miss");
            }
            //return false;
            continue;
          }
        }

        if (intersect_bezier_recursive(ray,curve,vu0[j+0],vu0[j+1],depth+1,u_o,t_o,Ng_o)) {
          //return true;
          found = true; 
          valid0 &= tp0.lower < t_o;
          valid1 &= tp1.lower < t_o;
        }
      }
      //return false;
      return found;
    }

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
    
#if 1

    struct Bezier1Intersector1
    {
      __forceinline Bezier1Intersector1(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
      __forceinline bool intersect(Ray& ray,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int Np,
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

        float u = 0.0f;
        float t = ray.tfar-dt;
        Vec3fa Ng = zero;

        const Ray ray1(zero,ray.dir,ray.tnear-dt,ray.tfar-dt);
        const BezierCurve3fa curve(p0,p1,p2,p3,0.0f,1.0f,1);
        if (!intersect_bezier_recursive(ray1,curve,0.0f,1.0f,1,u,t,Ng))
          return false;

        LineIntersectorHitM<VSIZEX> hit;
        hit.vu[0] = u;
        hit.vv[0] = 0.0f;
        hit.vt[0] = t+dt;
        hit.vNg.x[0] = Ng.x;
        hit.vNg.y[0] = Ng.y;
        hit.vNg.z[0] = Ng.z;
        vboolx valid_o = false;
        set(valid_o,0);
        return epilog(valid_o,hit);
      }
    };

#endif

#if 0

    struct Bezier1Intersector1
    {
      __forceinline Bezier1Intersector1(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
      __forceinline bool intersect(Ray& ray,
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int Np,
                                   const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);
        bool ishit = false;
        BezierCurve3fa curve2D(v0,v1,v2,v3,0.0f,1.0f,4);
        int N = Np-1; // calculate number of segments
        float rcpN = 1.0f/float(N);

        float u = 0.0f;
        float t = ray.tfar;
        Vec3fa Ng = zero;

        /* process SIMD-size-1 many segments per iteration */
        for (int i=0; i<N; i+=VSIZEX-1)
        {
          /* evaluate the bezier curve */
          vboolx valid = vintx(i)+vintx(step) < vintx(N);
          const Vec4vfx p  = curve2D.eval0(valid,i,N);
          const Vec4vfx dp = curve2D.derivative(valid,i,N);

          /* early exit */
          /*const Vec3vfx Q1(p.x,p.y,p.z);
          const Vec3vfx Q2(shift_right_1(p.x),shift_right_1(p.y),shift_right_1(p.z));
          valid &= abs(dot(Vec3vfx(ray.org)-Q1,normalize_safe(cross(Q2-Q1,Vec3vfx(ray.dir))))) <= max(p.w,shift_right_1(p.w));
          if (none(valid)) continue;*/
         
          /* intersect each bezier segment */
          vboolx valid_o = false;
          LineIntersectorHitM<VSIZEX> hit;

          for (size_t j=0; j<min(VSIZEX-1,N-i); j++)
          {
            //PRINT(i+j);
            //if (i+j != 0 && i+j != 1) continue;
            //std::cout << std::endl;
            //PRINT(j);
            const Vec3fa p1( p.x[j+0], p.y[j+0] ,p.z[j+0], p.w[j+0]);
            const Vec3fa p2( p.x[j+1], p.y[j+1] ,p.z[j+1], p.w[j+1]);
            const Vec3fa n1(dp.x[j+0],dp.y[j+0],dp.z[j+0],dp.w[j+0]);
            const Vec3fa n2(dp.x[j+1],dp.y[j+1],dp.z[j+1],dp.w[j+1]);
            const float  r1 =  p.w[j+0];
            const float  r2 =  p.w[j+1];
            const float t0 = float(i+j+0)*rcpN;
            const float t1 = float(i+j+1)*rcpN;
            //const FillCone cone(p1,n1,r1,p2,n2,r2);
            //if (!cone.intersect(ray,u,t,Ng)) continue;
            //if (!intersect_bezier_iterative(ray, curve2D,t0,u,t,Ng)) continue;
            if (!intersect_bezier_iterative2(ray,curve2D,t0,t1,u,t,Ng)) continue;
            //const BezierCurve3fa subcurve(p1,p1+(1.0f/3.0f)*n1,p2-(1.0f/3.0f)*n2,p2,t0,t1,0);
            //if (!intersect_bezier_recursive(ray,subcurve,u,t,Ng)) continue;
            hit.vu[j] = u; //(float(i+j)+u)*rcpN;
            hit.vv[j] = 0.0f;
            hit.vt[j] = t;
            hit.vNg.x[j] = Ng.x;
            hit.vNg.y[j] = Ng.y;
            hit.vNg.z[j] = Ng.z;
            set(valid_o,j);
          }
          
          /* update hit information */
          if (unlikely(none(valid_o))) continue;
          ishit |= epilog(valid_o,hit);
        }
        return ishit;
      }
    };

#endif

#if 0

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
