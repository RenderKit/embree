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

extern "C" int g_debug_int;

namespace embree
{
  namespace isa
  {
    __forceinline bool intersect_bezier_iterative2(const Ray& ray, const BezierCurve3fa& curve, float u0, float u1, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      //PRINT("intersecting");
      Vec3fa p0,n0,ddp0; curve.eval(u0,p0,n0,ddp0);
      Vec3fa p1,n1,ddp1; curve.eval(u1,p1,n1,ddp1);
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
        Vec3fa P,dPdu,ddPdu; curve.eval(u,P,dPdu,ddPdu);
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

    __forceinline bool intersect_bezier_iterative3(const Ray& ray, const BezierCurve3fa& curve, float u0, float u1, float t0, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      Vec3fa Q,P;
      //Vec3fa p0 = curve.v0;
      //Vec3fa p1 = curve.v3;

      Vec3fa p0,n0,ddp0; curve.eval(u0,p0,n0,ddp0);
      Vec3fa p1,n1,ddp1; curve.eval(u1,p1,n1,ddp1);
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

      /* iterative double step solver */
      float eps = 128.0f*float(ulp);
      float rcpLenP0P1 = rcp_length(p1-p0);
      float rcpLenDir = rcp_length(ray.dir);
      float t = t0;
      float u = u0 + (u1-u0)*dot(ray.org+t*ray.dir-p0,normalize(p1-p0))*rcpLenP0P1;
      for (size_t i=0; i<10; i++)
      {
        Q = ray.org + t*ray.dir;
        Vec3fa dPdu,ddPdu; curve.eval(u,P,dPdu,ddPdu);
        Vec3fa T = normalize(dPdu); // can be optimized away
        float du = dot(Q-P,T);
        float dt = sqrt(dot(Q-P,Q-P)-sqr(du))-P.w;
        u += du*rcpLenP0P1*(u1-u0)/(1.0f+P.w*maxC);
        t += dt*rcpLenDir;

        if (max(abs(du),abs(dt)) < eps) 
          break;
      }

      if (t < ray.tnear || t > ray.tfar) return false;
      if (std::isnan(u)) return false;
      if (std::isnan(t)) return false;
      if (Q == P) return false;

      u_o = u;
      t_o = t;
      Ng_o = Q-P;
      return true;
    }
    
    __forceinline bool intersect_bezier_iterative(const Ray& ray, const BezierCurve3fa& curve, float u, float& u_o, float& t_o, Vec3fa& Ng)
    {
      float t = 0.0f, d = 0.0f;
      for (size_t i=0; i<100; i++) 
      {
        //const float du = 0.0001f;
        Vec3fa P,dPdu,dPdu2; curve.eval(u,P,dPdu,dPdu2);
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

    template<int M>
      __forceinline vfloat<M> sqr_point_to_line_distance(const Vec3<vfloat<M>>& P, const Vec3<vfloat<M>>& Q0, const Vec3<vfloat<M>>& Q1) 
    {
      const Vec3<vfloat<M>> N = cross(P-Q0,P-Q1);
      const Vec3<vfloat<M>> D = Q1-Q0;
      return dot(N,N)*rcp(dot(D,D));
    }

    __forceinline bool intersect_bezier_recursive(const Ray& ray, const BezierCurve3fa& curve, float& u_o, float& t_o, Vec3fa& Ng_o)
    {
      //Cone::verify();
      //exit(1);

      //PRINT(curve);
      int maxDepth = g_debug_int;
      bool found = false;

      /* subdivide curve */
      const Vec4vfx dP0du = curve.derivative(vboolx(true),0,VSIZEX-1);
      const Vec4vfx dP3du = Vec4vfx(shift_right_1(dP0du.x),shift_right_1(dP0du.y),shift_right_1(dP0du.z),shift_right_1(dP0du.w));
      const Vec4vfx P0 = curve.eval0(vboolx(true),0,VSIZEX-1);
      const Vec4vfx P3 = Vec4vfx(shift_right_1(P0.x   ),shift_right_1(P0.y   ),shift_right_1(P0.z   ),shift_right_1(P0.w)   );
      const Vec4vfx P1 = P0 + Vec4vfx(1.0f/(3.0f*(VSIZEX-1)))*dP0du; 
      const Vec4vfx P2 = P3 - Vec4vfx(1.0f/(3.0f*(VSIZEX-1)))*dP3du;
      const vfloatx r1 = sqrt(sqr_point_to_line_distance(Vec3vfx(P1),Vec3vfx(P0),Vec3vfx(P3)));
      const vfloatx r2 = sqrt(sqr_point_to_line_distance(Vec3vfx(P2),Vec3vfx(P0),Vec3vfx(P3)));
      const vfloatx r = max(r1,r2)+max(P0.w,P1.w,P2.w,P3.w);
      const CylinderN<VSIZEX> cylinder(Vec3vfx(P0.x,P0.y,P0.z),Vec3vfx(P3.x,P3.y,P3.z),r);
      const ConeN<VSIZEX> cone(Vec3vfx(P0.x,P0.y,P0.z),P0.w,Vec3vfx(P3.x,P3.y,P3.z),P3.w);
      vboolx valid = true; clear(valid,VSIZEX-1);

      /* intersect with cylinder */
      BBox<vfloatx> tc; vfloatx u; Vec3vfx Ng;
      if (curve.depth == maxDepth) {
        valid &= cone    .intersect(ray.org,ray.dir,tc,u,Ng);
        valid &= tc.lower > ray.tnear;
      }
      else
        valid &= cylinder.intersect(ray.org,ray.dir,tc,u,Ng);
      if (none(valid)) return false;
      //PRINT(valid);
      u = clamp(u,vfloatx(0.0f),vfloatx(1.0f));
      //PRINT(valid);
      //PRINT(tc);

      /* intersect with cap-planes */
      BBox<vfloatx> tp(ray.tnear,ray.tfar);
      //PRINT(tp);
      tp = embree::intersect(tp,tc);
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
      if (none(valid)) return false;
      //PRINT(valid);

      /* iterate over all hits front to back */
      while (any(valid))
      {
        size_t i = select_min(valid,tp.lower);
        clear(valid,i);
        //PRINT2(curve.depth,i);

        if (curve.depth == maxDepth) 
        {
#if 0
          const float u0 = float(i+0)/float(VSIZEX);
          const float u1 = float(i+1)/float(VSIZEX);
          bool h = intersect_bezier_iterative3(ray,curve, u0, u1, tp.lower[i], u_o, t_o, Ng_o);
          u_o = (1.0f-u_o)*curve.t0 + u_o*curve.t1;
          if (u_o < 0.0f || u_o > 1.0f) return false;
          return h;
#else
          if (tp.lower[i] < t_o) {
            float uu = (float(i)+u[i])/float(VSIZEX);
            u_o = (1.0f-uu)*curve.t0 + uu*curve.t1;
            t_o = tp.lower[i];
            Ng_o = Vec3fa(Ng.x[i],Ng.y[i],Ng.z[i]);
            //u_o = float(i+1)/float(VSIZEX);
            //PRINT(u_o);
            //PRINT(t_o);
            //PRINT(Ng_o);
            if (h0.lower[i] == tp.lower[i]) Ng_o = -Vec3fa(dP0du.x[i],dP0du.y[i],dP0du.z[i]);
            if (h1.lower[i] == tp.lower[i]) Ng_o = +Vec3fa(dP3du.x[i],dP3du.y[i],dP3du.z[i]);
            return true;
            //found = true;
          } else {
            //PRINT("miss");
          }
          return false;
          //continue;
#endif
        }

        const Vec3fa p0(P0.x[i],P0.y[i],P0.z[i],P0.w[i]);
        const Vec3fa p1(P1.x[i],P1.y[i],P1.z[i],P1.w[i]);
        const Vec3fa p2(P2.x[i],P2.y[i],P2.z[i],P2.w[i]);
        const Vec3fa p3(P3.x[i],P3.y[i],P3.z[i],P3.w[i]);
        const float t0 = curve.t0+(curve.t1-curve.t0)*float(i+0)/float(VSIZEX);
        const float t1 = curve.t0+(curve.t1-curve.t0)*float(i+1)/float(VSIZEX);
        const BezierCurve3fa subcurve(p0,p1,p2,p3,t0,t1,curve.depth+1);
        if (intersect_bezier_recursive(ray,subcurve,u_o,t_o,Ng_o))
          return true;
          //found = true;
      }
      return false;
      //return found;
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
                                   const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const int Np,
                                   const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);

        float u = 0.0f;
        float t = ray.tfar;
        Vec3fa Ng = zero;

        BezierCurve3fa curve(v0,v1,v2,v3,0.0f,1.0f,1);
        if (!intersect_bezier_recursive(ray,curve,u,t,Ng))
          return false;

        LineIntersectorHitM<VSIZEX> hit;
        hit.vu[0] = u;
        hit.vv[0] = 0.0f;
        hit.vt[0] = t;
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
