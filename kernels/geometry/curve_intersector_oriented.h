// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
#include "curve_intersector_precalculations.h"
#include "curve_intersector_sweep.h"
#include "../subdiv/linear_bezier_patch.h"

#define DBG(x)

namespace embree
{
  namespace isa
  {
    struct CurveCounters
    {
      __forceinline CurveCounters()
        : numRecursions(0), numKrawczyk(0), numSolve(0), numSolveIterations(0) {}
      
      size_t numRecursions;
      size_t numKrawczyk;
      size_t numSolve;
      size_t numSolveIterations;
    };

    template<typename Epilog>
      struct TensorLinearCubicBezierSurfaceIntersector
      {
        const CurvePrecalculations1& pre;
        Ray& ray;
        TensorLinearCubicBezierSurface3fa curve3d;
        TensorLinearCubicBezierSurface2fa curve2d;
        float eps;
        const Epilog& epilog;
        bool isHit;
        CurveCounters counters;

        __forceinline TensorLinearCubicBezierSurfaceIntersector (const CurvePrecalculations1& pre, Ray& ray, const TensorLinearCubicBezierSurface3fa& curve3d, const Epilog& epilog)
          : pre(pre), ray(ray), curve3d(curve3d), epilog(epilog), isHit(false) {}

        __forceinline Interval1f solve_linear(const float u0, const float u1, const float& p0, const float& p1)
        {
          if (p1 == p0) {
            if (p0 == 0.0f) return Interval1f(u0,u1);
            else return Interval1f(empty);
          }
          const float t = -p0/(p1-p0);
          const float tt = lerp(u0,u1,t);
          return Interval1f(tt);
        }

        __forceinline void solve_linear(const float u0, const float u1, const Interval1f& p0, const Interval1f& p1, Interval1f& u)
        {
          if (sign(p0.lower) != sign(p0.upper)) u.extend(u0);
          if (sign(p0.lower) != sign(p1.lower)) u.extend(solve_linear(u0,u1,p0.lower,p1.lower));
          if (sign(p0.upper) != sign(p1.upper)) u.extend(solve_linear(u0,u1,p0.upper,p1.upper));
          if (sign(p1.lower) != sign(p1.upper)) u.extend(u1);
        }

        __forceinline Interval1f bezier_clipping(const CubicBezierCurve<Interval1f>& curve)
        {
          Interval1f u = empty;
          solve_linear(0.0f/3.0f,1.0f/3.0f,curve.p0,curve.p1,u);
          solve_linear(0.0f/3.0f,2.0f/3.0f,curve.p0,curve.p2,u);
          solve_linear(0.0f/3.0f,3.0f/3.0f,curve.p0,curve.p3,u);
          solve_linear(1.0f/3.0f,2.0f/3.0f,curve.p1,curve.p2,u);
          solve_linear(1.0f/3.0f,3.0f/3.0f,curve.p1,curve.p3,u);
          solve_linear(2.0f/3.0f,3.0f/3.0f,curve.p2,curve.p3,u);
          return intersect(u,Interval1f(0.0f,1.0f));
        }
        
        __forceinline Interval1f bezier_clipping(const LinearBezierCurve<Interval1f>& curve)
        {
          Interval1f v = empty;
          solve_linear(0.0f,1.0f,curve.p0,curve.p1,v);
          return intersect(v,Interval1f(0.0f,1.0f));
        }

        void solve_bezier_clipping(BBox1f cu, BBox1f cv, const TensorLinearCubicBezierSurface2fa& curve2)
        {
          BBox2fa bounds = curve2.bounds();
          if (bounds.upper.x < eps) return;
          if (bounds.upper.y < eps) return;
          if (bounds.lower.x > -eps) return;
          if (bounds.lower.y > -eps) return;
          
          if (max(cu.size(),cv.size()) < 1E-4f)
          {
            const float u = cu.center();
            const float v = cv.center();
            TensorLinearCubicBezierSurface1f curve_z = curve3d.xfm(pre.ray_space.row2(),ray.org);
            const float t = curve_z.eval(u,v);
            if (t >= ray.tnear() && t <= ray.tfar()) {
              const Vec3fa Ng = cross(curve3d.eval_du(u,v),curve3d.eval_dv(u,v));
              BezierCurveHit hit(t,u,v,Ng);
              isHit |= epilog(hit);
            }
            return;
          }

          
          const Vec2fa dv = curve2.axis_v();
          const TensorLinearCubicBezierSurface1f curve1v = curve2.xfm(dv);
          LinearBezierCurve<Interval1f> curve0v = curve1v.reduce_u();
          if (!curve0v.hasRoot()) return;
          
          const Interval1f v = bezier_clipping(curve0v);
          if (isEmpty(v)) return;
          TensorLinearCubicBezierSurface2fa curve2a = curve2.clip_v(v);
          cv = BBox1f(lerp(cv.lower,cv.upper,v.lower),lerp(cv.lower,cv.upper,v.upper));

          const Vec2fa du = curve2.axis_u();
          const TensorLinearCubicBezierSurface1f curve1u = curve2a.xfm(du);
          CubicBezierCurve<Interval1f> curve0u = curve1u.reduce_v();         
          int roots = curve0u.maxRoots();
          if (roots == 0) return;
          
          if (roots == 1)
          {
            const Interval1f u = bezier_clipping(curve0u);
            if (isEmpty(u)) return;
            TensorLinearCubicBezierSurface2fa curve2b = curve2a.clip_u(u);
            cu = BBox1f(lerp(cu.lower,cu.upper,u.lower),lerp(cu.lower,cu.upper,u.upper));
            solve_bezier_clipping(cu,cv,curve2b);
            return;
          }

          TensorLinearCubicBezierSurface2fa curve2l, curve2r;
          curve2a.split_u(curve2l,curve2r);
          solve_bezier_clipping(BBox1f(cu.lower,cu.center()),cv,curve2l);
          solve_bezier_clipping(BBox1f(cu.center(),cu.upper),cv,curve2r);
        }
        
        bool solve_bezier_clipping()
        {
          TensorLinearCubicBezierSurface3fa curve3dray = curve3d.xfm(pre.ray_space,ray.org);
          curve2d = TensorLinearCubicBezierSurface2fa(
            CubicBezierCurve2fa(Vec2fa(curve3dray.L.p0),Vec2fa(curve3dray.L.p1),Vec2fa(curve3dray.L.p2),Vec2fa(curve3dray.L.p3)),
            CubicBezierCurve2fa(Vec2fa(curve3dray.R.p0),Vec2fa(curve3dray.R.p1),Vec2fa(curve3dray.R.p2),Vec2fa(curve3dray.R.p3))
            );
          solve_bezier_clipping(BBox1f(0.0f,1.0f),BBox1f(0.0f,1.0f),curve2d);
          return isHit;
        }

        __forceinline void solve_newton_raphson(BBox1f cu, BBox1f cv)
        {
          Vec2fa uv(cu.center(),cv.center());
          const Vec2fa dfdu = curve2d.eval_du(uv.x,uv.y);
          const Vec2fa dfdv = curve2d.eval_dv(uv.x,uv.y);
          const LinearSpace2fa rcp_J = rcp(LinearSpace2fa(dfdu,dfdv));
          solve_newton_raphson_loop(cu,cv,uv,dfdu,dfdv,rcp_J);
        }

        __forceinline void solve_newton_raphson_loop(BBox1f cu, BBox1f cv, const Vec2fa& uv_in, const Vec2fa& dfdu, const Vec2fa& dfdv, const LinearSpace2fa& rcp_J)
        {
          Vec2fa uv = uv_in;
          counters.numSolve++;
          
          for (size_t i=0; i<200; i++)
          {
            counters.numSolveIterations++;
            const Vec2fa f = curve2d.eval(uv.x,uv.y);
            const Vec2fa duv = rcp_J*f;
            uv -= duv;

            if (max(fabs(f.x),fabs(f.y)) < eps)
            {
              const float u = uv.x;
              const float v = uv.y;
              if (!(u >= 0.0f && u <= 1.0f)) return; // rejects NaNs
              if (!(v >= 0.0f && v <= 1.0f)) return; // rejects NaNs
              const TensorLinearCubicBezierSurface1f curve_z = curve3d.xfm(pre.ray_space.row2(),ray.org);
              const float t = curve_z.eval(u,v);
              if (!(t > ray.tnear() && t < ray.tfar)) return; // rejects NaNs
              const Vec3fa Ng = cross(curve3d.eval_du(u,v),curve3d.eval_dv(u,v));
              BezierCurveHit hit(t,u,v,Ng);
              isHit |= epilog(hit);
              return;
            }
          }       
        }

        __forceinline bool clip_v(BBox1f& cu, BBox1f& cv)
        {
          const Vec2fa dv = curve2d.eval_dv(cu.lower,cv.lower);
          const TensorLinearCubicBezierSurface1f curve1v = curve2d.xfm(dv).clip(cu,cv);
          LinearBezierCurve<Interval1f> curve0v = curve1v.reduce_u();
          if (!curve0v.hasRoot()) return false;
          Interval1f v = bezier_clipping(curve0v);
          if (isEmpty(v)) return false;
          v = intersect(v + Interval1f(-0.1f,+0.1f),Interval1f(0.0f,1.0f));
          cv = BBox1f(lerp(cv.lower,cv.upper,v.lower),lerp(cv.lower,cv.upper,v.upper));
          return true;
        }

        __forceinline bool solve_krawczyk(BBox1f cu, BBox1f cv)
        {
          counters.numKrawczyk++;

          /* perform bezier clipping in v-direction to get tight v-bounds */
          TensorLinearCubicBezierSurface2fa curve2 = curve2d.clip(cu,cv);
          const Vec2fa dv = curve2.axis_v();
          const TensorLinearCubicBezierSurface1f curve1v = curve2.xfm(dv);
          LinearBezierCurve<Interval1f> curve0v = curve1v.reduce_u();
          if (!curve0v.hasRoot()) return true;
          Interval1f v = bezier_clipping(curve0v);
          if (isEmpty(v)) return true;
          v = intersect(v + Interval1f(-0.1f,+0.1f),Interval1f(0.0f,1.0f));
          curve2 = curve2.clip_v(v);
          cv = BBox1f(lerp(cv.lower,cv.upper,v.lower),lerp(cv.lower,cv.upper,v.upper));

          /* perform one newton raphson iteration */
          Vec2fa c(cu.center(),cv.center());
          Vec2fa f,dfdu,dfdv; curve2d.eval(c.x,c.y,f,dfdu,dfdv);
          const LinearSpace2fa rcp_J = rcp(LinearSpace2fa(dfdu,dfdv));
          const Vec2fa c1 = c - rcp_J*f;
          
          /* calculate bounds of derivatives */
          const BBox2fa bounds_du = (1.0f/cu.size())*curve2.derivative_u().bounds();
          const BBox2fa bounds_dv = (1.0f/cv.size())*curve2.derivative_v().bounds();

          /* calculate krawczyk test */
          LinearSpace2<Vec2<Interval1f>> I(Interval1f(1.0f), Interval1f(0.0f),
                                           Interval1f(0.0f), Interval1f(1.0f));

          LinearSpace2<Vec2<Interval1f>> G(Interval1f(bounds_du.lower.x,bounds_du.upper.x), Interval1f(bounds_dv.lower.x,bounds_dv.upper.x),
                                           Interval1f(bounds_du.lower.y,bounds_du.upper.y), Interval1f(bounds_dv.lower.y,bounds_dv.upper.y));

          const LinearSpace2<Vec2f> rcp_J2(rcp_J);
          const LinearSpace2<Vec2<Interval1f>> rcp_Ji(rcp_J2);
          
          const Vec2<Interval1f> x(cu,cv);
          const Vec2<Interval1f> K = Vec2<Interval1f>(Vec2f(c1)) + (I - rcp_Ji*G)*(x-Vec2<Interval1f>(Vec2f(c)));

          /* test if there is no solution */
          const Vec2<Interval1f> KK = intersect(K,x);
          if (isEmpty(KK.x) || isEmpty(KK.y)) return true;

          /* exit if convergence cannot get proven */
          if (!subset(K,x)) return false;

          /* solve using newton raphson iteration of convergence is guarenteed */
          solve_newton_raphson_loop(cu,cv,c1,dfdu,dfdv,rcp_J);
          return true;
        }
        
        void solve_newton_raphson_recursion(BBox1f cu, BBox1f cv)
        {
          counters.numRecursions++;

          /* for very short curve segments we assume convergence */
          if (cu.size() < 0.001f) {
            if (!clip_v(cu,cv)) return;
            return solve_newton_raphson(cu,cv);
          }

          /* we assume convergence for small u ranges and verify using krawczyk */
          if (cu.size() < 1.0f/6.0f) {
            if (solve_krawczyk(cu,cv)) {
              return;
            }
          }

          /* split the curve into VSIZEX-1 segments in u-direction */
          vboolx valid = true;
          TensorLinearCubicBezierSurface<Vec2vfx> subcurves = curve2d.clip_v(cv).vsplit_u(valid,cu);
          
          /* slabs test in v-direction */
          Vec2vfx ndu = cross(subcurves.axis_u());
          BBox<vfloatx> boundsu = subcurves.vxfm(ndu).bounds();
          valid &= boundsu.lower <= eps;
          valid &= boundsu.upper >= -eps;
          if (none(valid)) return;

          /* slabs test in u-direction */
          Vec2vfx ndv = cross(subcurves.axis_v());
          BBox<vfloatx> boundsv = subcurves.vxfm(ndv).bounds();
          valid &= boundsv.lower <= eps;
          valid &= boundsv.upper >= -eps;
          if (none(valid)) return;

          /* recurse into each hit curve segment */
          size_t mask = movemask(valid);
          while (mask)
          {
            const size_t i = __bscf(mask);
            const float u0 = float(i+0)*(1.0f/(VSIZEX-1));
            const float u1 = float(i+1)*(1.0f/(VSIZEX-1));
            const BBox1f cui(lerp(cu.lower,cu.upper,u0),lerp(cu.lower,cu.upper,u1));
            solve_newton_raphson_recursion(cui,cv);
          }
        }
        
        __forceinline bool solve_newton_raphson_main()
        {
          TensorLinearCubicBezierSurface3fa curve3dray = curve3d.xfm(pre.ray_space,ray.org);
          curve2d = TensorLinearCubicBezierSurface2fa(
            CubicBezierCurve2fa(Vec2fa(curve3dray.L.p0),Vec2fa(curve3dray.L.p1),Vec2fa(curve3dray.L.p2),Vec2fa(curve3dray.L.p3)),
            CubicBezierCurve2fa(Vec2fa(curve3dray.R.p0),Vec2fa(curve3dray.R.p1),Vec2fa(curve3dray.R.p2),Vec2fa(curve3dray.R.p3))
            );
            
          const BBox2fa b2 = curve2d.bounds();
          eps = 8.0f*float(ulp)*reduce_max(max(abs(b2.lower),abs(b2.upper)));

          BBox1f vu(0.0f,1.0f);
          BBox1f vv(0.0f,1.0f);
          solve_newton_raphson_recursion(vu,vv);
          /*if (isHit) {
            //((RayHit&)ray).u = counters.numRecursions/16.0f;
            //((RayHit&)ray).u = counters.numKrawczyk/4.0f;
            //((RayHit&)ray).u = counters.numSolve/4.0f;
            ((RayHit&)ray).u = counters.numSolveIterations/20.0f;
            ((RayHit&)ray).v = 0.0f;
            }*/
          return isHit;
        }
      };

    
    struct OrientedCurve1Intersector1
    {
      __forceinline OrientedCurve1Intersector1() {}
      
      __forceinline OrientedCurve1Intersector1(const Ray& ray, const void* ptr) {}
      
      template<typename Epilog>
      __noinline bool intersect(const CurvePrecalculations1& pre, Ray& ray, const CurveGeometry* geom, 
                                const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3,
                                const Vec3fa& n0, const Vec3fa& n1, const Vec3fa& n2, const Vec3fa& n3,
                                const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);

        // FIXME: what if n0 or n1 oriented along tangent?
        const Vec3fa k0 = normalize(cross(n0,v1-v0));
        const Vec3fa k3 = normalize(cross(n3,v3-v2));
        const Vec3fa d0 = v0.w*k0;
        const Vec3fa d1 = v1.w*k0;
        const Vec3fa d2 = v2.w*k3;
        const Vec3fa d3 = v3.w*k3;
        CubicBezierCurve3fa L(v0-d0,v1-d1,v2-d2,v3-d3);
        CubicBezierCurve3fa R(v0+d0,v1+d1,v2+d2,v3+d3);
        TensorLinearCubicBezierSurface3fa curve(L,R);
        //return TensorLinearCubicBezierSurfaceIntersector<Epilog>(pre,ray,curve,epilog).solve_bezier_clipping();
        return TensorLinearCubicBezierSurfaceIntersector<Epilog>(pre,ray,curve,epilog).solve_newton_raphson_main();
      }
    };
  }
}
