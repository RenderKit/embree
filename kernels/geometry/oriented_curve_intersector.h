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
#include "bezier_curve_intersector.h"

#define DBG(x) 

namespace embree
{
  namespace isa
  {
    template<typename V>
      struct Interval
      {
        V lower, upper;

        __forceinline Interval() {}
        __forceinline Interval           ( const Interval& other ) { lower = other.lower; upper = other.upper; }
        __forceinline Interval& operator=( const Interval& other ) { lower = other.lower; upper = other.upper; return *this; }

        __forceinline Interval(const V& a) : lower(a), upper(a) {}
        __forceinline Interval(const V& lower, const V& upper) : lower(lower), upper(upper) {}

        __forceinline friend Interval operator +( const Interval& a, const Interval& b )
        {
          return Interval(min(a.lower+b.lower,a.lower+b.upper,a.upper+b.lower,a.upper+b.upper),
                          max(a.lower+b.lower,a.lower+b.upper,a.upper+b.lower,a.upper+b.upper));
        }

        __forceinline friend Interval operator -( const Interval& a, const Interval& b )
        {
          return Interval(min(a.lower-b.lower,a.lower-b.upper,a.upper-b.lower,a.upper-b.upper),
                          max(a.lower-b.lower,a.lower-b.upper,a.upper-b.lower,a.upper-b.upper));
        }

        __forceinline friend Interval operator *( const Interval& a, const Interval& b )
        {
          return Interval(min(a.lower*b.lower,a.lower*b.upper,a.upper*b.lower,a.upper*b.upper),
                          max(a.lower*b.lower,a.lower*b.upper,a.upper*b.lower,a.upper*b.upper));
        }

        friend std::ostream& operator<<(std::ostream& cout, const Interval& a) {
          return cout << "[" << a.lower << ", " << a.upper << "]";
        }
      };
    
    template<typename V>
      struct LinearCurve
      {
        BBox<V> v;
        V p0,p1;
        
        __forceinline LinearCurve () {}
        
        __forceinline LinearCurve (V p0, V p1)
        : v(0.0f,1.0f), p0(p0), p1(p1) {}
        
        __forceinline LinearCurve (BBox1f v, V p0, V p1)
        : v(v), p0(p0), p1(p1) {}
        
        bool hasRoot() const;
        
        friend std::ostream& operator<<(std::ostream& cout, const LinearCurve& a) {
          return cout << "LinearCurve ( (" << a.v.lower << ", " << a.v.upper << "), " << a.p0 << ", " << a.p1 << ")";
        }
      };
    
    __forceinline int numRoots(const BBox1f& p0, const BBox1f& p1)
    {
      float eps = 1E-4f;
      bool neg0 = p0.lower < eps; bool pos0 = p0.upper > -eps;
      bool neg1 = p1.lower < eps; bool pos1 = p1.upper > -eps;
      return (neg0 && pos1) || (pos0 && neg1) || (neg0 && pos0) || (neg1 && pos1);
    }
    
    template<> __forceinline bool LinearCurve<BBox1f>::hasRoot() const {
      return numRoots(p0,p1);
    }
    
    template<typename V>
      struct MyBezierCurve
      {
        BBox1f u;
        V p0,p1,p2,p3;
        
        __forceinline MyBezierCurve () {}
        
        __forceinline MyBezierCurve (V p0, V p1, V p2, V p3)
        : u(0.0f,1.0f), p0(p0), p1(p1), p2(p2), p3(p3) {}
        
        __forceinline MyBezierCurve (BBox1f u, V p0, V p1, V p2, V p3)
        : u(u), p0(p0), p1(p1), p2(p2), p3(p3) {}
        
        __forceinline MyBezierCurve<float> xfm(const V& dx) const {
          return MyBezierCurve<float>(u,dot(p0,dx),dot(p1,dx),dot(p2,dx),dot(p3,dx));
        }

        __forceinline MyBezierCurve<float> xfm(const V& dx, const V& p) const {
          return MyBezierCurve<float>(u,dot(p0-p,dx),dot(p1-p,dx),dot(p2-p,dx),dot(p3-p,dx));
        }
        
        __forceinline MyBezierCurve<Vec2f> xfm(const V& dx, const V& dy, const V& p) const
        {
          const Vec2f q0(dot(p0-p,dx), dot(p0-p,dy));
          const Vec2f q1(dot(p1-p,dx), dot(p1-p,dy));
          const Vec2f q2(dot(p2-p,dx), dot(p2-p,dy));
          const Vec2f q3(dot(p3-p,dx), dot(p3-p,dy));
          return MyBezierCurve<Vec2f>(u,q0,q1,q2,q3);
        }
        
        __forceinline int maxRoots() const;

        __forceinline BBox<V> bounds() const {
          return merge(BBox<V>(p0),BBox<V>(p1),BBox<V>(p2),BBox<V>(p3));
        }

        __forceinline BBox1f bounds(const V& axis) const {
          return merge(BBox1f(dot(p0,axis)),BBox1f(dot(p1,axis)),BBox1f(dot(p2,axis)),BBox1f(dot(p3,axis)));
        }
        
        __forceinline void split(MyBezierCurve& left, MyBezierCurve& right, const float t = 0.5f) const
        {
          const V p00 = p0;
          const V p01 = p1;
          const V p02 = p2;
          const V p03 = p3;
          
          const V p10 = lerp(p00,p01,t);
          const V p11 = lerp(p01,p02,t);
          const V p12 = lerp(p02,p03,t);
          const V p20 = lerp(p10,p11,t);
          const V p21 = lerp(p11,p12,t);
          const V p30 = lerp(p20,p21,t);
          const float u10 = lerp(u.lower,u.upper,t);
          
          new (&left ) MyBezierCurve(BBox1f(u.lower,u10),p00,p10,p20,p30);
          new (&right) MyBezierCurve(BBox1f(u10,u.upper),p30,p21,p12,p03);
        }
        
        __forceinline V eval(float t) const
        {
          const V p00 = p0;
          const V p01 = p1;
          const V p02 = p2;
          const V p03 = p3;
          
          const V p10 = lerp(p00,p01,t);
          const V p11 = lerp(p01,p02,t);
          const V p12 = lerp(p02,p03,t);
          const V p20 = lerp(p10,p11,t);
          const V p21 = lerp(p11,p12,t);
          const V p30 = lerp(p20,p21,t);
          
          return p30;
        }
        
        __forceinline V eval_dt(float u) const
        {
          const float t1 = u;
          const float t0 = 1.0f-t1;
          const float B0 = -(t0*t0);
          const float B1 = madd(-2.0f,t0*t1,t0*t0);
          const float B2 = msub(+2.0f,t0*t1,t1*t1);
          const float B3 = +(t1*t1);
          return float(3.0f)*(B0*p0+B1*p1+B2*p2+B3*p3);
        }
        
        __forceinline MyBezierCurve clip(const BBox1f& u1) const
        {
          V f0 = eval(u1.lower);
          V df0 = eval_dt(u1.lower);
          V f1 = eval(u1.upper);
          V df1 = eval_dt(u1.upper);
          float s = u1.upper-u1.lower;
          float lower = lerp(u.lower,u.upper,u1.lower);
          float upper = lerp(u.lower,u.upper,u1.upper);
          return MyBezierCurve(BBox1f(lower,upper),f0,f0+s*(1.0f/3.0f)*df0,f1-s*(1.0f/3.0f)*df1,f1);
        }
        
        friend std::ostream& operator<<(std::ostream& cout, const MyBezierCurve& a) {
          return cout << "MyBezierCurve ( (" << a.u.lower << ", " << a.u.upper << "), " << a.p0 << ", " << a.p1 << ", " << a.p2 << ", " << a.p3 << ")";
        }
      };
    
    typedef MyBezierCurve<float> MyBezierCurve1f;
    typedef MyBezierCurve<Vec2f> MyBezierCurve2f;
    typedef MyBezierCurve<Vec3fa> MyBezierCurve3fa;
    
    template<> inline int MyBezierCurve<float>::maxRoots() const
    {
      float eps = 1E-4f;
      bool neg0 = p0 <= 0.0f; bool zero0 = fabs(p0) < eps;
      bool neg1 = p1 <= 0.0f; bool zero1 = fabs(p1) < eps;
      bool neg2 = p2 <= 0.0f; bool zero2 = fabs(p2) < eps;
      bool neg3 = p3 <= 0.0f; bool zero3 = fabs(p3) < eps;
      return (neg0 != neg1 || zero0) + (neg1 != neg2 || zero1) + (neg2 != neg3 || zero2 || zero3);
    }
    
    template<> inline int MyBezierCurve<BBox1f>::maxRoots() const {
      return numRoots(p0,p1) + numRoots(p1,p2) + numRoots(p2,p3);
    }
    
    template<typename V>
      struct OrientedBezierCurve
      {
        BBox1f u;
        BBox1f v;
        MyBezierCurve<V> L;
        MyBezierCurve<V> R;
        
        __forceinline OrientedBezierCurve() {}
        
        __forceinline OrientedBezierCurve(const OrientedBezierCurve<V>& curve)
          : u(curve.u), v(curve.v), L(curve.L), R(curve.R) {}
        
        __forceinline OrientedBezierCurve(const MyBezierCurve<V>& L, const MyBezierCurve<V>& R)
          : u(0.0f,1.0f), v(0.0f,1.0f), L(L), R(R) {}
        
        __forceinline OrientedBezierCurve(const BBox1f& u, const BBox1f& v, const MyBezierCurve<V>& L, const MyBezierCurve<V>& R)
          : u(u), v(v), L(L), R(R) {}

        __forceinline BBox<V> bounds() const {
          return merge(L.bounds(),R.bounds());
        }

        __forceinline BBox1f bounds(const V& axis) const {
          return merge(L.bounds(axis),R.bounds(axis));
        }
        
        __forceinline MyBezierCurve<BBox1f> reduce_v() const
        {
          const BBox1f p0(min(L.p0,R.p0),max(L.p0,R.p0));
          const BBox1f p1(min(L.p1,R.p1),max(L.p1,R.p1));
          const BBox1f p2(min(L.p2,R.p2),max(L.p2,R.p2));
          const BBox1f p3(min(L.p3,R.p3),max(L.p3,R.p3));
          return MyBezierCurve<BBox1f>(u,p0,p1,p2,p3);
        }
        
        __forceinline LinearCurve<BBox1f> reduce_u() const
        {
          const BBox1f p0(min(L.p0,L.p1,L.p2,L.p3),max(L.p0,L.p1,L.p2,L.p3));
          const BBox1f p1(min(R.p0,R.p1,R.p2,R.p3),max(R.p0,R.p1,R.p2,R.p3));
          return LinearCurve<BBox1f>(v,p0,p1);
        }
        
        __forceinline OrientedBezierCurve<float> xfm(const V& dx) const {
          return OrientedBezierCurve<float>(u,v,L.xfm(dx),R.xfm(dx));
        }

        __forceinline OrientedBezierCurve<float> xfm(const V& dx, const V& p) const {
          return OrientedBezierCurve<float>(u,v,L.xfm(dx,p),R.xfm(dx,p));
        }
        
        __forceinline OrientedBezierCurve<Vec2f> xfm(const V& dx, const V& dy, const V& p) const {
          return OrientedBezierCurve<Vec2f>(u,v,L.xfm(dx,dy,p),R.xfm(dx,dy,p));
        }
        
        __forceinline OrientedBezierCurve clip_u(const BBox1f& u1) const
        {
          float lower = lerp(u.lower,u.upper,u1.lower);
          float upper = lerp(u.lower,u.upper,u1.upper);
          return OrientedBezierCurve(BBox1f(lower,upper),v,L.clip(u1),R.clip(u1));
        }
        
        __forceinline OrientedBezierCurve clip_v(const BBox1f& v1) const
        {
          float lower = lerp(v.lower,v.upper,v1.lower);
          float upper = lerp(v.lower,v.upper,v1.upper);
          
          V L0 = lerp(L.p0,R.p0,v1.lower);
          V L1 = lerp(L.p1,R.p1,v1.lower);
          V L2 = lerp(L.p2,R.p2,v1.lower);
          V L3 = lerp(L.p3,R.p3,v1.lower);
          
          V R0 = lerp(L.p0,R.p0,v1.upper);
          V R1 = lerp(L.p1,R.p1,v1.upper);
          V R2 = lerp(L.p2,R.p2,v1.upper);
          V R3 = lerp(L.p3,R.p3,v1.upper);
          
          return OrientedBezierCurve(u,BBox1f(lower,upper),MyBezierCurve<V>(L.u,L0,L1,L2,L3),MyBezierCurve<V>(R.u,R0,R1,R2,R3));
        }
        
        __forceinline void split_u(OrientedBezierCurve& left, OrientedBezierCurve& right, const float u0 = 0.5f) const
        {
          MyBezierCurve<V> L0,L1; L.split(L0,L1,u0);
          MyBezierCurve<V> R0,R1; R.split(R0,R1,u0);
          float uc = lerp(u.lower,u.upper,u0);
          new (&left ) OrientedBezierCurve(BBox1f(u.lower,uc),v,L0,R0);
          new (&right) OrientedBezierCurve(BBox1f(uc,u.upper),v,L1,R1);
        }

        __forceinline V eval(const float u, const float v) const
        {
          V l = L.eval(u);
          V r = R.eval(u);
          return lerp(l,r,v);
        }

        __forceinline V eval_du(const float u, const float v) const
        {
          V l = L.eval_dt(u);
          V r = R.eval_dt(u);
          return lerp(l,r,v);
        }

        __forceinline V eval_dv(const float u, const float v) const
        {
          V l = L.eval(u);
          V r = R.eval(u);
          return r-l;
        }

        __forceinline V axis_u() const {
          return (L.p3-L.p0)+(R.p3-R.p0);
        }

        __forceinline V axis_v() const {
          return (R.p0-L.p0)+(R.p3-L.p3);
        }
        
        friend std::ostream& operator<<(std::ostream& cout, const OrientedBezierCurve& a)
        {
          return cout << "OrientedBezierCurve" << std::endl
                      << "{" << std::endl
                      << "  u = (" << a.u.lower << ", " << a.u.upper << "), " << std::endl
                      << "  v = (" << a.v.lower << ", " << a.v.upper << "), " << std::endl
                      << "  L = " << a.L << ", " << std::endl
                      << "  R = " << a.R << std::endl
                      << "}";
        }
      };
    
    typedef OrientedBezierCurve<float> OrientedBezierCurve1f;
    typedef OrientedBezierCurve<Vec2f> OrientedBezierCurve2f;
    typedef OrientedBezierCurve<Vec3fa> OrientedBezierCurve3fa;

    template<typename Epilog>
      struct OrientedBezierCurveIntersector
      {
        Ray& ray;
        LinearSpace3fa space;
        OrientedBezierCurve3fa curve3d;
        const Epilog& epilog;
        bool isHit;

        __forceinline OrientedBezierCurveIntersector (Ray& ray, const OrientedBezierCurve3fa& curve3d, const Epilog& epilog)
          : ray(ray), space(frame(ray.dir)), curve3d(curve3d), epilog(epilog), isHit(false) {}
        
        __forceinline BBox1f solve_linear(const float u0, const float u1, const float& p0, const float& p1)
        {
          if (p1 == p0) {
            if (p0 == 0.0f) return BBox1f(u0,u1);
            else return BBox1f(empty);
          }
          const float t = -p0/(p1-p0);
          const float tt = lerp(u0,u1,t);
          return BBox1f(tt);
        }
        
        __forceinline void solve_linear(const float u0, const float u1, const BBox1f& p0, const BBox1f& p1, BBox1f& u)
        {
          if (sign(p0.lower) != sign(p0.upper)) u.extend(u0);
          if (sign(p0.lower) != sign(p1.lower)) u.extend(solve_linear(u0,u1,p0.lower,p1.lower));
          if (sign(p0.upper) != sign(p1.upper)) u.extend(solve_linear(u0,u1,p0.upper,p1.upper));
          if (sign(p1.lower) != sign(p1.upper)) u.extend(u1);
        }
        
        __forceinline BBox1f bezier_clipping(const MyBezierCurve<BBox1f>& curve)
        {
          BBox1f u = empty;
          solve_linear(0.0f/3.0f,1.0f/3.0f,curve.p0,curve.p1,u);
          solve_linear(0.0f/3.0f,2.0f/3.0f,curve.p0,curve.p2,u);
          solve_linear(0.0f/3.0f,3.0f/3.0f,curve.p0,curve.p3,u);
          solve_linear(1.0f/3.0f,2.0f/3.0f,curve.p1,curve.p2,u);
          solve_linear(1.0f/3.0f,3.0f/3.0f,curve.p1,curve.p3,u);
          solve_linear(2.0f/3.0f,3.0f/3.0f,curve.p2,curve.p3,u);
          return embree::intersect(u,BBox1f(0.0f,1.0f));
        }
        
        __forceinline BBox1f bezier_clipping(const LinearCurve<BBox1f>& curve)
        {
          BBox1f v = empty;
          solve_linear(0.0f,1.0f,curve.p0,curve.p1,v);
          return embree::intersect(v,BBox1f(0.0f,1.0f));
        }
        
        __forceinline void intersect_u(const OrientedBezierCurve2f& curve2, const Vec2f& du)
        {
          //PRINT("intersect_u");
          
          const OrientedBezierCurve1f curve1 = curve2.xfm(du);
          //PRINT(curve1);
          MyBezierCurve<BBox1f> curve0 = curve1.reduce_v();
          //PRINT(curve0);
          
          int roots = curve0.maxRoots();
          //PRINT(roots);
          if (roots == 0) return;
          
          if (roots == 1)
          {
            //PRINT("bezier_clipping_u");
            const BBox1f u = bezier_clipping(curve0);
            //PRINT(u);
            if (u.empty()) return;
            OrientedBezierCurve2f curve2a = curve2.clip_u(u);
            //PRINT(curve2a);
            intersect(curve2a);
            return;
          }
          
          OrientedBezierCurve2f curve2a, curve2b;
          curve2.split_u(curve2a,curve2b);
          intersect(curve2a);
          intersect(curve2b);
        }
        
        __forceinline void intersect_v(const OrientedBezierCurve2f& curve2, const Vec2f& dv)
        {
          //PRINT("intersect_v");
          
          const OrientedBezierCurve1f curve1 = curve2.xfm(dv);
          LinearCurve<BBox1f> curve0 = curve1.reduce_u();
          
          if (!curve0.hasRoot())
            return;
          
          //PRINT("bezier_clipping_v");
          const BBox1f v = bezier_clipping(curve0);
          //PRINT(v);
          if (v.empty()) return;
          OrientedBezierCurve2f curve2a = curve2.clip_v(v);
          //PRINT(curve2a);
          intersect(curve2a);
        }

        __forceinline void intersect_uv(const OrientedBezierCurve2f& curve2)
        {
          const Vec2f du = normalize(curve2.axis_u());
          const Vec2f dv = normalize(curve2.axis_v());
          
          DBG(PRINT(du));
          DBG(PRINT(dv));
          const OrientedBezierCurve1f curve1v = curve2.xfm(dv);
          DBG(PRINT(curve1v));
          LinearCurve<BBox1f> curve0v = curve1v.reduce_u();
          DBG(PRINT(curve0v));
          
          if (!curve0v.hasRoot())
            return;
          
          DBG(PRINT("bezier_clipping_v"));
          const BBox1f v = bezier_clipping(curve0v);
          //PRINT(v);
          if (v.empty()) return;
          OrientedBezierCurve2f curve2a = curve2.clip_v(v);
          DBG(PRINT(curve2a));

#if 1
          //PRINT("intersect_u");
          
          const OrientedBezierCurve1f curve1u = curve2a.xfm(du);
          DBG(PRINT(curve1u));
          MyBezierCurve<BBox1f> curve0u = curve1u.reduce_v();
          DBG(PRINT(curve0u));
          
          int roots = curve0u.maxRoots();
          DBG(PRINT(roots));
          if (roots == 0) return;
          
          if (roots == 1)
          {
            DBG(PRINT("bezier_clipping_u"));
            const BBox1f u = bezier_clipping(curve0u);
            //PRINT(u);
            if (u.empty()) return;
            OrientedBezierCurve2f curve2b = curve2a.clip_u(u);
            DBG(PRINT(curve2b));
            intersect(curve2b);
            return;
          }
#endif
          DBG(PRINT("split"));
          OrientedBezierCurve2f curve2l, curve2r;
          curve2a.split_u(curve2l,curve2r);
          DBG(PRINT(curve2l));
          DBG(PRINT(curve2r));
          intersect(curve2l);
          intersect(curve2r);
        }
        
        void intersect(const OrientedBezierCurve2f& curve2)
        {
          BBox2f bounds = curve2.bounds();
          if (bounds.upper.x < 0.0f) return;
          if (bounds.upper.y < 0.0f) return;
          if (bounds.lower.x > 0.0f) return;
          if (bounds.lower.y > 0.0f) return;
          
          if (max(curve2.u.size(),curve2.v.size()) < 1E-4f)
          {
            DBG(PRINT2("solution",curve2));
            const float u = curve2.u.center();
            const float v = curve2.v.center();
            OrientedBezierCurve1f curve_z = curve3d.xfm(space.vz,ray.org);
            const float t = curve_z.eval(u,v)/length(ray.dir);
            if (t >= ray.tnear() && t <= ray.tfar()) {
              const Vec3fa Ng = cross(curve3d.eval_du(u,v),curve3d.eval_dv(u,v));
              BezierCurveHit hit(t,u,v,Ng);
              isHit |= epilog(hit);
            }
            return;
          }
          
          DBG(std::cout << std::endl << std::endl);
          DBG(std::cout << "intersect" << std::endl);
          DBG(PRINT(curve2));
          //PRINT(du);
          //PRINT(dv);
          //PRINT(length(du));
          //PRINT(length(dv));
          intersect_uv(curve2);
          //if (length(du) > length(dv)) intersect_u(curve2,normalize(du));
          //else                         intersect_v(curve2,normalize(dv));
        }
        
        bool intersect()
        {
          /* transform bezier curve into ray space */
          
          //space.vx = -space.vx;
          //space.vy = -space.vy;
          //PRINT(space);
          OrientedBezierCurve2f curve2 = curve3d.xfm(space.vx,space.vy,ray.org);
          //PRINT(curve2);
          //curve2 = curve2.clip_u(BBox1f(0.00484675, 0.0326178)).clip_v(BBox1f(0.961587, 1));
          intersect(curve2);
          return isHit;
        }

        void solve_newton_raphson(const OrientedBezierCurve2f& curve2)
        {
          Vec2f uv(0.5f,0.5f);
                      
          for (size_t i=0; i<20; i++)
          {
            const Vec2f f    = curve2.eval(uv.x,uv.y);
            const Vec2f dfdu = curve2.eval_du(uv.x,uv.y);
            const Vec2f dfdv = curve2.eval_dv(uv.x,uv.y);
            const LinearSpace2f rcp_J = rcp(LinearSpace2f(dfdu,dfdv));
            const Vec2f duv = rcp_J*f;
            uv -= duv;

            if (max(fabs(duv.x),fabs(duv.y)) < 1E-4f)
            {
              DBG(PRINT2("solution",curve2));
              const float u = lerp(curve2.u.lower,curve2.u.upper,uv.x);
              const float v = lerp(curve2.v.lower,curve2.v.upper,uv.y);
              if (!(u >= 0.0f && u <= 1.0f)) return; // rejects NaNs
              if (!(v >= 0.0f && v <= 1.0f)) return; // rejects NaNs
              const OrientedBezierCurve1f curve_z = curve3d.xfm(space.vz,ray.org);
              const float t = curve_z.eval(u,v)/length(ray.dir);
              if (!(t > ray.tnear() && t < ray.tfar())) return; // rejects NaNs
              const Vec3fa Ng = cross(curve3d.eval_du(u,v),curve3d.eval_dv(u,v));
              BezierCurveHit hit(t,u,v,Ng);
              isHit |= epilog(hit);
              return;
            }
          }       
        }

        void solve_newton_raphson2(const OrientedBezierCurve2f& curve2)
        {
          Vec2f uv(0.5f,0.5f);
          const Vec2f dfdu = curve2.eval_du(uv.x,uv.y);
          const Vec2f dfdv = curve2.eval_dv(uv.x,uv.y);
          const LinearSpace2f rcp_J = rcp(LinearSpace2f(dfdu,dfdv));
            
          for (size_t i=0; i<20; i++)
          {
            const Vec2f f    = curve2.eval(uv.x,uv.y);
            const Vec2f duv = rcp_J*f;
            uv -= duv;

            if (max(fabs(duv.x),fabs(duv.y)) < 1E-4f)
            {
              DBG(PRINT2("solution",curve2));
              const float u = lerp(curve2.u.lower,curve2.u.upper,uv.x);
              const float v = lerp(curve2.v.lower,curve2.v.upper,uv.y);
              if (!(u >= 0.0f && u <= 1.0f)) return; // rejects NaNs
              if (!(v >= 0.0f && v <= 1.0f)) return; // rejects NaNs
              const OrientedBezierCurve1f curve_z = curve3d.xfm(space.vz,ray.org);
              const float t = curve_z.eval(u,v)/length(ray.dir);
              if (!(t > ray.tnear() && t < ray.tfar())) return; // rejects NaNs
              const Vec3fa Ng = cross(curve3d.eval_du(u,v),curve3d.eval_dv(u,v));
              BezierCurveHit hit(t,u,v,Ng);
              isHit |= epilog(hit);
              return;
            }
          }       
        }

        void krawczyk(const OrientedBezierCurve2f& curve2)
        {
          
        }
        
        void intersect_newton_raphson(const OrientedBezierCurve2f& curve2)
        {
          BBox2f bounds = curve2.bounds();
          if (bounds.upper.x < 0.0f) return;
          if (bounds.upper.y < 0.0f) return;
          if (bounds.lower.x > 0.0f) return;
          if (bounds.lower.y > 0.0f) return;

          Vec2f du = curve2.axis_u();
          Vec2f dv = curve2.axis_v();
          Vec2f ndu = Vec2f(-du.y,du.x);
          Vec2f ndv = Vec2f(-dv.y,dv.x);
          BBox1f boundsu = curve2.bounds(ndu);
          BBox1f boundsv = curve2.bounds(ndv);
          if (boundsu.upper < 0.0f) return;
          if (boundsv.upper < 0.0f) return;
          if (boundsu.lower > 0.0f) return;
          if (boundsv.lower > 0.0f) return;

          if (curve2.u.size() < 0.05f)
            return solve_newton_raphson2(curve2);
          
          OrientedBezierCurve2f curve2l, curve2r;
          curve2.split_u(curve2l,curve2r);
          intersect_newton_raphson(curve2l);
          intersect_newton_raphson(curve2r);
        }

        __forceinline void intersect_newton_raphson2(const OrientedBezierCurve2f& curve2)
        {
          {
          BBox2f bounds = curve2.bounds();
          if (bounds.upper.x < 0.0f) return;
          if (bounds.upper.y < 0.0f) return;
          if (bounds.lower.x > 0.0f) return;
          if (bounds.lower.y > 0.0f) return;

          Vec2f du = curve2.axis_u();
          Vec2f dv = curve2.axis_v();
          Vec2f ndu = Vec2f(-du.y,du.x);
          Vec2f ndv = Vec2f(-dv.y,dv.x);
          BBox1f boundsu = curve2.bounds(ndu);
          BBox1f boundsv = curve2.bounds(ndv);
          if (boundsu.upper < 0.0f) return;
          if (boundsv.upper < 0.0f) return;
          if (boundsu.lower > 0.0f) return;
          if (boundsv.lower > 0.0f) return;
          }
          
          const Vec2f du = normalize(curve2.axis_u());
          const Vec2f dv = normalize(curve2.axis_v());
       
          DBG(PRINT(du));
          DBG(PRINT(dv));
          const OrientedBezierCurve1f curve1v = curve2.xfm(dv);
          DBG(PRINT(curve1v));
          LinearCurve<BBox1f> curve0v = curve1v.reduce_u();
          DBG(PRINT(curve0v));
          
          if (!curve0v.hasRoot())
            return;
          
          DBG(PRINT("bezier_clipping_v"));
          const BBox1f v = bezier_clipping(curve0v);
          //PRINT(v);
          if (v.empty()) return;
          OrientedBezierCurve2f curve2a = curve2.clip_v(v);
          DBG(PRINT(curve2a));

#if 1
          //PRINT("intersect_u");
          
          const OrientedBezierCurve1f curve1u = curve2a.xfm(du);
          DBG(PRINT(curve1u));
          MyBezierCurve<BBox1f> curve0u = curve1u.reduce_v();
          DBG(PRINT(curve0u));
          
          int roots = curve0u.maxRoots();
          DBG(PRINT(roots));
          if (roots == 0) return;
          
          if (roots == 1)
          {
            DBG(PRINT("bezier_clipping_u"));
            const BBox1f u = bezier_clipping(curve0u);
            //PRINT(u);
            if (u.empty()) return;
            OrientedBezierCurve2f curve2b = curve2a.clip_u(u);
            DBG(PRINT(curve2b));
            solve_newton_raphson(curve2b);
            return;
          }
#endif
          DBG(PRINT("split"));
          OrientedBezierCurve2f curve2l, curve2r;
          curve2a.split_u(curve2l,curve2r);
          DBG(PRINT(curve2l));
          DBG(PRINT(curve2r));
          intersect_newton_raphson2(curve2l);
          intersect_newton_raphson2(curve2r);
        }
        

        bool intersect_newton_raphson()
        {
          OrientedBezierCurve2f curve2 = curve3d.xfm(space.vx,space.vy,ray.org);
          intersect_newton_raphson(curve2);
          return isHit;
        }
      };

    
    struct OrientedBezierCurve1Intersector1
    {
      __forceinline OrientedBezierCurve1Intersector1() {}
      
      __forceinline OrientedBezierCurve1Intersector1(const Ray& ray, const void* ptr) {}
      
      template<typename Epilog>
      __noinline bool intersect(Ray& ray,
                                const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3,
                                const Vec3fa& n0, const Vec3fa& n1, const Vec3fa& n2, const Vec3fa& n3,
                                const Epilog& epilog) const
      {
        STAT3(normal.trav_prims,1,1,1);
        
        MyBezierCurve3fa center(v0,v1,v2,v3);
        Vec3fa d0 = v0.w*normalize(cross(n0,center.eval_dt(0.0f/3.0f)));
        Vec3fa d1 = v1.w*normalize(cross(n1,center.eval_dt(1.0f/3.0f)));
        Vec3fa d2 = v2.w*normalize(cross(n2,center.eval_dt(2.0f/3.0f)));
        Vec3fa d3 = v3.w*normalize(cross(n3,center.eval_dt(3.0f/3.0f)));

        MyBezierCurve3fa L(v0-d0,v1-d1,v2-d2,v3-d3);
        MyBezierCurve3fa R(v0+d0,v1+d1,v2+d2,v3+d3);
        OrientedBezierCurve3fa curve(L,R);
        //return OrientedBezierCurveIntersector<Epilog>(ray,curve,epilog).intersect();
        return OrientedBezierCurveIntersector<Epilog>(ray,curve,epilog).intersect_newton_raphson();
      }
    };
  }
}
