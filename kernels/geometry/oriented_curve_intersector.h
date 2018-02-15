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
#include "../../common/math/interval.h"

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

    __forceinline int numRoots(const Interval1f& p0, const Interval1f& p1)
    {
      float eps = 1E-4f;
      bool neg0 = p0.lower < eps; bool pos0 = p0.upper > -eps;
      bool neg1 = p1.lower < eps; bool pos1 = p1.upper > -eps;
      return (neg0 && pos1) || (pos0 && neg1) || (neg0 && pos0) || (neg1 && pos1);
    }
    
    template<typename V>
      struct LinearBezierCurve
      {
        V p0,p1;
        
        __forceinline LinearBezierCurve () {}

        __forceinline LinearBezierCurve (const LinearBezierCurve& other)
          : p0(other.p0), p1(other.p1) {}
        
        __forceinline LinearBezierCurve& operator= (const LinearBezierCurve& other) {
          p0 = other.p0; p1 = other.p1; return *this;
        }
        
        __forceinline LinearBezierCurve (const V& p0, const V& p1)
          : p0(p0), p1(p1) {}

        __forceinline V begin() const { return p0; }
        __forceinline V end  () const { return p1; }
        
        bool hasRoot() const;
        
        friend std::ostream& operator<<(std::ostream& cout, const LinearBezierCurve& a) {
          return cout << "LinearBezierCurve (" << a.p0 << ", " << a.p1 << ")";
        }
      };
    
    template<> __forceinline bool LinearBezierCurve<Interval1f>::hasRoot() const {
      return numRoots(p0,p1);
    }

    template<typename V>
      struct QuadraticBezierCurve
      {
        V p0,p1,p2;
        
        __forceinline QuadraticBezierCurve () {}
        
        __forceinline QuadraticBezierCurve (const QuadraticBezierCurve& other)
          : p0(other.p0), p1(other.p1), p2(other.p2) {}
        
        __forceinline QuadraticBezierCurve& operator= (const QuadraticBezierCurve& other) {
          p0 = other.p0; p1 = other.p1; p2 = other.p2; return *this;
        }
          
        __forceinline QuadraticBezierCurve (const V& p0, const V& p1, const V& p2)
          : p0(p0), p1(p1), p2(p2) {}
        
        __forceinline V begin() const { return p0; }
        __forceinline V end  () const { return p2; }
             
        __forceinline V interval() const {
          return merge(p0,p1,p2);
        }

        __forceinline BBox<V> bounds() const {
          return merge(BBox<V>(p0),BBox<V>(p1),BBox<V>(p2));
        }

        friend std::ostream& operator<<(std::ostream& cout, const QuadraticBezierCurve& a) {
          return cout << "QuadraticBezierCurve ( (" << a.u.lower << ", " << a.u.upper << "), " << a.p0 << ", " << a.p1 << ", " << a.p2 << ")";
        }
      };


    typedef QuadraticBezierCurve<float> QuadraticBezierCurve1f;
    typedef QuadraticBezierCurve<Vec2fa> QuadraticBezierCurve2fa;
    typedef QuadraticBezierCurve<Vec3fa> QuadraticBezierCurve3fa;
  
    template<typename V>
      struct CubicBezierCurve
      {
        V p0,p1,p2,p3;
        
        __forceinline CubicBezierCurve () {}

        template<typename T1>
        __forceinline CubicBezierCurve (const CubicBezierCurve<T1>& other)
          : p0(other.p0), p1(other.p1), p2(other.p2), p3(other.p3) {}

        __forceinline CubicBezierCurve& operator= (const CubicBezierCurve& other) {
          p0 = other.p0; p1 = other.p1; p2 = other.p2; p3 = other.p3; return *this;
        }
          
        __forceinline CubicBezierCurve (const V& p0, const V& p1, const V& p2, const V& p3)
            : p0(p0), p1(p1), p2(p2), p3(p3) {}
        
        __forceinline V begin() const { return p0; }
        __forceinline V end  () const { return p3; }
             
        __forceinline CubicBezierCurve<float> xfm(const V& dx) const {
          return CubicBezierCurve<float>(dot(p0,dx),dot(p1,dx),dot(p2,dx),dot(p3,dx));
        }

        __forceinline CubicBezierCurve<vfloatx> vxfm(const V& dx) const {
          return CubicBezierCurve<vfloatx>(dot(p0,dx),dot(p1,dx),dot(p2,dx),dot(p3,dx));
        }

        __forceinline CubicBezierCurve<float> xfm(const V& dx, const V& p) const {
          return CubicBezierCurve<float>(dot(p0-p,dx),dot(p1-p,dx),dot(p2-p,dx),dot(p3-p,dx));
        }
        
        __forceinline CubicBezierCurve<Vec2fa> xfm(const V& dx, const V& dy, const V& p) const;

        __forceinline int maxRoots() const;

        __forceinline BBox<V> bounds() const {
          return merge(BBox<V>(p0),BBox<V>(p1),BBox<V>(p2),BBox<V>(p3));
        }

        __forceinline friend CubicBezierCurve operator +( const CubicBezierCurve& a, const CubicBezierCurve& b ) {
          return CubicBezierCurve(a.p0+b.p0,a.p1+b.p1,a.p2+b.p2,a.p3+b.p3);
        }
        
        __forceinline friend CubicBezierCurve operator -( const CubicBezierCurve& a, const CubicBezierCurve& b ) {
          return CubicBezierCurve(a.p0-b.p0,a.p1-b.p1,a.p2-b.p2,a.p3-b.p3);
        }

        __forceinline friend CubicBezierCurve operator -( const CubicBezierCurve& a, const V& b ) {
          return CubicBezierCurve(a.p0-b,a.p1-b,a.p2-b,a.p3-b);
        }

        __forceinline friend CubicBezierCurve operator *( const V& a, const CubicBezierCurve& b ) {
          return CubicBezierCurve(a*b.p0,a*b.p1,a*b.p2,a*b.p3);
        }

        __forceinline friend CubicBezierCurve madd( const V& a, const CubicBezierCurve& b,  const CubicBezierCurve& c) {
          return CubicBezierCurve(madd(a,b.p0,c.p0),madd(a,b.p1,c.p1),madd(a,b.p2,c.p2),madd(a,b.p3,c.p3));
        }

        __forceinline friend CubicBezierCurve lerp ( const CubicBezierCurve& a, const CubicBezierCurve& b, const V& t ) {
          return madd((V(1.0f)-t),a,t*b);
        }

        __forceinline friend CubicBezierCurve merge ( const CubicBezierCurve& a, const CubicBezierCurve& b ) {
          return CubicBezierCurve(merge(a.p0,b.p0),merge(a.p1,b.p1),merge(a.p2,b.p2),merge(a.p3,b.p3));
        }

        __forceinline void split(CubicBezierCurve& left, CubicBezierCurve& right, const float t = 0.5f) const
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
          
          new (&left ) CubicBezierCurve(p00,p10,p20,p30);
          new (&right) CubicBezierCurve(p30,p21,p12,p03);
        }

        __forceinline CubicBezierCurve<Vec2vfx> split() const
        {
          const float u0 = 0.0f, u1 = 1.0f;
          const float dscale = (u1-u0)*(1.0f/(3.0f*(VSIZEX-1)));
          const vfloatx vu0 = lerp(u0,u1,vfloatx(step)*(1.0f/(VSIZEX-1)));
          Vec2vfx P0, dP0du; evalN(vu0,P0,dP0du); dP0du = dP0du * Vec2vfx(dscale);
          const Vec2vfx P3 = shift_right_1(P0);
          const Vec2vfx dP3du = shift_right_1(dP0du); 
          const Vec2vfx P1 = P0 + dP0du; 
          const Vec2vfx P2 = P3 - dP3du;
          return CubicBezierCurve<Vec2vfx>(P0,P1,P2,P3);
        }

        __forceinline CubicBezierCurve<Vec2vfx> split(const BBox1f& u) const
        {
          const float u0 = u.lower, u1 = u.upper;
          const float dscale = (u1-u0)*(1.0f/(3.0f*(VSIZEX-1)));
          const vfloatx vu0 = lerp(u0,u1,vfloatx(step)*(1.0f/(VSIZEX-1)));
          Vec2vfx P0, dP0du; evalN(vu0,P0,dP0du); dP0du = dP0du * Vec2vfx(dscale);
          const Vec2vfx P3 = shift_right_1(P0);
          const Vec2vfx dP3du = shift_right_1(dP0du); 
          const Vec2vfx P1 = P0 + dP0du; 
          const Vec2vfx P2 = P3 - dP3du;
          return CubicBezierCurve<Vec2vfx>(P0,P1,P2,P3);
        }
        
        __forceinline void eval(float t, V& p, V& dp) const
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
          
          p = p30;
          dp = V(3.0f)*(p21-p20);
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

        __forceinline V eval_dt(float t) const
        {
          const V p00 = p1-p0;
          const V p01 = p2-p1;
          const V p02 = p3-p2;
          const V p10 = lerp(p00,p01,t);
          const V p11 = lerp(p01,p02,t);
          const V p20 = lerp(p10,p11,t);
          return V(3.0f)*p20;
        }
        
        __forceinline void evalN(const vfloatx& t, Vec2vfx& p, Vec2vfx& dp) const
        {
          const Vec2vfx p00 = p0;
          const Vec2vfx p01 = p1;
          const Vec2vfx p02 = p2;
          const Vec2vfx p03 = p3;
          
          const Vec2vfx p10 = lerp(p00,p01,t);
          const Vec2vfx p11 = lerp(p01,p02,t);
          const Vec2vfx p12 = lerp(p02,p03,t);
          
          const Vec2vfx p20 = lerp(p10,p11,t);
          const Vec2vfx p21 = lerp(p11,p12,t);
          
          const Vec2vfx p30 = lerp(p20,p21,t);
          
          p = p30;
          dp = vfloatx(3.0f)*(p21-p20);
        }
        
        __forceinline CubicBezierCurve clip(const Interval1f& u1) const
        {
          V f0,df0; eval(u1.lower,f0,df0);
          V f1,df1; eval(u1.upper,f1,df1);
          float s = u1.upper-u1.lower;
          return CubicBezierCurve(f0,f0+s*(1.0f/3.0f)*df0,f1-s*(1.0f/3.0f)*df1,f1);
        }

        __forceinline QuadraticBezierCurve<V> derivative() const
        {
          const V q0 = 3.0f*(p1-p0);
          const V q1 = 3.0f*(p2-p1);
          const V q2 = 3.0f*(p3-p2);
          return QuadraticBezierCurve<V>(q0,q1,q2);
        }

        __forceinline BBox<V> derivative_bounds(const Interval1f& u1) const
        {
          V f0,df0; eval(u1.lower,f0,df0);
          V f3,df3; eval(u1.upper,f3,df3);
          const float s = u1.upper-u1.lower;
          const V f1 = f0+s*(1.0f/3.0f)*df0;
          const V f2 = f3-s*(1.0f/3.0f)*df3;
          const V q0 = s*df0;
          const V q1 = 3.0f*(f2-f1);
          const V q2 = s*df3;
          return merge(BBox<V>(q0),BBox<V>(q1),BBox<V>(q2));
        }
        
        friend std::ostream& operator<<(std::ostream& cout, const CubicBezierCurve& a) {
          return cout << "CubicBezierCurve (" << a.p0 << ", " << a.p1 << ", " << a.p2 << ", " << a.p3 << ")";
        }
      };
    
    typedef CubicBezierCurve<float> CubicBezierCurve1f;
    typedef CubicBezierCurve<Vec2fa> CubicBezierCurve2fa;
    typedef CubicBezierCurve<Vec3fa> CubicBezierCurve3fa;

    template<>
      __forceinline CubicBezierCurve<Vec2fa> CubicBezierCurve<Vec3fa>::xfm(const Vec3fa& dx, const Vec3fa& dy, const Vec3fa& p) const
    {
#if 0
      Vec3<vfloat4> p0123;
      transpose(vfloat4(p0),vfloat4(p1),vfloat4(p2),vfloat4(p3),p0123.x,p0123.y,p0123.z);
#if 0 //defined(__AVX__)
      vfloat8 q0123xy = dot(Vec3<vfloat8>(p0123)-Vec3<vfloat8>(p),Vec3<vfloat8>(vfloat8(vfloat4(dx.x),vfloat4(dy.x)),vfloat8(vfloat4(dx.y),vfloat4(dy.y)),vfloat8(vfloat4(dx.z),vfloat4(dy.z))));
      vfloat4 q0123x = extract4<0>(q0123xy);
      vfloat4 q0123y = extract4<1>(q0123xy);
#else
      vfloat4 q0123x = dot(p0123-Vec3<vfloat4>(p),Vec3<vfloat4>(dx));
      vfloat4 q0123y = dot(p0123-Vec3<vfloat4>(p),Vec3<vfloat4>(dy));
#endif
      vfloat4 q0,q1,q2,q3;
      transpose(q0123x,q0123y,q0123x,q0123y,q0,q1,q2,q3);
      return CubicBezierCurve<Vec2fa>(Vec2fa(q0),Vec2fa(q1),Vec2fa(q2),Vec2fa(q3));
#else
#if defined (__AVX__)
      const vfloat8 q0xy(dot(vfloat8(vfloat4(p0-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
      const vfloat8 q1xy(dot(vfloat8(vfloat4(p1-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
      const vfloat8 q2xy(dot(vfloat8(vfloat4(p2-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
      const vfloat8 q3xy(dot(vfloat8(vfloat4(p3-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
      const Vec2fa q0(unpacklo(extract4<0>(q0xy),extract4<1>(q0xy)));
      const Vec2fa q1(unpacklo(extract4<0>(q1xy),extract4<1>(q1xy)));
      const Vec2fa q2(unpacklo(extract4<0>(q2xy),extract4<1>(q2xy)));
      const Vec2fa q3(unpacklo(extract4<0>(q3xy),extract4<1>(q3xy)));
      return CubicBezierCurve<Vec2fa>(q0,q1,q2,q3);
#else
      const Vec2fa q0(dot(p0-p,dx), dot(p0-p,dy));
      const Vec2fa q1(dot(p1-p,dx), dot(p1-p,dy));
      const Vec2fa q2(dot(p2-p,dx), dot(p2-p,dy));
      const Vec2fa q3(dot(p3-p,dx), dot(p3-p,dy));
      return CubicBezierCurve<Vec2fa>(q0,q1,q2,q3);
#endif
#endif
    }   

    template<> inline int CubicBezierCurve<float>::maxRoots() const
    {
      float eps = 1E-4f;
      bool neg0 = p0 <= 0.0f; bool zero0 = fabs(p0) < eps;
      bool neg1 = p1 <= 0.0f; bool zero1 = fabs(p1) < eps;
      bool neg2 = p2 <= 0.0f; bool zero2 = fabs(p2) < eps;
      bool neg3 = p3 <= 0.0f; bool zero3 = fabs(p3) < eps;
      return (neg0 != neg1 || zero0) + (neg1 != neg2 || zero1) + (neg2 != neg3 || zero2 || zero3);
    }
    
    template<> __forceinline int CubicBezierCurve<Interval1f>::maxRoots() const {
      return numRoots(p0,p1) + numRoots(p1,p2) + numRoots(p2,p3);
    }

    template<typename V>
      struct TensorLinearQuadraticBezierSurface
      {
        QuadraticBezierCurve<V> L;
        QuadraticBezierCurve<V> R;
        
        __forceinline TensorLinearQuadraticBezierSurface() {}
        
        __forceinline TensorLinearQuadraticBezierSurface(const TensorLinearQuadraticBezierSurface<V>& curve)
          : L(curve.L), R(curve.R) {}

        __forceinline TensorLinearQuadraticBezierSurface& operator= (const TensorLinearQuadraticBezierSurface& other) {
          L = other.L; R = other.R; return *this;
        }
        
        __forceinline TensorLinearQuadraticBezierSurface(const QuadraticBezierCurve<V>& L, const QuadraticBezierCurve<V>& R)
          : L(L), R(R) {}
        
        __forceinline BBox<V> bounds() const {
          return merge(L.bounds(),R.bounds());
        }
      };

    template<>
      struct TensorLinearQuadraticBezierSurface<Vec2fa>
      {
        QuadraticBezierCurve<vfloat4> LR;
        
        __forceinline TensorLinearQuadraticBezierSurface() {}
        
        __forceinline TensorLinearQuadraticBezierSurface(const TensorLinearQuadraticBezierSurface<Vec2fa>& curve)
          : LR(curve.LR) {}

        __forceinline TensorLinearQuadraticBezierSurface& operator= (const TensorLinearQuadraticBezierSurface& other) {
          LR = other.LR; return *this;
        }
        
        __forceinline TensorLinearQuadraticBezierSurface(const QuadraticBezierCurve<vfloat4>& LR)
          : LR(LR) {}
        
        __forceinline BBox<Vec2fa> bounds() const
        {
          const BBox<vfloat4> b = LR.bounds();
          const BBox<Vec2fa> bl(Vec2fa(b.lower),Vec2fa(b.upper));
          const BBox<Vec2fa> br(Vec2fa(shuffle<2,3,2,3>(b.lower)),Vec2fa(shuffle<2,3,2,3>(b.upper)));
          return merge(bl,br);
        }
      };
    
    template<typename V>
      struct TensorLinearCubicBezierSurface
      {
        CubicBezierCurve<V> L;
        CubicBezierCurve<V> R;
        
        __forceinline TensorLinearCubicBezierSurface() {}
        
        __forceinline TensorLinearCubicBezierSurface(const TensorLinearCubicBezierSurface& curve)
          : L(curve.L), R(curve.R) {}

        __forceinline TensorLinearCubicBezierSurface& operator= (const TensorLinearCubicBezierSurface& other) {
          L = other.L; R = other.R; return *this;
        }
          
        __forceinline TensorLinearCubicBezierSurface(const CubicBezierCurve<V>& L, const CubicBezierCurve<V>& R)
          : L(L), R(R) {}
        
        __forceinline BBox<V> bounds() const {
          return merge(L.bounds(),R.bounds());
        }

        __forceinline CubicBezierCurve<Interval1f> reduce_v() const {
          return merge(CubicBezierCurve<Interval<V>>(L),CubicBezierCurve<Interval<V>>(R));
        }
        
        __forceinline LinearBezierCurve<Interval1f> reduce_u() const {
          return LinearBezierCurve<Interval1f>(L.bounds(),R.bounds());
        }

        __forceinline TensorLinearCubicBezierSurface<float> xfm(const V& dx) const {
          return TensorLinearCubicBezierSurface<float>(L.xfm(dx),R.xfm(dx));
        }

         __forceinline TensorLinearCubicBezierSurface<vfloatx> vxfm(const V& dx) const {
          return TensorLinearCubicBezierSurface<vfloatx>(L.vxfm(dx),R.vxfm(dx));
        }

        __forceinline TensorLinearCubicBezierSurface<float> xfm(const V& dx, const V& p) const {
          return TensorLinearCubicBezierSurface<float>(L.xfm(dx,p),R.xfm(dx,p));
        }
        
        __forceinline TensorLinearCubicBezierSurface<Vec2fa> xfm(const V& dx, const V& dy, const V& p) const;
        
        __forceinline TensorLinearCubicBezierSurface clip_u(const Interval1f& u) const {
          return TensorLinearCubicBezierSurface(L.clip(u),R.clip(u));
        }
        
        __forceinline TensorLinearCubicBezierSurface clip_v(const Interval1f& v) const {
          return TensorLinearCubicBezierSurface(lerp(L,R,V(v.lower)),lerp(L,R,V(v.upper)));
        }

        __forceinline TensorLinearCubicBezierSurface clip(const Interval1f& u, const Interval1f& v) const {
          return clip_v(v).clip_u(u);
        }

        __forceinline void split_u(TensorLinearCubicBezierSurface& left, TensorLinearCubicBezierSurface& right, const float u = 0.5f) const
        {
          CubicBezierCurve<V> L0,L1; L.split(L0,L1,u);
          CubicBezierCurve<V> R0,R1; R.split(R0,R1,u);
          new (&left ) TensorLinearCubicBezierSurface(L0,R0);
          new (&right) TensorLinearCubicBezierSurface(L1,R1);
        }

        __forceinline TensorLinearCubicBezierSurface<Vec2vfx> vsplit_u(vboolx& valid, const BBox1f& u) const {
          valid = true; clear(valid,VSIZEX-1);
          return TensorLinearCubicBezierSurface<Vec2vfx>(L.split(u),R.split(u));
        }

        __forceinline V eval(const float u, const float v) const {
          return lerp(L,R,V(v)).eval(u);
        }

        __forceinline V eval_du(const float u, const float v) const {
          return lerp(L,R,V(v)).eval_dt(u);
        }

        __forceinline V eval_dv(const float u, const float v) const {
          return (R-L).eval(u);
        }

        __forceinline void eval(const float u, const float v, V& p, V& dpdu, V& dpdv) const
        {
          V p0, dp0du; L.eval(u,p0,dp0du);
          V p1, dp1du; R.eval(u,p1,dp1du);
          p = lerp(p0,p1,v);
          dpdu = lerp(dp0du,dp1du,v);
          dpdv = p1-p0;
        }

        __forceinline TensorLinearQuadraticBezierSurface<V> derivative_u() const {
          return TensorLinearQuadraticBezierSurface<V>(L.derivative(),R.derivative());
        }

        __forceinline CubicBezierCurve<V> derivative_v() const {
          return R-L;
        }

        __forceinline V axis_u() const {
          return (L.end()-L.begin())+(R.end()-R.begin());
        }

        __forceinline V axis_v() const {
          return (R.begin()-L.begin())+(R.end()-L.end());
        }
        
        friend std::ostream& operator<<(std::ostream& cout, const TensorLinearCubicBezierSurface& a)
        {
          return cout << "TensorLinearCubicBezierSurface" << std::endl
                      << "{" << std::endl
                      << "  L = " << a.L << ", " << std::endl
                      << "  R = " << a.R << std::endl
                      << "}";
        }
      };

    template<>
      struct TensorLinearCubicBezierSurface<Vec2fa>
    {
      CubicBezierCurve<vfloat4> LR;
      
      __forceinline TensorLinearCubicBezierSurface() {}
      
      __forceinline TensorLinearCubicBezierSurface(const TensorLinearCubicBezierSurface& curve)
        : LR(curve.LR) {}

      __forceinline TensorLinearCubicBezierSurface& operator= (const TensorLinearCubicBezierSurface& other) {
          LR = other.LR; return *this;
        }

      __forceinline TensorLinearCubicBezierSurface(const CubicBezierCurve<vfloat4>& LR)
        : LR(LR) {}
      
      __forceinline TensorLinearCubicBezierSurface(const CubicBezierCurve<Vec2fa>& L, const CubicBezierCurve<Vec2fa>& R)
        : LR(shuffle<0,1,0,1>(vfloat4(L.p0),vfloat4(R.p0)),shuffle<0,1,0,1>(vfloat4(L.p1),vfloat4(R.p1)),shuffle<0,1,0,1>(vfloat4(L.p2),vfloat4(R.p2)),shuffle<0,1,0,1>(vfloat4(L.p3),vfloat4(R.p3))) {}
      
      __forceinline CubicBezierCurve<Vec2fa> getL() const {
        return CubicBezierCurve<Vec2fa>(Vec2fa(LR.p0),Vec2fa(LR.p1),Vec2fa(LR.p2),Vec2fa(LR.p3));
      }
      
      __forceinline CubicBezierCurve<Vec2fa> getR() const {
        return CubicBezierCurve<Vec2fa>(Vec2fa(shuffle<2,3,2,3>(LR.p0)),Vec2fa(shuffle<2,3,2,3>(LR.p1)),Vec2fa(shuffle<2,3,2,3>(LR.p2)),Vec2fa(shuffle<2,3,2,3>(LR.p3)));
      }
      
      __forceinline BBox<Vec2fa> bounds() const
      {
        const BBox<vfloat4> b = LR.bounds();
        const BBox<Vec2fa> bl(Vec2fa(b.lower),Vec2fa(b.upper));
        const BBox<Vec2fa> br(Vec2fa(shuffle<2,3,2,3>(b.lower)),Vec2fa(shuffle<2,3,2,3>(b.upper)));
        return merge(bl,br);
      }
      
      __forceinline BBox1f bounds(const Vec2fa& axis) const
      {
        const CubicBezierCurve<vfloat4> LRx = LR;
        const CubicBezierCurve<vfloat4> LRy(shuffle<1,0,3,2>(LR.p0),shuffle<1,0,3,2>(LR.p1),shuffle<1,0,3,2>(LR.p2),shuffle<1,0,3,2>(LR.p3));
        const CubicBezierCurve<vfloat4> LRa = madd(shuffle<0>(vfloat4(axis)),LRx,shuffle<1>(vfloat4(axis))*LRy);
        const BBox<vfloat4> Lb = LRa.bounds();
        const BBox<vfloat4> Rb(shuffle<3>(Lb.lower),shuffle<3>(Lb.upper));
        const BBox<vfloat4> b = merge(Lb,Rb);
        return BBox1f(b.lower[0],b.upper[0]);
      }

      __forceinline TensorLinearCubicBezierSurface<float> xfm(const Vec2fa& dx) const
      {
        const CubicBezierCurve<vfloat4> LRx = LR;
        const CubicBezierCurve<vfloat4> LRy(shuffle<1,0,3,2>(LR.p0),shuffle<1,0,3,2>(LR.p1),shuffle<1,0,3,2>(LR.p2),shuffle<1,0,3,2>(LR.p3));
        const CubicBezierCurve<vfloat4> LRa = madd(shuffle<0>(vfloat4(dx)),LRx,shuffle<1>(vfloat4(dx))*LRy);
        return TensorLinearCubicBezierSurface<float>(CubicBezierCurve<float>(LRa.p0[0],LRa.p1[0],LRa.p2[0],LRa.p3[0]),
                                                     CubicBezierCurve<float>(LRa.p0[2],LRa.p1[2],LRa.p2[2],LRa.p3[2]));
      }
      
      __forceinline TensorLinearCubicBezierSurface<float> xfm(const Vec2fa& dx, const Vec2fa& p) const
      {
        const vfloat4 pxyxy = shuffle<0,1,0,1>(vfloat4(p));
        const CubicBezierCurve<vfloat4> LRx = LR-pxyxy;
        const CubicBezierCurve<vfloat4> LRy(shuffle<1,0,3,2>(LR.p0),shuffle<1,0,3,2>(LR.p1),shuffle<1,0,3,2>(LR.p2),shuffle<1,0,3,2>(LR.p3));
        const CubicBezierCurve<vfloat4> LRa = madd(shuffle<0>(vfloat4(dx)),LRx,shuffle<1>(vfloat4(dx))*LRy);
        return TensorLinearCubicBezierSurface<float>(CubicBezierCurve<float>(LRa.p0[0],LRa.p1[0],LRa.p2[0],LRa.p3[0]),
                                                     CubicBezierCurve<float>(LRa.p0[2],LRa.p1[2],LRa.p2[2],LRa.p3[2]));
      }

      __forceinline TensorLinearCubicBezierSurface clip_u(const Interval1f& u) const {
        return TensorLinearCubicBezierSurface(LR.clip(u));
      }
      
      __forceinline TensorLinearCubicBezierSurface clip_v(const Interval1f& v) const
      {
        const CubicBezierCurve<vfloat4> LL(shuffle<0,1,0,1>(LR.p0),shuffle<0,1,0,1>(LR.p1),shuffle<0,1,0,1>(LR.p2),shuffle<0,1,0,1>(LR.p3));
        const CubicBezierCurve<vfloat4> RR(shuffle<2,3,2,3>(LR.p0),shuffle<2,3,2,3>(LR.p1),shuffle<2,3,2,3>(LR.p2),shuffle<2,3,2,3>(LR.p3));
        return TensorLinearCubicBezierSurface(lerp(LL,RR,vfloat4(v.lower,v.lower,v.upper,v.upper)));
      }
      
      __forceinline TensorLinearCubicBezierSurface clip(const Interval1f& u, const Interval1f& v) const {
        return clip_v(v).clip_u(u);
      }
      
      __forceinline void split_u(TensorLinearCubicBezierSurface& left, TensorLinearCubicBezierSurface& right, const float u = 0.5f) const
      {
        CubicBezierCurve<vfloat4> LR0,LR1; LR.split(LR0,LR1,u);
        new (&left ) TensorLinearCubicBezierSurface(LR0);
        new (&right) TensorLinearCubicBezierSurface(LR1);
      }
      
      __forceinline TensorLinearCubicBezierSurface<Vec2vfx> vsplit_u(vboolx& valid, const BBox1f& u) const {
        valid = true; clear(valid,VSIZEX-1);
        return TensorLinearCubicBezierSurface<Vec2vfx>(getL().split(u),getR().split(u));
      }
      
      __forceinline Vec2fa eval(const float u, const float v) const
      {
        const vfloat4 p = LR.eval(u);
        return Vec2fa(lerp(shuffle<0,1,0,1>(p),shuffle<2,3,2,3>(p),v));
      }
      
      __forceinline Vec2fa eval_du(const float u, const float v) const
      {
        const vfloat4 dpdu = LR.eval_dt(u);
        return Vec2fa(lerp(shuffle<0,1,0,1>(dpdu),shuffle<2,3,2,3>(dpdu),v));
      }
      
      __forceinline Vec2fa eval_dv(const float u, const float v) const
      {
        const vfloat4 p = LR.eval(u);
        return Vec2fa(shuffle<2,3,2,3>(p)-shuffle<0,1,0,1>(p));
      }
      
      __forceinline void eval(const float u, const float v, Vec2fa& p, Vec2fa& dpdu, Vec2fa& dpdv) const
      {
        vfloat4 p0, dp0du; LR.eval(u,p0,dp0du);
        p = Vec2fa(lerp(shuffle<0,1,0,1>(p0),shuffle<2,3,2,3>(p0),v));
        dpdu = Vec2fa(lerp(shuffle<0,1,0,1>(dp0du),shuffle<2,3,2,3>(dp0du),v));
        dpdv = Vec2fa(shuffle<2,3,2,3>(p0)-shuffle<0,1,0,1>(p0));
      }
      
      __forceinline TensorLinearQuadraticBezierSurface<Vec2fa> derivative_u() const {
        return TensorLinearQuadraticBezierSurface<Vec2fa>(LR.derivative());
      }
      
      __forceinline CubicBezierCurve<Vec2fa> derivative_v() const {
        return getR()-getL();
      }
      
      __forceinline Vec2fa axis_u() const
      {
        const CubicBezierCurve<Vec2fa> L = getL();
        const CubicBezierCurve<Vec2fa> R = getR();
        return (L.end()-L.begin())+(R.end()-R.begin());
      }
      
      __forceinline Vec2fa axis_v() const
      {
        const CubicBezierCurve<Vec2fa> L = getL();
        const CubicBezierCurve<Vec2fa> R = getR();
        return (R.begin()-L.begin())+(R.end()-L.end());
      }
      
      friend std::ostream& operator<<(std::ostream& cout, const TensorLinearCubicBezierSurface& a)
      {
        return cout << "TensorLinearCubicBezierSurface" << std::endl
                    << "{" << std::endl
                    << "  L = " << a.getL() << ", " << std::endl
                    << "  R = " << a.getR() << std::endl
                    << "}";
      }
    };

    template<typename V>
      __forceinline TensorLinearCubicBezierSurface<Vec2fa> TensorLinearCubicBezierSurface<V>::xfm(const V& dx, const V& dy, const V& p) const {
      return TensorLinearCubicBezierSurface<Vec2fa>(L.xfm(dx,dy,p),R.xfm(dx,dy,p));
    }
    
    typedef TensorLinearCubicBezierSurface<float> TensorLinearCubicBezierSurface1f;
    typedef TensorLinearCubicBezierSurface<Vec2fa> TensorLinearCubicBezierSurface2fa;
    typedef TensorLinearCubicBezierSurface<Vec3fa> TensorLinearCubicBezierSurface3fa;

    template<typename Epilog>
      struct TensorLinearCubicBezierSurfaceIntersector
      {
        Ray& ray;
        LinearSpace3fa space; // FIXME: could get precalculated
        TensorLinearCubicBezierSurface3fa curve3d;
        TensorLinearCubicBezierSurface2fa curve2d;
        float eps;
        const Epilog& epilog;
        bool isHit;
        CurveCounters counters;

        __forceinline TensorLinearCubicBezierSurfaceIntersector (Ray& ray, const TensorLinearCubicBezierSurface3fa& curve3d, const Epilog& epilog)
          : ray(ray), space(frame(ray.dir)), curve3d(curve3d), epilog(epilog), isHit(false) {}
        
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
            TensorLinearCubicBezierSurface1f curve_z = curve3d.xfm(space.vz,ray.org);
            const float t = curve_z.eval(u,v)/length(ray.dir);
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
          TensorLinearCubicBezierSurface2fa curve2 = curve3d.xfm(space.vx,space.vy,ray.org);
          solve_bezier_clipping(BBox1f(0.0f,1.0f),BBox1f(0.0f,1.0f),curve2);
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

        __forceinline void solve_newton_raphson_loop(BBox1f cu, BBox1f cv, Vec2fa uv, const Vec2fa& dfdu, const Vec2fa& dfdv, const LinearSpace2fa& rcp_J)
        {
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
              const TensorLinearCubicBezierSurface1f curve_z = curve3d.xfm(space.vz,ray.org);
              const float t = curve_z.eval(u,v)/length(ray.dir);
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
          curve2d = curve3d.xfm(space.vx,space.vy,ray.org);
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

    
    struct OrientedBezierCurve1Intersector1
    {
      __forceinline OrientedBezierCurve1Intersector1() {}
      
      __forceinline OrientedBezierCurve1Intersector1(const Ray& ray, const void* ptr) {}
      
      template<typename Epilog>
      __noinline bool intersect(const CurvePrecalculations1& pre, Ray& ray, const NativeCurves* geom, 
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
        //return TensorLinearCubicBezierSurfaceIntersector<Epilog>(ray,curve,epilog).solve_bezier_clipping();
        return TensorLinearCubicBezierSurfaceIntersector<Epilog>(ray,curve,epilog).solve_newton_raphson_main();
      }
    };
  }
}
