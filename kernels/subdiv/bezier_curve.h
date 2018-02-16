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

#include "../common/default.h"

namespace embree
{
  class BezierBasis
  {
  public:

    template<typename T>
      static __forceinline Vec4<T> eval(const T& u) 
    {
      const T t1 = u;
      const T t0 = 1.0f-t1;
      const T B0 = t0 * t0 * t0;
      const T B1 = 3.0f * t1 * (t0 * t0);
      const T B2 = 3.0f * (t1 * t1) * t0;
      const T B3 = t1 * t1 * t1;
      return Vec4<T>(B0,B1,B2,B3);
    }
    
    template<typename T>
      static __forceinline Vec4<T>  derivative(const T& u)
    {
      const T t1 = u;
      const T t0 = 1.0f-t1;
      const T B0 = -(t0*t0);
      const T B1 = madd(-2.0f,t0*t1,t0*t0);
      const T B2 = msub(+2.0f,t0*t1,t1*t1);
      const T B3 = +(t1*t1);
      return T(3.0f)*Vec4<T>(B0,B1,B2,B3);
    }

    template<typename T>
      static __forceinline Vec4<T>  derivative2(const T& u)
    {
      const T t1 = u;
      const T t0 = 1.0f-t1;
      const T B0 = t0;
      const T B1 = madd(-2.0f,t0,t1);
      const T B2 = madd(-2.0f,t1,t0);
      const T B3 = t1;
      return T(6.0f)*Vec4<T>(B0,B1,B2,B3);
    }
  };
  
  struct PrecomputedBezierBasis
  {
    enum { N = 16 };
  public:
    PrecomputedBezierBasis() {}
    PrecomputedBezierBasis(int shift);

    /* basis for bezier evaluation */
  public:
    float c0[N+1][N+1];
    float c1[N+1][N+1];
    float c2[N+1][N+1];
    float c3[N+1][N+1];
    
    /* basis for bezier derivative evaluation */
  public:
    float d0[N+1][N+1];
    float d1[N+1][N+1];
    float d2[N+1][N+1];
    float d3[N+1][N+1];
  };
  extern PrecomputedBezierBasis bezier_basis0;
  extern PrecomputedBezierBasis bezier_basis1;

  
  template<typename V>
    struct LinearBezierCurve
    {
      V v0,v1;
      
      __forceinline LinearBezierCurve () {}
      
      __forceinline LinearBezierCurve (const LinearBezierCurve& other)
        : v0(other.v0), v1(other.v1) {}
      
      __forceinline LinearBezierCurve& operator= (const LinearBezierCurve& other) {
        v0 = other.v0; v1 = other.v1; return *this;
      }
        
        __forceinline LinearBezierCurve (const V& v0, const V& v1)
          : v0(v0), v1(v1) {}
      
      __forceinline V begin() const { return v0; }
      __forceinline V end  () const { return v1; }
      
      bool hasRoot() const;
      
      friend std::ostream& operator<<(std::ostream& cout, const LinearBezierCurve& a) {
        return cout << "LinearBezierCurve (" << a.v0 << ", " << a.v1 << ")";
      }
    };
  
  template<> __forceinline bool LinearBezierCurve<Interval1f>::hasRoot() const {
    return numRoots(v0,v1);
  }
  
  template<typename V>
    struct QuadraticBezierCurve
    {
      V v0,v1,v2;
      
      __forceinline QuadraticBezierCurve () {}
      
      __forceinline QuadraticBezierCurve (const QuadraticBezierCurve& other)
        : v0(other.v0), v1(other.v1), v2(other.v2) {}
      
      __forceinline QuadraticBezierCurve& operator= (const QuadraticBezierCurve& other) {
        v0 = other.v0; v1 = other.v1; v2 = other.v2; return *this;
      }
        
        __forceinline QuadraticBezierCurve (const V& v0, const V& v1, const V& v2)
          : v0(v0), v1(v1), v2(v2) {}
      
      __forceinline V begin() const { return v0; }
      __forceinline V end  () const { return v2; }
      
      __forceinline V interval() const {
        return merge(v0,v1,v2);
      }
      
      __forceinline BBox<V> bounds() const {
        return merge(BBox<V>(v0),BBox<V>(v1),BBox<V>(v2));
      }
      
      friend std::ostream& operator<<(std::ostream& cout, const QuadraticBezierCurve& a) {
        return cout << "QuadraticBezierCurve ( (" << a.u.lower << ", " << a.u.upper << "), " << a.v0 << ", " << a.v1 << ", " << a.v2 << ")";
      }
    };
  
  
  typedef QuadraticBezierCurve<float> QuadraticBezierCurve1f;
  typedef QuadraticBezierCurve<Vec2fa> QuadraticBezierCurve2fa;
  typedef QuadraticBezierCurve<Vec3fa> QuadraticBezierCurve3fa;

  template<typename Vertex>
    struct CubicBezierCurve
    {
      Vertex v0,v1,v2,v3;
      
      __forceinline CubicBezierCurve() {}

      template<typename T1>
      __forceinline CubicBezierCurve (const CubicBezierCurve<T1>& other)
      : v0(other.v0), v1(other.v1), v2(other.v2), v3(other.v3) {}
      
      __forceinline CubicBezierCurve& operator= (const CubicBezierCurve& other) {
        v0 = other.v0; v1 = other.v1; v2 = other.v2; v3 = other.v3; return *this;
      }
      
      __forceinline CubicBezierCurve(const Vertex& v0, const Vertex& v1, const Vertex& v2, const Vertex& v3)
        : v0(v0), v1(v1), v2(v2), v3(v3) {}

      __forceinline Vertex begin() const {
        return v0;
      }

      __forceinline Vertex end() const {
        return v3;
      }

       __forceinline CubicBezierCurve<float> xfm(const Vertex& dx) const {
        return CubicBezierCurve<float>(dot(v0,dx),dot(v1,dx),dot(v2,dx),dot(v3,dx));
      }
      
      __forceinline CubicBezierCurve<vfloatx> vxfm(const Vertex& dx) const {
        return CubicBezierCurve<vfloatx>(dot(v0,dx),dot(v1,dx),dot(v2,dx),dot(v3,dx));
      }
      
      __forceinline CubicBezierCurve<float> xfm(const Vertex& dx, const Vertex& p) const {
        return CubicBezierCurve<float>(dot(v0-p,dx),dot(v1-p,dx),dot(v2-p,dx),dot(v3-p,dx));
      }
      
      __forceinline CubicBezierCurve<Vec2fa> xfm(const Vertex& dx, const Vertex& dy, const Vertex& p) const;
      
      __forceinline CubicBezierCurve<Vec3fa> xfm(const LinearSpace3fa& space, const Vec3fa& p) const
      {
        const Vec3fa q0 = xfmVector(space,v0-p);
        const Vec3fa q1 = xfmVector(space,v1-p);
        const Vec3fa q2 = xfmVector(space,v2-p);
        const Vec3fa q3 = xfmVector(space,v3-p);
        return CubicBezierCurve<Vec3fa>(q0,q1,q2,q3);
      }
      
      __forceinline int maxRoots() const;
      
      __forceinline BBox<Vertex> bounds() const {
        return merge(BBox<Vertex>(v0),BBox<Vertex>(v1),BBox<Vertex>(v2),BBox<Vertex>(v3));
      }
      
      __forceinline friend CubicBezierCurve operator +( const CubicBezierCurve& a, const CubicBezierCurve& b ) {
        return CubicBezierCurve(a.v0+b.v0,a.v1+b.v1,a.v2+b.v2,a.v3+b.v3);
      }
      
      __forceinline friend CubicBezierCurve operator -( const CubicBezierCurve& a, const CubicBezierCurve& b ) {
        return CubicBezierCurve(a.v0-b.v0,a.v1-b.v1,a.v2-b.v2,a.v3-b.v3);
      }
      
      __forceinline friend CubicBezierCurve operator -( const CubicBezierCurve& a, const Vertex& b ) {
        return CubicBezierCurve(a.v0-b,a.v1-b,a.v2-b,a.v3-b);
      }
      
      __forceinline friend CubicBezierCurve operator *( const Vertex& a, const CubicBezierCurve& b ) {
        return CubicBezierCurve(a*b.v0,a*b.v1,a*b.v2,a*b.v3);
      }
      
      __forceinline friend CubicBezierCurve cmadd( const Vertex& a, const CubicBezierCurve& b,  const CubicBezierCurve& c) {
        return CubicBezierCurve(madd(a,b.v0,c.v0),madd(a,b.v1,c.v1),madd(a,b.v2,c.v2),madd(a,b.v3,c.v3));
      }
      
      __forceinline friend CubicBezierCurve clerp ( const CubicBezierCurve& a, const CubicBezierCurve& b, const Vertex& t ) {
        return cmadd((Vertex(1.0f)-t),a,t*b);
      }
      
      __forceinline friend CubicBezierCurve merge ( const CubicBezierCurve& a, const CubicBezierCurve& b ) {
        return CubicBezierCurve(merge(a.v0,b.v0),merge(a.v1,b.v1),merge(a.v2,b.v2),merge(a.v3,b.v3));
      }
      
      __forceinline void split(CubicBezierCurve& left, CubicBezierCurve& right, const float t = 0.5f) const
      {
        const Vertex p00 = v0;
        const Vertex p01 = v1;
        const Vertex p02 = v2;
        const Vertex p03 = v3;
        
        const Vertex p10 = lerp(p00,p01,t);
        const Vertex p11 = lerp(p01,p02,t);
        const Vertex p12 = lerp(p02,p03,t);
        const Vertex p20 = lerp(p10,p11,t);
        const Vertex p21 = lerp(p11,p12,t);
        const Vertex p30 = lerp(p20,p21,t);
        
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
      
      __forceinline void eval(float t, Vertex& p, Vertex& dp) const
      {
        const Vertex p00 = v0;
        const Vertex p01 = v1;
        const Vertex p02 = v2;
        const Vertex p03 = v3;
        
        const Vertex p10 = lerp(p00,p01,t);
        const Vertex p11 = lerp(p01,p02,t);
        const Vertex p12 = lerp(p02,p03,t);
        const Vertex p20 = lerp(p10,p11,t);
        const Vertex p21 = lerp(p11,p12,t);
        const Vertex p30 = lerp(p20,p21,t);
        
        p = p30;
        dp = Vertex(3.0f)*(p21-p20);
      }

#if 0
      __forceinline Vertex eval(float t) const
      {
        const Vertex p00 = v0;
        const Vertex p01 = v1;
        const Vertex p02 = v2;
        const Vertex p03 = v3;
        
        const Vertex p10 = lerp(p00,p01,t);
        const Vertex p11 = lerp(p01,p02,t);
        const Vertex p12 = lerp(p02,p03,t);
        const Vertex p20 = lerp(p10,p11,t);
        const Vertex p21 = lerp(p11,p12,t);
        const Vertex p30 = lerp(p20,p21,t);
        
        return p30;
      }
#else
      __forceinline Vertex eval(const float t) const 
      {
        const Vec4<float> b = BezierBasis::eval(t);
        return madd(b.x,v0,madd(b.y,v1,madd(b.z,v2,b.w*v3)));
      }
#endif
      
      __forceinline Vertex eval_dt(float t) const
      {
        const Vertex p00 = v1-v0;
        const Vertex p01 = v2-v1;
        const Vertex p02 = v3-v2;
        const Vertex p10 = lerp(p00,p01,t);
        const Vertex p11 = lerp(p01,p02,t);
        const Vertex p20 = lerp(p10,p11,t);
        return Vertex(3.0f)*p20;
      }

      __forceinline Vertex eval_du(const float t) const
      {
        const Vec4<float> b = BezierBasis::derivative(t);
        return madd(b.x,v0,madd(b.y,v1,madd(b.z,v2,b.w*v3)));
      }

      __forceinline Vertex eval_dudu(const float t) const 
      {
        const Vec4<float> b = BezierBasis::derivative2(t);
        return madd(b.x,v0,madd(b.y,v1,madd(b.z,v2,b.w*v3)));
      }
      
      __forceinline void evalN(const vfloatx& t, Vec2vfx& p, Vec2vfx& dp) const
      {
        const Vec2vfx p00 = v0;
        const Vec2vfx p01 = v1;
        const Vec2vfx p02 = v2;
        const Vec2vfx p03 = v3;
        
        const Vec2vfx p10 = lerp(p00,p01,t);
        const Vec2vfx p11 = lerp(p01,p02,t);
        const Vec2vfx p12 = lerp(p02,p03,t);
        
        const Vec2vfx p20 = lerp(p10,p11,t);
        const Vec2vfx p21 = lerp(p11,p12,t);
        
        const Vec2vfx p30 = lerp(p20,p21,t);
        
        p = p30;
        dp = vfloatx(3.0f)*(p21-p20);
      }

      __forceinline void eval(const float t, Vertex& p, Vertex& dp, Vertex& ddp) const
      {
        const Vertex p00 = v0;
        const Vertex p01 = v1;
        const Vertex p02 = v2;
        const Vertex p03 = v3;
        const Vertex p10 = lerp(p00,p01,t);
        const Vertex p11 = lerp(p01,p02,t);
        const Vertex p12 = lerp(p02,p03,t);
        const Vertex p20 = lerp(p10,p11,t);
        const Vertex p21 = lerp(p11,p12,t);
        const Vertex p30 = lerp(p20,p21,t);
        p = p30;
        dp = 3.0f*(p21-p20);
        ddp = eval_dudu(t);
      }
      
      __forceinline CubicBezierCurve clip(const Interval1f& u1) const
      {
        Vertex f0,df0; eval(u1.lower,f0,df0);
        Vertex f1,df1; eval(u1.upper,f1,df1);
        float s = u1.upper-u1.lower;
        return CubicBezierCurve(f0,f0+s*(1.0f/3.0f)*df0,f1-s*(1.0f/3.0f)*df1,f1);
      }
      
      __forceinline QuadraticBezierCurve<Vertex> derivative() const
      {
        const Vertex q0 = 3.0f*(v1-v0);
        const Vertex q1 = 3.0f*(v2-v1);
        const Vertex q2 = 3.0f*(v3-v2);
        return QuadraticBezierCurve<Vertex>(q0,q1,q2);
      }
      
      __forceinline BBox<Vertex> derivative_bounds(const Interval1f& u1) const
      {
        Vertex f0,df0; eval(u1.lower,f0,df0);
        Vertex f3,df3; eval(u1.upper,f3,df3);
        const float s = u1.upper-u1.lower;
        const Vertex f1 = f0+s*(1.0f/3.0f)*df0;
        const Vertex f2 = f3-s*(1.0f/3.0f)*df3;
        const Vertex q0 = s*df0;
        const Vertex q1 = 3.0f*(f2-f1);
        const Vertex q2 = s*df3;
        return merge(BBox<Vertex>(q0),BBox<Vertex>(q1),BBox<Vertex>(q2));
      }
      
      template<int M>
      __forceinline Vec4vf<M> veval(const vfloat<M>& t) const 
      {
        const Vec4vf<M> b = BezierBasis::eval(t);
        return madd(b.x, Vec4vf<M>(v0), madd(b.y, Vec4vf<M>(v1), madd(b.z, Vec4vf<M>(v2), b.w * Vec4vf<M>(v3))));
      }

      template<int M>
      __forceinline Vec4vf<M> veval_du(const vfloat<M>& t) const 
      {
        const Vec4vf<M> b = BezierBasis::derivative(t);
        return madd(b.x, Vec4vf<M>(v0), madd(b.y, Vec4vf<M>(v1), madd(b.z, Vec4vf<M>(v2), b.w * Vec4vf<M>(v3))));
      }

      template<int M>
      __forceinline Vec4vf<M> veval_dudu(const vfloat<M>& t) const 
      {
        const Vec4vf<M> b = BezierBasis::derivative2(t);
        return madd(b.x, Vec4vf<M>(v0), madd(b.y, Vec4vf<M>(v1), madd(b.z, Vec4vf<M>(v2), b.w * Vec4vf<M>(v3))));
      }
      
      template<int M>
      __forceinline void veval(const vfloat<M>& t, Vec4vf<M>& p, Vec4vf<M>& dp) const
      {
        const Vec4vf<M> p00 = v0;
        const Vec4vf<M> p01 = v1;
        const Vec4vf<M> p02 = v2;
        const Vec4vf<M> p03 = v3;
        
        const Vec4vf<M> p10 = lerp(p00,p01,t);
        const Vec4vf<M> p11 = lerp(p01,p02,t);
        const Vec4vf<M> p12 = lerp(p02,p03,t);
        const Vec4vf<M> p20 = lerp(p10,p11,t);
        const Vec4vf<M> p21 = lerp(p11,p12,t);
        const Vec4vf<M> p30 = lerp(p20,p21,t);
        
        p = p30;
        dp = vfloat<M>(3.0f)*(p21-p20);
      }
      
      template<int M>
      __forceinline Vec4vf<M> eval0(const int ofs, const int size) const
      {
        assert(size <= PrecomputedBezierBasis::N);
        assert(ofs <= size);
        return madd(vfloat<M>::loadu(&bezier_basis0.c0[size][ofs]), Vec4vf<M>(v0),
                    madd(vfloat<M>::loadu(&bezier_basis0.c1[size][ofs]), Vec4vf<M>(v1),
                         madd(vfloat<M>::loadu(&bezier_basis0.c2[size][ofs]), Vec4vf<M>(v2),
                              vfloat<M>::loadu(&bezier_basis0.c3[size][ofs]) * Vec4vf<M>(v3))));
      }
      
      template<int M>
      __forceinline Vec4vf<M> eval1(const int ofs, const int size) const
      {
        assert(size <= PrecomputedBezierBasis::N);
        assert(ofs <= size);
        return madd(vfloat<M>::loadu(&bezier_basis1.c0[size][ofs]), Vec4vf<M>(v0), 
                    madd(vfloat<M>::loadu(&bezier_basis1.c1[size][ofs]), Vec4vf<M>(v1),
                         madd(vfloat<M>::loadu(&bezier_basis1.c2[size][ofs]), Vec4vf<M>(v2),
                              vfloat<M>::loadu(&bezier_basis1.c3[size][ofs]) * Vec4vf<M>(v3))));
      }
      
      template<int M>
      __forceinline Vec4vf<M> derivative0(const int ofs, const int size) const
      {
        assert(size <= PrecomputedBezierBasis::N);
        assert(ofs <= size);
        return madd(vfloat<M>::loadu(&bezier_basis0.d0[size][ofs]), Vec4vf<M>(v0),
                    madd(vfloat<M>::loadu(&bezier_basis0.d1[size][ofs]), Vec4vf<M>(v1),
                         madd(vfloat<M>::loadu(&bezier_basis0.d2[size][ofs]), Vec4vf<M>(v2),
                              vfloat<M>::loadu(&bezier_basis0.d3[size][ofs]) * Vec4vf<M>(v3))));
      }
      
      template<int M>
      __forceinline Vec4vf<M> derivative1(const int ofs, const int size) const
      {
        assert(size <= PrecomputedBezierBasis::N);
        assert(ofs <= size);
        return madd(vfloat<M>::loadu(&bezier_basis1.d0[size][ofs]), Vec4vf<M>(v0),
                    madd(vfloat<M>::loadu(&bezier_basis1.d1[size][ofs]), Vec4vf<M>(v1),
                         madd(vfloat<M>::loadu(&bezier_basis1.d2[size][ofs]), Vec4vf<M>(v2),
                              vfloat<M>::loadu(&bezier_basis1.d3[size][ofs]) * Vec4vf<M>(v3))));
      }
      
      /* calculates bounds of bezier curve geometry */
      __forceinline BBox3fa accurateBounds() const
      {
        const int N = 7;
        const float scale = 1.0f/(3.0f*(N-1));
        Vec4vfx pl(pos_inf), pu(neg_inf);
        for (int i=0; i<=N; i+=VSIZEX)
        {
          vintx vi = vintx(i)+vintx(step);
          vboolx valid = vi <= vintx(N);
          const Vec4vfx p  = eval0<VSIZEX>(i,N);
          const Vec4vfx dp = derivative0<VSIZEX>(i,N);
          const Vec4vfx pm = p-Vec4vfx(scale)*select(vi!=vintx(0),dp,Vec4vfx(zero));
          const Vec4vfx pp = p+Vec4vfx(scale)*select(vi!=vintx(N),dp,Vec4vfx(zero));
          pl = select(valid,min(pl,p,pm,pp),pl); // FIXME: use masked min
          pu = select(valid,max(pu,p,pm,pp),pu); // FIXME: use masked min
        }
        const Vec3fa lower(reduce_min(pl.x),reduce_min(pl.y),reduce_min(pl.z));
        const Vec3fa upper(reduce_max(pu.x),reduce_max(pu.y),reduce_max(pu.z));
        const float r_min = reduce_min(pl.w);
        const float r_max = reduce_max(pu.w);
        const Vec3fa upper_r = Vec3fa(max(abs(r_min),abs(r_max)));
        return enlarge(BBox3fa(lower,upper),upper_r);
      }
      
      /* calculates bounds when tessellated into N line segments */
      __forceinline BBox3fa tessellatedBounds(int N) const
      {
        if (likely(N == 4))
        {
          const Vec4vf4 pi = eval0<4>(0,4);
          const Vec3fa lower(reduce_min(pi.x),reduce_min(pi.y),reduce_min(pi.z));
          const Vec3fa upper(reduce_max(pi.x),reduce_max(pi.y),reduce_max(pi.z));
          const Vec3fa upper_r = Vec3fa(reduce_max(abs(pi.w)));
          return enlarge(BBox3fa(min(lower,v3),max(upper,v3)),max(upper_r,Vec3fa(abs(v3.w))));
        } 
        else
        {
          Vec3vfx pl(pos_inf), pu(neg_inf); vfloatx ru(0.0f);
          for (int i=0; i<N; i+=VSIZEX)
          {
            vboolx valid = vintx(i)+vintx(step) < vintx(N);
            const Vec4vfx pi = eval0<VSIZEX>(i,N);
            
            pl.x = select(valid,min(pl.x,pi.x),pl.x); // FIXME: use masked min
            pl.y = select(valid,min(pl.y,pi.y),pl.y); 
            pl.z = select(valid,min(pl.z,pi.z),pl.z); 
            
            pu.x = select(valid,max(pu.x,pi.x),pu.x); // FIXME: use masked min
            pu.y = select(valid,max(pu.y,pi.y),pu.y); 
            pu.z = select(valid,max(pu.z,pi.z),pu.z); 
            
            ru   = select(valid,max(ru,abs(pi.w)),ru);
          }
          const Vec3fa lower(reduce_min(pl.x),reduce_min(pl.y),reduce_min(pl.z));
          const Vec3fa upper(reduce_max(pu.x),reduce_max(pu.y),reduce_max(pu.z));
          const Vec3fa upper_r(reduce_max(ru));
          return enlarge(BBox3fa(min(lower,v3),max(upper,v3)),max(upper_r,Vec3fa(abs(v3.w))));
        }
      }
      
      friend inline std::ostream& operator<<(std::ostream& cout, const CubicBezierCurve& curve) {
        return cout << "CubicBezierCurve { v0 = " << curve.v0 << ", v1 = " << curve.v1 << ", v2 = " << curve.v2 << ", v3 = " << curve.v3 << " }";
      }
    };
  
  template<typename Vertex> using BezierCurveT = CubicBezierCurve<Vertex>;
  
  typedef CubicBezierCurve<float> CubicBezierCurve1f;
  typedef CubicBezierCurve<Vec2fa> CubicBezierCurve2fa;
  typedef CubicBezierCurve<Vec3fa> CubicBezierCurve3fa;
  typedef CubicBezierCurve<Vec3fa> BezierCurve3fa;
  
  template<>
    __forceinline CubicBezierCurve<Vec2fa> CubicBezierCurve<Vec3fa>::xfm(const Vec3fa& dx, const Vec3fa& dy, const Vec3fa& p) const
  {
#if 0
    Vec3<vfloat4> p0123;
    transpose(vfloat4(v0),vfloat4(v1),vfloat4(v2),vfloat4(v3),p0123.x,p0123.y,p0123.z);
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
    const vfloat8 q0xy(dot(vfloat8(vfloat4(v0-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
    const vfloat8 q1xy(dot(vfloat8(vfloat4(v1-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
    const vfloat8 q2xy(dot(vfloat8(vfloat4(v2-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
    const vfloat8 q3xy(dot(vfloat8(vfloat4(v3-p)),vfloat8(vfloat4(dx),vfloat4(dy))));
    const Vec2fa q0(unpacklo(extract4<0>(q0xy),extract4<1>(q0xy)));
    const Vec2fa q1(unpacklo(extract4<0>(q1xy),extract4<1>(q1xy)));
    const Vec2fa q2(unpacklo(extract4<0>(q2xy),extract4<1>(q2xy)));
    const Vec2fa q3(unpacklo(extract4<0>(q3xy),extract4<1>(q3xy)));
    return CubicBezierCurve<Vec2fa>(q0,q1,q2,q3);
#else
    const Vec2fa q0(dot(v0-p,dx), dot(v0-p,dy));
    const Vec2fa q1(dot(v1-p,dx), dot(v1-p,dy));
    const Vec2fa q2(dot(v2-p,dx), dot(v2-p,dy));
    const Vec2fa q3(dot(v3-p,dx), dot(v3-p,dy));
    return CubicBezierCurve<Vec2fa>(q0,q1,q2,q3);
#endif
#endif
  }   
  
  template<> inline int CubicBezierCurve<float>::maxRoots() const
  {
    float eps = 1E-4f;
    bool neg0 = v0 <= 0.0f; bool zero0 = fabs(v0) < eps;
    bool neg1 = v1 <= 0.0f; bool zero1 = fabs(v1) < eps;
    bool neg2 = v2 <= 0.0f; bool zero2 = fabs(v2) < eps;
    bool neg3 = v3 <= 0.0f; bool zero3 = fabs(v3) < eps;
    return (neg0 != neg1 || zero0) + (neg1 != neg2 || zero1) + (neg2 != neg3 || zero2 || zero3);
  }
  
  template<> __forceinline int CubicBezierCurve<Interval1f>::maxRoots() const {
    return numRoots(v0,v1) + numRoots(v1,v2) + numRoots(v2,v3);
  }
}
