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

#include "bezier_curve.h"

namespace embree
{
  namespace isa
  {   
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

        template<typename SourceCurve3fa>
        __forceinline static TensorLinearCubicBezierSurface fromCenterAndNormalCurve(const SourceCurve3fa& center, const SourceCurve3fa& normal)
        {
          CubicBezierCurve3fa vcurve; convert(center,vcurve);
          CubicBezierCurve3fa ncurve; convert(normal,ncurve);
          
          const Vec3fa k0 = normalize(cross(ncurve.begin(),vcurve.begin_direction()));
          const Vec3fa k3 = normalize(cross(ncurve.end()  ,vcurve.end_direction()));
          const Vec3fa d0 = vcurve.v0.w*k0;
          const Vec3fa d1 = vcurve.v1.w*k0;
          const Vec3fa d2 = vcurve.v2.w*k3;
          const Vec3fa d3 = vcurve.v3.w*k3;
          
          CubicBezierCurve<V> L(vcurve.v0-d0,vcurve.v1-d1,vcurve.v2-d2,vcurve.v3-d3);
          CubicBezierCurve<V> R(vcurve.v0+d0,vcurve.v1+d1,vcurve.v2+d2,vcurve.v3+d3);
          return TensorLinearCubicBezierSurface(L,R);
        }

        template<typename SourceCurve3fa>
        __forceinline static TensorLinearCubicBezierSurface fromCenterCurveAndNormals(const SourceCurve3fa& center, const Vec3fa& n0, const Vec3fa& n1)
        {
          CubicBezierCurve3fa vcurve; convert(center,vcurve);
          
          const Vec3fa k0 = normalize(cross(n0,vcurve.begin_direction()));
          const Vec3fa k3 = normalize(cross(n1,vcurve.end_direction()));
          const Vec3fa d0 = vcurve.v0.w*k0;
          const Vec3fa d1 = vcurve.v1.w*k0;
          const Vec3fa d2 = vcurve.v2.w*k3;
          const Vec3fa d3 = vcurve.v3.w*k3;
          
          CubicBezierCurve<V> L(vcurve.v0-d0,vcurve.v1-d1,vcurve.v2-d2,vcurve.v3-d3);
          CubicBezierCurve<V> R(vcurve.v0+d0,vcurve.v1+d1,vcurve.v2+d2,vcurve.v3+d3);
          return TensorLinearCubicBezierSurface(L,R);
        }
        
        __forceinline BBox<V> bounds() const {
          return merge(L.bounds(),R.bounds());
        }

        __forceinline BBox3fa accurateBounds() const {
          return merge(L.accurateBounds(),R.accurateBounds());
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

        __forceinline TensorLinearCubicBezierSurface<Vec3fa> xfm(const LinearSpace3fa& space) const {
          return TensorLinearCubicBezierSurface(L.xfm(space),R.xfm(space));
        }
        
        __forceinline TensorLinearCubicBezierSurface<Vec3fa> xfm(const LinearSpace3fa& space, const Vec3fa& p) const {
          return TensorLinearCubicBezierSurface(L.xfm(space,p),R.xfm(space,p));
        }

        __forceinline TensorLinearCubicBezierSurface<Vec3fa> xfm(const LinearSpace3fa& space, const Vec3fa& p, const float s) const {
          return TensorLinearCubicBezierSurface(L.xfm(space,p,s),R.xfm(space,p,s));
        }

        __forceinline TensorLinearCubicBezierSurface clip_u(const Interval1f& u) const {
          return TensorLinearCubicBezierSurface(L.clip(u),R.clip(u));
        }
        
        __forceinline TensorLinearCubicBezierSurface clip_v(const Interval1f& v) const {
          return TensorLinearCubicBezierSurface(clerp(L,R,V(v.lower)),clerp(L,R,V(v.upper)));
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
          return clerp(L,R,V(v)).eval(u);
        }
        
        __forceinline V eval_du(const float u, const float v) const {
          return clerp(L,R,V(v)).eval_dt(u);
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
        : LR(shuffle<0,1,0,1>(vfloat4(L.v0),vfloat4(R.v0)),shuffle<0,1,0,1>(vfloat4(L.v1),vfloat4(R.v1)),shuffle<0,1,0,1>(vfloat4(L.v2),vfloat4(R.v2)),shuffle<0,1,0,1>(vfloat4(L.v3),vfloat4(R.v3))) {}
      
      __forceinline CubicBezierCurve<Vec2fa> getL() const {
        return CubicBezierCurve<Vec2fa>(Vec2fa(LR.v0),Vec2fa(LR.v1),Vec2fa(LR.v2),Vec2fa(LR.v3));
      }
      
      __forceinline CubicBezierCurve<Vec2fa> getR() const {
        return CubicBezierCurve<Vec2fa>(Vec2fa(shuffle<2,3,2,3>(LR.v0)),Vec2fa(shuffle<2,3,2,3>(LR.v1)),Vec2fa(shuffle<2,3,2,3>(LR.v2)),Vec2fa(shuffle<2,3,2,3>(LR.v3)));
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
        const CubicBezierCurve<vfloat4> LRy(shuffle<1,0,3,2>(LR.v0),shuffle<1,0,3,2>(LR.v1),shuffle<1,0,3,2>(LR.v2),shuffle<1,0,3,2>(LR.v3));
        const CubicBezierCurve<vfloat4> LRa = cmadd(shuffle<0>(vfloat4(axis)),LRx,shuffle<1>(vfloat4(axis))*LRy);
        const BBox<vfloat4> Lb = LRa.bounds();
        const BBox<vfloat4> Rb(shuffle<3>(Lb.lower),shuffle<3>(Lb.upper));
        const BBox<vfloat4> b = merge(Lb,Rb);
        return BBox1f(b.lower[0],b.upper[0]);
      }

      __forceinline TensorLinearCubicBezierSurface<float> xfm(const Vec2fa& dx) const
      {
        const CubicBezierCurve<vfloat4> LRx = LR;
        const CubicBezierCurve<vfloat4> LRy(shuffle<1,0,3,2>(LR.v0),shuffle<1,0,3,2>(LR.v1),shuffle<1,0,3,2>(LR.v2),shuffle<1,0,3,2>(LR.v3));
        const CubicBezierCurve<vfloat4> LRa = cmadd(shuffle<0>(vfloat4(dx)),LRx,shuffle<1>(vfloat4(dx))*LRy);
        return TensorLinearCubicBezierSurface<float>(CubicBezierCurve<float>(LRa.v0[0],LRa.v1[0],LRa.v2[0],LRa.v3[0]),
                                                     CubicBezierCurve<float>(LRa.v0[2],LRa.v1[2],LRa.v2[2],LRa.v3[2]));
      }
      
      __forceinline TensorLinearCubicBezierSurface<float> xfm(const Vec2fa& dx, const Vec2fa& p) const
      {
        const vfloat4 pxyxy = shuffle<0,1,0,1>(vfloat4(p));
        const CubicBezierCurve<vfloat4> LRx = LR-pxyxy;
        const CubicBezierCurve<vfloat4> LRy(shuffle<1,0,3,2>(LR.v0),shuffle<1,0,3,2>(LR.v1),shuffle<1,0,3,2>(LR.v2),shuffle<1,0,3,2>(LR.v3));
        const CubicBezierCurve<vfloat4> LRa = cmadd(shuffle<0>(vfloat4(dx)),LRx,shuffle<1>(vfloat4(dx))*LRy);
        return TensorLinearCubicBezierSurface<float>(CubicBezierCurve<float>(LRa.v0[0],LRa.v1[0],LRa.v2[0],LRa.v3[0]),
                                                     CubicBezierCurve<float>(LRa.v0[2],LRa.v1[2],LRa.v2[2],LRa.v3[2]));
      }

      __forceinline TensorLinearCubicBezierSurface clip_u(const Interval1f& u) const {
        return TensorLinearCubicBezierSurface(LR.clip(u));
      }
      
      __forceinline TensorLinearCubicBezierSurface clip_v(const Interval1f& v) const
      {
        const CubicBezierCurve<vfloat4> LL(shuffle<0,1,0,1>(LR.v0),shuffle<0,1,0,1>(LR.v1),shuffle<0,1,0,1>(LR.v2),shuffle<0,1,0,1>(LR.v3));
        const CubicBezierCurve<vfloat4> RR(shuffle<2,3,2,3>(LR.v0),shuffle<2,3,2,3>(LR.v1),shuffle<2,3,2,3>(LR.v2),shuffle<2,3,2,3>(LR.v3));
        return TensorLinearCubicBezierSurface(clerp(LL,RR,vfloat4(v.lower,v.lower,v.upper,v.upper)));
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

    typedef TensorLinearCubicBezierSurface<float> TensorLinearCubicBezierSurface1f;
    typedef TensorLinearCubicBezierSurface<Vec2fa> TensorLinearCubicBezierSurface2fa;
    typedef TensorLinearCubicBezierSurface<Vec3fa> TensorLinearCubicBezierSurface3fa;
  }
}
