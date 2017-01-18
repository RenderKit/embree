// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
  template<typename Vertex>
    struct BezierCurveT
  {
    Vertex v0,v1,v2,v3;
    float t0,t1;
    int depth;

    __forceinline BezierCurveT() {}

    __forceinline BezierCurveT(const Vertex& v0, 
                               const Vertex& v1, 
                               const Vertex& v2, 
                               const Vertex& v3,
                               const float t0 = 0.0f,
                               const float t1 = 1.0f,
                               const int depth = 0)
      : v0(v0), v1(v1), v2(v2), v3(v3), t0(t0), t1(t1), depth(depth) {}

    __forceinline const BBox3fa bounds() const {
      BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
      return enlarge(b,Vertex(b.upper.w));
    }

    __forceinline void subdivide(BezierCurveT& left, BezierCurveT& right) const
    {
      const Vertex p00 = v0;
      const Vertex p01 = v1;
      const Vertex p02 = v2;
      const Vertex p03 = v3;

      const Vertex p10 = (p00 + p01) * 0.5f;
      const Vertex p11 = (p01 + p02) * 0.5f;
      const Vertex p12 = (p02 + p03) * 0.5f;
      const Vertex p20 = (p10 + p11) * 0.5f;
      const Vertex p21 = (p11 + p12) * 0.5f;
      const Vertex p30 = (p20 + p21) * 0.5f;

      const float t01 = (t0 + t1) * 0.5f;

      left.v0 = p00;
      left.v1 = p10;
      left.v2 = p20;
      left.v3 = p30;
      left.t0 = t0;
      left.t1 = t01;
      left.depth = depth-1;
        
      right.v0 = p30;
      right.v1 = p21;
      right.v2 = p12;
      right.v3 = p03;
      right.t0 = t01;
      right.t1 = t1;
      right.depth = depth-1;
    }

    __forceinline Vertex eval(const float t) const
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
    
    __forceinline Vertex eval_du(const float t) const
    {
      const float t0 = 1.0f - t, t1 = t;
      const float B0 = -(t0*t0);
      const float B1 = madd(-2.0f,t0*t1,t0*t0);
      const float B2 = msub(+2.0f,t0*t1,t1*t1);
      const float B3 = +(t1*t1);
      return 3.0f*(madd(B0,v0,madd(B1,v1,madd(B2,v2,B3*v3))));
    }

    __forceinline Vertex eval_dudu(const float t) const
    {
      const float t0 = 1.0f - t, t1 = t;
      const float C0 = t0;
      const float C1 = madd(-2.0f,t0,t1);
      const float C2 = madd(-2.0f,t1,t0);
      const float C3 = t1;
      return 6.0f*(madd(C0,v0,madd(C1,v1,madd(C2,v2,C3*v3))));
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

    friend inline std::ostream& operator<<(std::ostream& cout, const BezierCurveT& curve) {
      return cout << "{ v0 = " << curve.v0 << ", v1 = " << curve.v1 << ", v2 = " << curve.v2 << ", v3 = " << curve.v3 << ", depth = " << curve.depth << " }";
    }
  };

  template<int N>
    struct BezierCurve4N
    {
      Vec4<vfloat<N>> v0,v1,v2,v3;

    __forceinline BezierCurve4N() {}

    __forceinline BezierCurve4N(const Vec4<vfloat<N>>& v0, 
                                const Vec4<vfloat<N>>& v1, 
                                const Vec4<vfloat<N>>& v2, 
                                const Vec4<vfloat<N>>& v3)
      : v0(v0), v1(v1), v2(v2), v3(v3) {}
    };

  struct BezierCoefficients
  {
    enum { N = 16 };
  public:
    BezierCoefficients(int shift);

    /* coefficients for function evaluation */
  public:
    float c0[N+1][N+1];
    float c1[N+1][N+1];
    float c2[N+1][N+1];
    float c3[N+1][N+1];

    /* coefficients for derivative evaluation */
  public:
    float d0[N+1][N+1];
    float d1[N+1][N+1];
    float d2[N+1][N+1];
    float d3[N+1][N+1];
  };
  extern BezierCoefficients bezier_coeff0;
  extern BezierCoefficients bezier_coeff1;

  struct BezierCurve3fa : public BezierCurveT<Vec3fa>
  {
    //using BezierCurveT<Vec3fa>::BezierCurveT; // FIXME: not supported by VS2010

    __forceinline BezierCurve3fa() {}
    __forceinline BezierCurve3fa(const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const float t0 = 0.0f, const float t1 = 1.0f, const int depth = 0)
      : BezierCurveT<Vec3fa>(v0,v1,v2,v3,t0,t1,depth) {}
    
    template<int N>
      __forceinline BezierCurve4N<N> subdivide(const float u0, const float u1) const
    {
      const vfloat<N> lu = vfloat<N>(step)*(1.0f/(N-1));
      const vfloat<N> vu0 = (vfloat<N>(one)-lu)*u0 + lu*u1;
      Vec4<vfloat<N>> P0, dP0du; evalN(vu0,P0,dP0du);
      const Vec4<vfloat<N>>  P3   = Vec4<vfloat<N>>(shift_right_1(P0.x   ),shift_right_1(P0.y   ),shift_right_1(P0.z   ),shift_right_1(P0.w)   );
      const Vec4<vfloat<N>> dP3du = Vec4<vfloat<N>>(shift_right_1(dP0du.x),shift_right_1(dP0du.y),shift_right_1(dP0du.z),shift_right_1(dP0du.w));
      const Vec4<vfloat<N>> P1 = P0 + Vec4<vfloat<N>>((u1-u0)/(3.0f*(VSIZEX-1)))*dP0du; 
      const Vec4<vfloat<N>> P2 = P3 - Vec4<vfloat<N>>((u1-u0)/(3.0f*(VSIZEX-1)))*dP3du;
      return BezierCurve4N<N>(P0,P1,P2,P3);
    }
    
    __forceinline void evalN(const vfloatx& t, Vec4vfx& p, Vec4vfx& dp) const
    {
      const Vec4vfx p00 = v0;
      const Vec4vfx p01 = v1;
      const Vec4vfx p02 = v2;
      const Vec4vfx p03 = v3;

      const Vec4vfx p10 = lerp(p00,p01,t);
      const Vec4vfx p11 = lerp(p01,p02,t);
      const Vec4vfx p12 = lerp(p02,p03,t);
      const Vec4vfx p20 = lerp(p10,p11,t);
      const Vec4vfx p21 = lerp(p11,p12,t);
      const Vec4vfx p30 = lerp(p20,p21,t);

      p = p30;
      dp = vfloatx(3.0f)*(p21-p20);
    }


    template<int M>
      __forceinline Vec4<vfloat<M>> eval0(const vbool<M>& valid, const int ofs, const int size) const
    {
      assert(size <= BezierCoefficients::N);
      assert(ofs <= size);
      return madd(vfloat<M>::loadu(&bezier_coeff0.c0[size][ofs]), Vec4<vfloat<M>>(v0),
                  madd(vfloat<M>::loadu(&bezier_coeff0.c1[size][ofs]), Vec4<vfloat<M>>(v1),
                       madd(vfloat<M>::loadu(&bezier_coeff0.c2[size][ofs]), Vec4<vfloat<M>>(v2),
                            vfloat<M>::loadu(&bezier_coeff0.c3[size][ofs]) * Vec4<vfloat<M>>(v3))));
    }
    
    template<int M>
      __forceinline Vec4<vfloat<M>> eval1(const vbool<M>& valid, const int ofs, const int size) const
    {
      assert(size <= BezierCoefficients::N);
      assert(ofs <= size);
      return madd(vfloat<M>::loadu(&bezier_coeff1.c0[size][ofs]), Vec4<vfloat<M>>(v0), 
                  madd(vfloat<M>::loadu(&bezier_coeff1.c1[size][ofs]), Vec4<vfloat<M>>(v1),
                       madd(vfloat<M>::loadu(&bezier_coeff1.c2[size][ofs]), Vec4<vfloat<M>>(v2),
                            vfloat<M>::loadu(&bezier_coeff1.c3[size][ofs]) * Vec4<vfloat<M>>(v3))));
    }

    template<int M>
      __forceinline Vec4<vfloat<M>> derivative0(const vbool<M>& valid, const int ofs, const int size) const
    {
      assert(size <= BezierCoefficients::N);
      assert(ofs <= size);
      return madd(vfloat<M>::loadu(&bezier_coeff0.d0[size][ofs]), Vec4<vfloat<M>>(v0),
                  madd(vfloat<M>::loadu(&bezier_coeff0.d1[size][ofs]), Vec4<vfloat<M>>(v1),
                       madd(vfloat<M>::loadu(&bezier_coeff0.d2[size][ofs]), Vec4<vfloat<M>>(v2),
                            vfloat<M>::loadu(&bezier_coeff0.d3[size][ofs]) * Vec4<vfloat<M>>(v3))));
    }

    template<int M>
      __forceinline Vec4<vfloat<M>> derivative1(const vbool<M>& valid, const int ofs, const int size) const
    {
      assert(size <= BezierCoefficients::N);
      assert(ofs <= size);
      return madd(vfloat<M>::loadu(&bezier_coeff1.d0[size][ofs]), Vec4<vfloat<M>>(v0),
                  madd(vfloat<M>::loadu(&bezier_coeff1.d1[size][ofs]), Vec4<vfloat<M>>(v1),
                       madd(vfloat<M>::loadu(&bezier_coeff1.d2[size][ofs]), Vec4<vfloat<M>>(v2),
                            vfloat<M>::loadu(&bezier_coeff1.d3[size][ofs]) * Vec4<vfloat<M>>(v3))));
    }

    /* calculates bounds of bezier curve geometry */
    __forceinline BBox3fa bounds() const
    {
      const int N = 7;
      const float scale = 1.0f/(3.0f*(N-1));
      Vec4vfx pl(pos_inf), pu(neg_inf);
      for (int i=0; i<=N; i+=VSIZEX)
      {
        vintx vi = vintx(i)+vintx(step);
        vboolx valid = vi <= vintx(N);
        const Vec4vfx p  = eval0(valid,i,N);
        const Vec4vfx dp = derivative0(valid,i,N);
        const Vec4vfx pm = p-Vec4vfx(scale)*select(vi!=vintx(0),dp,Vec4vfx(zero));
        const Vec4vfx pp = p+Vec4vfx(scale)*select(vi!=vintx(N),dp,Vec4vfx(zero));
        pl = select(valid,min(pl,p,pm,pp),pl); // FIXME: use masked min
        pu = select(valid,max(pu,p,pm,pp),pu); // FIXME: use masked min
      }
      const Vec3fa lower(reduce_min(pl.x),reduce_min(pl.y),reduce_min(pl.z));
      const Vec3fa upper(reduce_max(pu.x),reduce_max(pu.y),reduce_max(pu.z));
      const Vec3fa upper_r = Vec3fa(reduce_max(max(-pl.w,pu.w)));
      return enlarge(BBox3fa(lower,upper),upper_r);
    }

    /* calculates bounds of bezier curve geometry when tessellated into N line segments */
    __forceinline BBox3fa bounds(int N) const
    {
      if (likely(N == 4))
      {
        const Vec4vf4 pi = eval0(vbool4(true),0,4);
        const Vec3fa lower(reduce_min(pi.x),reduce_min(pi.y),reduce_min(pi.z));
        const Vec3fa upper(reduce_max(pi.x),reduce_max(pi.y),reduce_max(pi.z));
        const Vec3fa upper_r = Vec3fa(reduce_max(abs(pi.w)));
        return enlarge(BBox3fa(min(lower,v3),max(upper,v3)),max(upper_r,Vec3fa(v3.w)));
      } 
      else
      {
        Vec4vfx pl(pos_inf), pu(neg_inf);
        for (int i=0; i<N; i+=VSIZEX)
        {
          vboolx valid = vintx(i)+vintx(step) < vintx(N);
          const Vec4vfx pi = eval0(valid,i,N);
          
          pl.x = select(valid,min(pl.x,pi.x),pl.x); // FIXME: use masked min
          pl.y = select(valid,min(pl.y,pi.y),pl.y); 
          pl.z = select(valid,min(pl.z,pi.z),pl.z); 
          pl.w = select(valid,min(pl.w,pi.w),pl.w); 
          
          pu.x = select(valid,max(pu.x,pi.x),pu.x); // FIXME: use masked min
          pu.y = select(valid,max(pu.y,pi.y),pu.y); 
          pu.z = select(valid,max(pu.z,pi.z),pu.z); 
          pu.w = select(valid,max(pu.w,pi.w),pu.w); 
        }
        const Vec3fa lower(reduce_min(pl.x),reduce_min(pl.y),reduce_min(pl.z));
        const Vec3fa upper(reduce_max(pu.x),reduce_max(pu.y),reduce_max(pu.z));
        const Vec3fa upper_r = Vec3fa(reduce_max(max(-pl.w,pu.w)));
        return enlarge(BBox3fa(min(lower,v3),max(upper,v3)),max(upper_r,Vec3fa(abs(v3.w))));
      }
    }
  };
}
