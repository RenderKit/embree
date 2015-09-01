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

#include "../globals.h"

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

    __forceinline void eval(const float t, Vertex& point, Vertex& tangent) const
    {
      const float t0 = 1.0f - t, t1 = t;

      const Vertex p00 = v0;
      const Vertex p01 = v1;
      const Vertex p02 = v2;
      const Vertex p03 = v3;

      const Vertex p10 = p00 * t0 + p01 * t1;
      const Vertex p11 = p01 * t0 + p02 * t1;
      const Vertex p12 = p02 * t0 + p03 * t1;
      const Vertex p20 = p10 * t0 + p11 * t1;
      const Vertex p21 = p11 * t0 + p12 * t1;
      const Vertex p30 = p20 * t0 + p21 * t1;

      point = p30;
      tangent = p21-p20;
    }

    friend inline std::ostream& operator<<(std::ostream& cout, const BezierCurveT& curve) {
      return cout << "{ v0 = " << curve.v0 << ", v1 = " << curve.v1 << ", v2 = " << curve.v2 << ", v3 = " << curve.v3 << ", depth = " << curve.depth << " }";
    }
  };

  struct BezierCurve3fa : public BezierCurveT<Vec3fa>
  {
    //using BezierCurveT<Vec3fa>::BezierCurveT; // FIXME: not supported by VS2010

	__forceinline BezierCurve3fa() {}
	__forceinline BezierCurve3fa(const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const float t0 = 0.0f, const float t1 = 1.0f, const int depth = 0)
      : BezierCurveT<Vec3fa>(v0,v1,v2,v3,t0,t1,depth) {}

#if defined(__SSE__)
    __forceinline sse4f eval4(const float4& c0, const float4& c1, const float4& c2, const float4& c3) const // FIXME: c0,1,2,3 should not get passed in
    {
      const sse4f p00 = sse4f(v0);
      const sse4f p01 = sse4f(v1);
      const sse4f p02 = sse4f(v2);
      const sse4f p03 = sse4f(v3);
      return c0*p00 + c1*p01 + c2*p02 + c3*p03; // FIXME: use fmadd
    }
#endif

#if defined(__AVX__)
    __forceinline avx4f eval8(const float8& c0, const float8& c1, const float8& c2, const float8& c3) const // FIXME: c0,1,2,3 should not get passed in
    {
      const avx4f p00 = avx4f(v0);
      const avx4f p01 = avx4f(v1);
      const avx4f p02 = avx4f(v2);
      const avx4f p03 = avx4f(v3);
      return c0*p00 + c1*p01 + c2*p02 + c3*p03; // FIXME: use fmadd
    }
#endif
  };
}
