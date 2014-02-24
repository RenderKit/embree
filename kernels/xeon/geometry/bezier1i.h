// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "common/default.h"
#include "primitive.h"

namespace embree
{
#if defined(__AVX__)
  extern avxf coeff0[4];
  extern avxf coeff1[4];
#endif

  struct Bezier1i
  {
  public:

    /*! Default constructor. */
    __forceinline Bezier1i () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1i (const Vec3fa* p, const unsigned int geomID, const unsigned int primID, const unsigned int mask)
      : p(p), geomID(geomID), primID(primID), mask(mask) {}

    /*! calculate the bounds of the triangle */
    __forceinline BBox3fa bounds() const {
      const BBox3fa b = merge(BBox3fa(p[0]),BBox3fa(p[1]),BBox3fa(p[2]),BBox3fa(p[3]));
      return enlarge(b,Vec3fa(b.upper.w));
    }

   

  public:
    const Vec3fa* p;      //!< pointer to first control point (x,y,z,r)
    unsigned int geomID;  //!< geometry ID
    unsigned int primID;  //!< primitive ID
    unsigned int mask;    //!< geometry mask
  };

  struct Bezier1iType : public PrimitiveType {
    Bezier1iType ();
    size_t blocks(size_t x) const;
    size_t size(const char* This) const;
  };

  struct SceneBezier1i : public Bezier1iType
  {
    static SceneBezier1i type;
    void pack(char* dst, atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, void* geom) const; 
    void pack(char* dst, const PrimRef* prims, size_t num, void* geom) const;
    BBox3fa update(char* prim, size_t num, void* geom) const;
  };

  struct BezierCurve3D
  {
    Vec3fa v0,v1,v2,v3;
    float t0,t1;
    int depth;

    __forceinline BezierCurve3D() {}

    __forceinline BezierCurve3D(const Vec3fa& v0, 
                                const Vec3fa& v1, 
                                const Vec3fa& v2, 
                                const Vec3fa& v3,
                                const float t0,
                                const float t1,
                                const int depth)
      : v0(v0), v1(v1), v2(v2), v3(v3), t0(t0), t1(t1), depth(depth) {}

    __forceinline const BBox3fa bounds() const {
      BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
      return enlarge(b,Vec3fa(b.upper.w));
    }

    __forceinline void subdivide(BezierCurve3D& left, BezierCurve3D& right) const
    {
      const Vec3fa p00 = v0;
      const Vec3fa p01 = v1;
      const Vec3fa p02 = v2;
      const Vec3fa p03 = v3;

      const Vec3fa p10 = (p00 + p01) * 0.5f;
      const Vec3fa p11 = (p01 + p02) * 0.5f;
      const Vec3fa p12 = (p02 + p03) * 0.5f;
      const Vec3fa p20 = (p10 + p11) * 0.5f;
      const Vec3fa p21 = (p11 + p12) * 0.5f;
      const Vec3fa p30 = (p20 + p21) * 0.5f;

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

    __forceinline void eval(const float t, Vec3fa& point, Vec3fa& tangent) const
    {
      const float t0 = 1.0f - t, t1 = t;

      const Vec3fa p00 = v0;
      const Vec3fa p01 = v1;
      const Vec3fa p02 = v2;
      const Vec3fa p03 = v3;

      const Vec3fa p10 = p00 * t0 + p01 * t1;
      const Vec3fa p11 = p01 * t0 + p02 * t1;
      const Vec3fa p12 = p02 * t0 + p03 * t1;
      const Vec3fa p20 = p10 * t0 + p11 * t1;
      const Vec3fa p21 = p11 * t0 + p12 * t1;
      const Vec3fa p30 = p20 * t0 + p21 * t1;

      point = p30;
      tangent = p21-p20;
    }

#if defined(__AVX__)
    __forceinline avx4f eval(const avxf& c0, const avxf& c1, const avxf& c2, const avxf& c3) const
    {
      const avx4f p00 = avx4f(v0);
      const avx4f p01 = avx4f(v1);
      const avx4f p02 = avx4f(v2);
      const avx4f p03 = avx4f(v3);
      return c0*p00 + c1*p01 + c2*p02 + c3*p03; // FIXME: use fmadd
    }
#endif

    friend inline std::ostream& operator<<(std::ostream& cout, const BezierCurve3D& curve) {
      return cout << "{ v0 = " << curve.v0 << ", v1 = " << curve.v1 << ", v2 = " << curve.v2 << ", v3 = " << curve.v3 << ", depth = " << curve.depth << " }";
    }
  };

}
