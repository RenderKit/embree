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
#include "bezier1i.h" // FIXME: remove that dependency

namespace embree
{
#if defined(__AVX__)
  extern avxf coeff0[4];
  extern avxf coeff1[4];
#endif

  struct Bezier1
  {
  public:

    /*! Default constructor. */
    __forceinline Bezier1 () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1 (const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3, const float t0, const float t1,
                           const unsigned int geomID, const unsigned int primID)
      : p0(p0), p1(p1), p2(p2), p3(p3), t0(t0), t1(t1), geomID(geomID), primID(primID) {}

    /*! returns size of t range */
    __forceinline float dt() const {
      return t1-t0;
    }
      
    /*! calculate the center of the curve */
    __forceinline const Vec3fa center() const { // FIXME: remove this function, use center2 intead
      return p0+p3;
    }

    /*! calculate the center of the curve */
    __forceinline const Vec3fa center(const AffineSpace3fa& space) const { // FIXME: remove this function, use center2 intead
      return xfmPoint(space,p0)+xfmPoint(space,p3);
    }

    /*! calculate the center of the curve */
    __forceinline const Vec3fa center2() const {
      return p0+p3;
    }

    /*! calculate the center of the curve */
    __forceinline const Vec3fa center2(const AffineSpace3fa& space) const {
      return xfmPoint(space,p0)+xfmPoint(space,p3);
    }

#if defined(__AVX__)
    /*! calculate the bounds of the curve */
    __forceinline const BBox3fa bounds() const 
    {
#if 1
      const BezierCurve3D curve2D(p0,p1,p2,p3,0.0f,1.0f,0);
      const avx4f pi = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const Vec3fa lower(reduce_min(pi.x),reduce_min(pi.y),reduce_min(pi.z));
      const Vec3fa upper(reduce_max(pi.x),reduce_max(pi.y),reduce_max(pi.z));
      const Vec3fa upper_r = reduce_max(abs(pi.w));
      return enlarge(BBox3fa(min(lower,p3),max(upper,p3)),max(upper_r,p3.w));
#else
      const BBox3fa b = merge(BBox3fa(p0),BBox3fa(p1),BBox3fa(p2),BBox3fa(p3));
      return enlarge(b,Vec3fa(b.upper.w));
#endif
    }
    
    /*! calculate bounds in specified coordinate space */
    __forceinline const BBox3fa bounds(const AffineSpace3fa& space) const 
    {
      Vec3fa b0 = xfmPoint(space,p0); b0.w = p0.w;
      Vec3fa b1 = xfmPoint(space,p1); b1.w = p1.w;
      Vec3fa b2 = xfmPoint(space,p2); b2.w = p2.w;
      Vec3fa b3 = xfmPoint(space,p3); b3.w = p3.w;
#if 1
      const BezierCurve3D curve2D(b0,b1,b2,b3,0.0f,1.0f,0);
      const avx4f pi = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const Vec3fa lower(reduce_min(pi.x),reduce_min(pi.y),reduce_min(pi.z));
      const Vec3fa upper(reduce_max(pi.x),reduce_max(pi.y),reduce_max(pi.z));
      const Vec3fa upper_r = reduce_max(abs(pi.w));
      return enlarge(BBox3fa(min(lower,b3),max(upper,b3)),max(upper_r,b3.w));
#else
      const BBox3fa b = merge(BBox3fa(b0),BBox3fa(b1),BBox3fa(b2),BBox3fa(b3));
      return enlarge(b,Vec3fa(b.upper.w));
#endif
    }
#endif
    
    /*! subdivide the bezier curve */
    __forceinline void subdivide(Bezier1& left_o, Bezier1& right_o, const float T = 0.5f) const
    {
      const Vec3fa p00 = p0;
      const Vec3fa p01 = p1;
      const Vec3fa p02 = p2;
      const Vec3fa p03 = p3;
      
      const float T0 = 1.0f - T, T1 = T;
      const Vec3fa p10 = T0*p00 + T1*p01;
      const Vec3fa p11 = T0*p01 + T1*p02;
      const Vec3fa p12 = T0*p02 + T1*p03;
      const Vec3fa p20 = T0*p10 + T1*p11;
      const Vec3fa p21 = T0*p11 + T1*p12;
      const Vec3fa p30 = T0*p20 + T1*p21;
      
      const float t01 = T0*t0 + T1*t1;
      const unsigned int geomID = this->geomID;
      const unsigned int primID = this->primID;
      
      new (&left_o ) Bezier1(p00,p10,p20,p30,t0,t01,geomID,primID);
      new (&right_o) Bezier1(p30,p21,p12,p03,t01,t1,geomID,primID);
    }
    
    /*! split the hair using splitting plane */
    bool split(const Vec3fa& plane, Bezier1& left_o, Bezier1& right_o) const
    {
      /*! test if start and end points lie on different sides of plane */
      const float p0p = dot(p0,plane)+plane.w;
      const float p3p = dot(p3,plane)+plane.w;
      if (p0p == 0.0f || p3p == 0.0f) return false;
      if (p0p < 0.0f && p3p < 0.0f) return false;
      if (p0p > 0.0f && p3p > 0.0f) return false;
      
      /*! search for the t-value that splits the curve into one part
       *  left and right of the plane */
      float u0 = 0.0f, u1 = 1.0f;
      while (u1-u0 > 0.01f) 
      //while (u1-u0 > 0.0001f) 
      {
        const float tc = 0.5f*(u0+u1);
        Bezier1 left,right; subdivide(left,right,tc);
        const float lp0p = dot(left.p0,plane)+plane.w;
        const float lp3p = dot(left.p3,plane)+plane.w;
        if (lp0p <= 0.0f && lp3p >= 0.0f) { u1 = tc; continue; }
        if (lp0p >= 0.0f && lp3p <= 0.0f) { u1 = tc; continue; }
        u0 = tc; 
      }
      
      /*! return the split curve */
      if (p0p < 0.0f) subdivide(left_o,right_o,0.5f*(u0+u1));
      else            subdivide(right_o,left_o,0.5f*(u0+u1));
      return true;
    }

    /*! split the hair using splitting plane */
    bool split(const int dim, const float pos, Bezier1& left_o, Bezier1& right_o) const
    {
      /*! test if start and end points lie on different sides of plane */
      const float p0p = p0[dim];
      const float p3p = p3[dim];
      if (p0p == pos || p3p == pos) return false;
      if (p0p < pos && p3p < pos) return false;
      if (p0p > pos && p3p > pos) return false;
      
      /*! search for the t-value that splits the curve into one part
       *  left and right of the plane */
      float u0 = 0.0f, u1 = 1.0f;
      while (u1-u0 > 0.01f) 
      //while (u1-u0 > 0.0001f) 
      {
        const float tc = 0.5f*(u0+u1);
        Bezier1 left,right; subdivide(left,right,tc);
        const float lp0p = left.p0[dim];
        const float lp3p = left.p3[dim];
        if (lp0p <= pos && lp3p >= pos) { u1 = tc; continue; }
        if (lp0p >= pos && lp3p <= pos) { u1 = tc; continue; }
        u0 = tc; 
      }
      
      /*! return the split curve */
      if (p0p < pos) subdivide(left_o,right_o,0.5f*(u0+u1));
      else            subdivide(right_o,left_o,0.5f*(u0+u1));
      return true;
    }

    friend std::ostream& operator<<(std::ostream& cout, const Bezier1& b) {
      return std::cout << "Bezier1 { " << std::endl << 
        " p0 = " << b.p0 << ", " << std::endl <<
        " p1 = " << b.p1 << ", " << std::endl <<
        " p2 = " << b.p2 << ", " << std::endl <<
        " p3 = " << b.p3 << ",  " << std::endl <<
        " t0 = " << b.t0 << ",  t1 = " << b.t1 << ", " << std::endl <<
        " geomID = " << b.geomID << ", primID = " << b.primID << std::endl << 
      "}";
    }
    
  public:
    Vec3fa p0;            //!< 1st control point (x,y,z,r)
    Vec3fa p1;            //!< 2nd control point (x,y,z,r)
    Vec3fa p2;            //!< 3rd control point (x,y,z,r)
    Vec3fa p3;            //!< 4th control point (x,y,z,r)
    float t0,t1;          //!< t range of this sub-curve
    unsigned geomID;      //!< geometry ID
    unsigned primID;      //!< primitive ID
  };

  struct Bezier1Type : public PrimitiveType 
  {
    static Bezier1Type type;
    Bezier1Type ();
    size_t blocks(size_t x) const;
    size_t size(const char* This) const;
  };
}
