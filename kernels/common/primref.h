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

#include "common/default.h"

namespace embree
{
  /*! A primitive reference stores the bounds of the primitive and its ID. */
  struct __aligned(32) PrimRef 
  {
    __forceinline PrimRef () {}

#if defined(__AVX__)
    __forceinline PrimRef(const PrimRef& v) { 
      store8f((float*)this,load8f((float*)&v));
    }
    __forceinline void operator=(const PrimRef& v) { 
      store8f((float*)this,load8f((float*)&v));
    }
#endif

#if defined(__MIC__)
      __forceinline PrimRef(const PrimRef& v) { 
        compactustore16f_low(0xff,(float*)this,uload16f_low((float*)&v.lower));
      }
    
    __forceinline void operator=(const PrimRef& v) { 
      compactustore16f_low(0xff,(float*)this,uload16f_low((float*)&v.lower));
    }
#endif

    __forceinline PrimRef (const BBox3fa& bounds, unsigned geomID, unsigned primID) {
      lower = bounds.lower; lower.a = geomID;
      upper = bounds.upper; upper.a = primID;
    }

    __forceinline PrimRef (const BBox3fa& bounds, size_t id) {
#if defined(__X86_64__)
      lower = bounds.lower; lower.u = id & 0xFFFFFFFF;
      upper = bounds.upper; upper.u = (id >> 32) & 0xFFFFFFFF;
#else
      lower = bounds.lower; lower.u = id;
      upper = bounds.upper; upper.u = 0;
#endif
    }

    /*! calculates twice the center of the primitive */
    __forceinline const Vec3fa center2() const {
      return lower+upper;
    }
    
    /*! return the bounding box of the primitive */
    __forceinline const BBox3fa bounds() const {
      return BBox3fa(lower,upper);
    }

#if defined(__MIC__)
    __forceinline mic2f getBounds() const { 
      return mic2f(broadcast4to16f((float*)&lower),broadcast4to16f((float*)&upper)); 
    }
#endif

    /*! returns the geometry ID */
    __forceinline unsigned geomID() const { 
      return lower.a;
    }

    /*! returns the primitive ID */
    __forceinline unsigned primID() const { 
      return upper.a;
    }

    /*! returns an size_t sized ID */
    __forceinline size_t ID() const { 
#if defined(__X86_64__)
      return size_t(lower.u) + (size_t(upper.u) << 32);
#else
      return size_t(lower.u);
#endif
    }

    /*! special function for operator< */
    __forceinline uint64 ID64() const {
      return (((uint64)primID()) << 32) + (uint64)geomID();
    }
    
    /*! allows sorting the primrefs by ID */
    friend __forceinline bool operator<(const PrimRef& p0, const PrimRef& p1) {
      return p0.ID64() < p1.ID64();
    }

  public:
    Vec3fa lower;     //!< lower bounds and geomID
    Vec3fa upper;     //!< upper bounds and primID
  };

  /*! Outputs primitive reference to a stream. */
  inline std::ostream& operator<<(std::ostream& cout, const PrimRef& ref) {
    return cout << "{ lower = " << ref.lower << ", upper = " << ref.upper << ", geomID = " << ref.geomID() << ", primID = " << ref.primID() << " }";
  }

  __forceinline void xchg(PrimRef& a, PrimRef& b)
  {
#if defined(__AVX__) || defined(__AVX2__)
    const avxf aa = load8f((float*)&a);
    const avxf bb = load8f((float*)&b);
    store8f((float*)&a,bb);
    store8f((float*)&b,aa);
#elif defined(__MIC__)
    const mic_f aa = uload16f_low((float*)&a.lower);
    const mic_f bb = uload16f_low((float*)&b.lower);
    compactustore16f_low(0xff,(float*)&b.lower,aa);
    compactustore16f_low(0xff,(float*)&a.lower,bb);
#else
    std::swap(a,b);
#endif
  }

#if 0
  __forceinline bool subset(const PrimRef& a, const PrimRef& b)
  { 
    for ( size_t i = 0 ; i < 3 ; i++ ) if ( a.lower[i] < b.lower[i] ) return false;
    for ( size_t i = 0 ; i < 3 ; i++ ) if ( a.upper[i] > b.upper[i] ) return false;
    return true; 
  }


  __forceinline float area( const PrimRef& a ) 
  { 
    const Vec3fa d = a.upper - a.lower; 
    return 2.0f*(d.x*(d.y+d.z)+d.y*d.z); 
  }
#endif

  __forceinline void splitTriangle(const PrimRef& prim, int dim, float pos, 
                                   const Vec3fa& a, const Vec3fa& b, const Vec3fa& c, PrimRef& left_o, PrimRef& right_o)
  {
    BBox3fa left = empty, right = empty;
    const Vec3fa v[3] = { a,b,c };
    
    /* clip triangle to left and right box by processing all edges */
    Vec3fa v1 = v[2];
    for (size_t i=0; i<3; i++)
    {
      Vec3fa v0 = v1; v1 = v[i];
      float v0d = v0[dim], v1d = v1[dim];
      
      if (v0d <= pos) left. extend(v0); // this point is on left side
      if (v0d >= pos) right.extend(v0); // this point is on right side
      
      if ((v0d < pos && pos < v1d) || (v1d < pos && pos < v0d)) // the edge crosses the splitting location
      {
        assert((v1d-v0d) != 0.0f);
        Vec3fa c = v0 + (pos-v0d)/(v1d-v0d)*(v1-v0);
        left.extend(c);
        right.extend(c);
      }
    }
    //assert(!left.empty());  // happens if split does not hit triangle
    //assert(!right.empty()); // happens if split does not hit triangle
    
    /* clip against current bounds */
    BBox3fa bounds = prim.bounds();
    BBox3fa cleft (max(left .lower,bounds.lower),min(left .upper,bounds.upper));
    BBox3fa cright(max(right.lower,bounds.lower),min(right.upper,bounds.upper));
    
    new (&left_o ) PrimRef(cleft, prim.geomID(), prim.primID());
    new (&right_o) PrimRef(cright,prim.geomID(), prim.primID());
  }
}
