// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "default.h"

namespace embree
{
  /*! A primitive reference stores the bounds of the primitive and its ID. */
  struct __aligned(32) PrimRef2
  {
    __forceinline PrimRef2 () {}

    __forceinline PrimRef2 (const LBBox3fa& lbounds_i, unsigned int numTimeSegments, unsigned int geomID, unsigned int primID) 
      : lbounds(lbounds_i)
    {
      lbounds.bounds0.lower.a = geomID;
      lbounds.bounds0.upper.a = primID;
      lbounds.bounds1.lower.a = numTimeSegments;
    }

    /*! returns bounds for binning */
    __forceinline LBBox3fa bounds() const {
      return lbounds;
    }

    /*! returns the number of time segments of this primref */
    __forceinline unsigned size() const { 
      return lbounds.bounds1.lower.a;
    }

    /*! returns center for binning */
    __forceinline Vec3fa binCenter() const {
      //return center2(lbounds.interpolate(0.0f));
      return center2(lbounds.interpolate(0.5f));
      //return center2(lbounds.interpolate(1.0f));
    }

    /*! returns bounds and centroid used for binning */
    __forceinline void binBoundsAndCenter(LBBox3fa& bounds_o, Vec3fa& center_o) const 
    {
      bounds_o = bounds();
      center_o = binCenter();
    }

    /*! returns the geometry ID */
    __forceinline unsigned geomID() const { 
      return lbounds.bounds0.lower.a;
    }

    /*! returns the primitive ID */
    __forceinline unsigned primID() const { 
      return lbounds.bounds0.upper.a;
    }

    /*! special function for operator< */
    __forceinline uint64_t ID64() const {
      return (((uint64_t)primID()) << 32) + (uint64_t)geomID();
    }
    
    /*! allows sorting the primrefs by ID */
    friend __forceinline bool operator<(const PrimRef2& p0, const PrimRef2& p1) {
      return p0.ID64() < p1.ID64();
    }

    /*! Outputs primitive reference to a stream. */
    friend __forceinline std::ostream& operator<<(std::ostream& cout, const PrimRef2& ref) {
      return cout << "{ lbounds = " << ref.lbounds << ", geomID = " << ref.geomID() << ", primID = " << ref.primID() << " }";
    }

  public:
    LBBox3fa lbounds;
  };
}
