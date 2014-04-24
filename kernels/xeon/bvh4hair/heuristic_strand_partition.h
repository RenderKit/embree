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

#include "geometry/bezier1.h"
#include "builders/primrefalloc.h"

namespace embree
{
  /*! Tries to split hair into two differently aligned hair strands */
  struct StrandSplit
  {
    typedef PrimRefBlockT<Bezier1> BezierRefBlock;
    typedef atomic_set<BezierRefBlock> BezierRefList;

  public:
    StrandSplit () {}
    
    StrandSplit (const NAABBox3fa& bounds0, const Vec3fa& axis0, const size_t num0,
		 const NAABBox3fa& bounds1, const Vec3fa& axis1, const size_t num1);
    
    /*! calculates standard surface area heuristic for the split */
    __forceinline float splitSAH(float intCost) const {
      return intCost*float(num0)*halfArea(bounds0.bounds) + intCost*float(num1)*halfArea(bounds1.bounds);
    }
    
    /*! finds the two hair strands */
    static const StrandSplit find(size_t threadIndex, BezierRefList& curves);
    
    /*! splits hair list into the two strands */
    void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& curves, BezierRefList& lcurves_o, BezierRefList& rcurves_o) const;
    
  public:
    NAABBox3fa bounds0, bounds1;  //!< bounds of the strands
    Vec3fa axis0, axis1;          //!< axis the strands are aligned into
    size_t num0, num1;            //!< number of hairs in the strands
  };
}
