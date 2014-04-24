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
#include "heuristic_fallback.h"

namespace embree
{
  /*! Performs spatial split in geometry center */
  struct SpatialSplit
  {
    /*! number of bins */
    static const size_t BINS = 16;

    typedef PrimRefBlockT<Bezier1> BezierRefBlock;
    typedef atomic_set<BezierRefBlock> BezierRefList;

    /*! Compute the number of blocks occupied for each dimension. */
    __forceinline static ssei blocks(const ssei& a) { return (a+ssei(3)) >> 2; }
	
    /*! Compute the number of blocks occupied in one dimension. */
    __forceinline static size_t  blocks(size_t a) { return (a+3) >> 2; }
    
  public:
    
    __forceinline SpatialSplit () {}
    
    /*! calculates standard surface area heuristic for the split */
    __forceinline float splitSAH(float intCost) const {
      return intCost*float(num0)*halfArea(bounds0.bounds) + intCost*float(num1)*halfArea(bounds1.bounds);
    }
    
    /*! finds the two hair strands */
    static const SpatialSplit find(size_t threadIndex, size_t depth, const PrimInfo& pinfo, BezierRefList& curves, const LinearSpace3fa& space);
    
    /*! splits hair list into the two strands */
    void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& curves, BezierRefList& lprims_o, PrimInfo& linfo_o, BezierRefList& rprims_o, PrimInfo& rinfo_o) const;
    
  public:
    LinearSpace3fa space;
    NAABBox3fa bounds0, bounds1;
    float pos;
    int dim;
    float cost;
    size_t numReplications;
    size_t num0, num1;            //!< number of hairs in the strands
    ssef ofs,scale;
  };
}
