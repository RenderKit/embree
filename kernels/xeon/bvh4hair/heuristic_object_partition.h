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
  /*! Performs standard object binning */
  struct ObjectPartition
  {
    /*! number of bins */
    static const size_t BINS = 16;

    typedef PrimRefBlockT<Bezier1> BezierRefBlock;
    typedef atomic_set<BezierRefBlock> BezierRefList;

    /*! Compute the number of blocks occupied for each dimension. */
    //__forceinline static ssei blocks(const ssei& a) { return (a+ssei(3)) >> 2; }
    __forceinline static ssei blocks(const ssei& a) { return a; }
	
    /*! Compute the number of blocks occupied in one dimension. */
    //__forceinline static size_t  blocks(size_t a) { return (a+3) >> 2; }
    __forceinline static size_t  blocks(size_t a) { return a; }

    /*! mapping into bins */
    struct Mapping
    {
    public:
      __forceinline Mapping() {}

      /*! calculates the mapping */
      __forceinline Mapping(const BBox3fa& centBounds, const LinearSpace3fa& space);

      /*! slower but safe binning */
      __forceinline ssei bin(const Vec3fa& p) const;

      /*! faster but unsafe binning */
      __forceinline ssei bin_unsafe(const Vec3fa& p) const;

      /*! returns true if the mapping is invalid in some dimension */
      __forceinline bool invalid(const int dim) const;
    public:
      ssef ofs,scale;
      LinearSpace3fa space;
    };

    /*! stores all information to perform some split */
    struct Split
    {
      /*! constructs invalid split by default */
      __forceinline Split()
	: dim(-1), pos(0), cost(inf) {}

      /*! constructs specified split */
      __forceinline Split(float cost, int dim, int pos, const Mapping& mapping)
	: cost(cost), dim(dim), pos(pos), mapping(mapping) {}

      /*! calculates surface area heuristic for performing the split */
      __forceinline float splitSAH() const {
	return cost;
      }

      /*! single threaded splitting into two sets */
      void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& curves, 
		 BezierRefList& lprims_o, PrimInfo& linfo_o, BezierRefList& rprims_o, PrimInfo& rinfo_o) const;

    public:
      float cost;      //!< SAH cost of the split
      int dim;         //!< split dimension
      int pos;         //!< bin index for splitting
      Mapping mapping; //!< mapping into bins
    };

    /*! stores all binning information */
    struct BinInfo
    {
      BinInfo();
      void  bin (BezierRefList& prims, const Mapping& mapping);
      Split best(BezierRefList& prims, const Mapping& mapping);

    private:
      BBox3fa bounds[BINS][4];
      ssei    counts[BINS];
    };

  public:
    /*! single threaded code that finds the best split */
    static Split find(size_t threadIndex, BezierRefList& curves, const LinearSpace3fa& space);
  };
}
