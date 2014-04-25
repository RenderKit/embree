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
  struct SpatialSplit
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
      __forceinline Mapping(const PrimInfo& pinfo);
      __forceinline ssei bin(const Vec3fa& p) const;
      __forceinline float pos(const int bin, const int dim) const;
      __forceinline bool invalid(const int dim) const;
    private:
      ssef ofs,scale;
    };
    
    /*! stores all information required to perform some split */
    struct Split
    {
      __forceinline Split() 
	: sah(inf), dim(-1), pos(0.0f) {}

      __forceinline Split(float sah, int dim, float pos, const Mapping& mapping)
	: sah(sah), dim(dim), pos(pos), mapping(mapping) {}

      /*! calculates standard surface area heuristic for the split */
      __forceinline float splitSAH() const { return sah; }

      /*! splits hair list into the two strands */
      void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& curves, 
		 BezierRefList& lprims_o, PrimInfo& linfo_o, BezierRefList& rprims_o, PrimInfo& rinfo_o) const;

    public:
      float sah;
      int   dim;
      float pos;
      Mapping mapping;
    };

    /*! stores all binning information */
    struct BinInfo
    {
      BinInfo();
      void bin(BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping);
      Split best(BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping);

      BBox3fa bounds[BINS][4];
      ssei    numBegin[BINS];
      ssei    numEnd[BINS];
    };

    /*! finds the two hair strands */
    static const Split find(size_t threadIndex, BezierRefList& curves, const PrimInfo& pinfo);
  };
}
