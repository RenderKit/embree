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
    //__forceinline static ssei blocks(const ssei& a) { return (a+ssei(3)) >> 2; }
    __forceinline static ssei blocks(const ssei& a) { return a; }
	
    /*! Compute the number of blocks occupied in one dimension. */
    //__forceinline static size_t  blocks(size_t a) { return (a+3) >> 2; }
    __forceinline static size_t  blocks(size_t a) { return a; }

    /*! mapping into bins */
    struct Mapping
    {
      __forceinline Mapping() {}

      __forceinline Mapping(const PrimInfo& pinfo) {
	const ssef diag = (ssef) pinfo.geomBounds.size();
	scale = select(diag != 0.0f,rcp(diag) * ssef(BINS * 0.99f),ssef(0.0f));
	ofs  = (ssef) pinfo.geomBounds.lower;
      }
      
      __forceinline ssei bin(const Vec3fa& p) const 
      {
	const ssei i = floori((ssef(p)-ofs)*scale);
#if 0
	assert(i[0] >=0 && i[0] < BINS);
	assert(i[1] >=0 && i[1] < BINS);
	assert(i[2] >=0 && i[2] < BINS);
	return i;
#else
	return clamp(i,ssei(0),ssei(BINS-1));
#endif
      }

      __forceinline float pos(const int bin, const int dim) const {
	return float(bin)/scale[dim]+ofs[dim];
      }

      __forceinline bool invalid(const int dim) const {
	return scale[dim] == 0.0f;
      }

    public:
      ssef ofs,scale;
    };
    
    struct Split
    {
      /*! calculates standard surface area heuristic for the split */
      __forceinline float splitSAH(float intCost) const {
	return intCost*sah;
      }

      /*! splits hair list into the two strands */
      void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& curves, 
		 BezierRefList& lprims_o, PrimInfo& linfo_o, BezierRefList& rprims_o, PrimInfo& rinfo_o) const;

    public:
      float pos;
      int dim;
      float sah;
      Mapping mapping;
    };

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
