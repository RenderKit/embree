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


      /*! splits hair list into the two strands */
      void split_parallel(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<Bezier1>& alloc, BezierRefList& prims, 
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

      /*! bins an array of primitives */
      void bin (const Bezier1* prims, size_t N, const PrimInfo& pinfo, const Mapping& mapping);

      /*! bins a list of primitives */
      void bin(BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping);
      
      /*! finds the best split by scanning binning information */
      Split best(BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping);

      /*! merges in other binning information */
      void merge (const BinInfo& other);

    private:
      BBox3fa bounds[BINS][4];
      ssei    numBegin[BINS];
      ssei    numEnd[BINS];
    };

    /*! task for parallel binning */
    struct TaskBinParallel
    {
      /*! construction executes the task */
      TaskBinParallel(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const Mapping& mapping);

    private:

      /*! parallel binning */
      TASK_RUN_FUNCTION(TaskBinParallel,task_bin_parallel);
      
    private:
      BezierRefList::iterator iter; 
      PrimInfo pinfo;
      Mapping mapping;
      BinInfo binners[32];

    public:
      Split split; //!< best split
    };

    /*! task for parallel splitting */
    struct TaskSplitParallel
    {
      /*! construction executes the task */
      TaskSplitParallel(size_t threadIndex, size_t threadCount, const Split* split, PrimRefBlockAlloc<Bezier1>& alloc, 
			BezierRefList& prims, 
			BezierRefList& lprims_o, PrimInfo& linfo_o, 
			BezierRefList& rprims_o, PrimInfo& rinfo_o);

    private:

      /*! parallel split task function */
      TASK_RUN_FUNCTION(TaskSplitParallel,task_split_parallel);

      /*! input data */
    private:
      const Split* split;
      PrimRefBlockAlloc<Bezier1>& alloc;
      BezierRefList prims;
      PrimInfo linfos[32];
      PrimInfo rinfos[32];

      /*! output data */
    private:
      BezierRefList& lprims_o; 
      PrimInfo& linfo_o;
      BezierRefList& rprims_o;
      PrimInfo& rinfo_o;
    };

    /*! finds the two hair strands */
    static const Split find(size_t threadIndex, BezierRefList& curves, const PrimInfo& pinfo);
    
    static const Split find_parallel(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo);
  };
}
