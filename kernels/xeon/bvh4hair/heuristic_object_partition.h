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
    struct Split;
    typedef atomic_set<PrimRefBlockT<Bezier1> > BezierRefList; //!< list of bezier primitives

  public:

    /*! finds the best split */
    template<bool Parallel = false>
      static const Split find(size_t threadIndex, size_t threadCount, BezierRefList& curves, const LinearSpace3fa& space);

  private:

    /*! number of bins */
    static const size_t BINS = 16;

    /*! number of tasks */
    static const size_t maxTasks = 32;

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
      ssef ofs,scale;        //!< linear function that maps to bin ID
      LinearSpace3fa space;  //!< space the binning is performed in
    };

  public:

    /*! stores all information to perform some split */
    struct Split
    {
      /*! construct an invalid split by default */
      __forceinline Split()
	: sah(inf), dim(-1), pos(0) {}

      /*! constructs specified split */
      __forceinline Split(float sah, int dim, int pos, const Mapping& mapping)
	: sah(sah), dim(dim), pos(pos), mapping(mapping) {}

      /*! calculates surface area heuristic for performing the split */
      __forceinline float splitSAH() const { return sah; }

      /*! splitting into two sets */
      template<bool Parallel = false>
      void split(size_t threadIndex, size_t threadCount, 
		 PrimRefBlockAlloc<Bezier1>& alloc, 
		 BezierRefList& prims, 
		 BezierRefList& lprims_o, PrimInfo& linfo_o, 
		 BezierRefList& rprims_o, PrimInfo& rinfo_o) const;
      
    public:
      float sah;       //!< SAH cost of the split
      int dim;         //!< split dimension
      int pos;         //!< bin index for splitting
      Mapping mapping; //!< mapping into bins
    };

  private:

    /*! stores all binning information */
    struct __aligned(64) BinInfo
    {
      BinInfo();

      /*! bins an array of primitives */
      void bin (const Bezier1* prims, size_t N, const Mapping& mapping);
  
      /*! bins a list of primitives */
      void bin (BezierRefList& prims, const Mapping& mapping);
      
      /*! merges in other binning information */
      void merge (const BinInfo& other);
      
      /*! finds the best split by scanning binning information */
      Split best(BezierRefList& prims, const Mapping& mapping);

    private:
      BBox3fa bounds[BINS][4]; //!< geometry bounds for each bin in each dimension
      ssei    counts[BINS];    //!< counts number of primitives that map into the bins
    };

    /*! task for parallel binning */
    struct TaskBinParallel
    {
      /*! construction executes the task */
      TaskBinParallel(size_t threadIndex, size_t threadCount, BezierRefList& prims, const LinearSpace3fa& space);

    private:

      /*! parallel bounding calculations */
      TASK_RUN_FUNCTION(TaskBinParallel,task_bound_parallel);

      /*! parallel binning */
      TASK_RUN_FUNCTION(TaskBinParallel,task_bin_parallel);
      
      /*! state for bounding stage */
    private:
      BezierRefList::iterator iter0; //!< iterator for bounding stage 
      LinearSpace3fa space; //!< space for bounding calculations
      BBox3fa centBounds;   //!< calculated centroid bounds
      BBox3fa geomBounds;   //!< calculated geometry bounds

      /*! state for binning stage */
    private:
      BezierRefList::iterator iter1; //!< iterator for binning stage
      Mapping mapping;
      BinInfo binners[maxTasks];

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
      PrimInfo linfos[maxTasks];
      PrimInfo rinfos[maxTasks];

      /*! output data */
    private:
      BezierRefList& lprims_o; 
      PrimInfo& linfo_o;
      BezierRefList& rprims_o;
      PrimInfo& rinfo_o;
    };
  };
}
