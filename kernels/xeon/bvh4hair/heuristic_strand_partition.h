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
  /*! Tries to split hair into two differently aligned hair strands */
  struct StrandSplit
  {
    struct Split;
    typedef atomic_set<PrimRefBlockT<Bezier1> > BezierRefList;

  public:
    StrandSplit () {}
    
    /*! finds the two hair strands */
    static const Split find(size_t threadIndex, BezierRefList& curves);
    
  public:

    /*! stores all information to perform some split */
    struct Split
    {    
      /*! construct an invalid split by default */
      __forceinline Split()
	: sah(inf), axis0(zero), axis1(zero) {}

      /*! constructs specified split */
      __forceinline Split(const float sah, const Vec3fa& axis0, const Vec3fa& axis1)
	: sah(sah), axis0(axis0), axis1(axis1) {}

      /*! calculates standard surface area heuristic for the split */
      __forceinline float splitSAH(const float intCost) const {
	return intCost*sah;
      }

      /*! single threaded splitting into two sets */
      void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, 
		 BezierRefList& prims, 
		 BezierRefList& lprims_o, PrimInfo& linfo_o, 
		 BezierRefList& rprims_o, PrimInfo& rinfo_o) const;

      /*! multi threaded splitting into two sets */
      void split_parallel(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<Bezier1>& alloc, 
			  BezierRefList& prims, 
			  BezierRefList& lprims_o, PrimInfo& linfo_o, 
			  BezierRefList& rprims_o, PrimInfo& rinfo_o) const;

    private:
      float sah;             //!< SAH cost of the split
      Vec3fa axis0, axis1;   //!< axis the two strands are aligned into
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
  };
}
