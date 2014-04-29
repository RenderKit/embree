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

#include "heuristic_object_partition.h"
#include "heuristic_spatial_split.h"
#include "heuristic_strand_partition.h"
#include "heuristic_fallback.h"

namespace embree
{
  /*! Union type that selects between different splits */
  class __aligned(16) Split
  {
  private:

    enum Ty { OBJECT_SPLIT, SPATIAL_SPLIT, STRAND_SPLIT };

    enum { S0 = sizeof(ObjectPartition::Split), 
	   S1 = sizeof(SpatialSplit::Split), 
	   S2 = sizeof(StrandSplit::Split),
	   S12 = S0 > S1 ? S0 : S1,
	   SIZE = S12 > S2 ? S12 : S2 };

    typedef atomic_set<PrimRefBlockT<Bezier1> > BezierRefList; //!< list of bezier primitives

  public:

    /*! construction from object partitioning */
    __forceinline Split (ObjectPartition::Split& split) 
      : type(OBJECT_SPLIT), sah(split.splitSAH()) { new (&data) ObjectPartition::Split(split); }

    /*! construction from spatial split */
    __forceinline Split (SpatialSplit::Split& split) 
      : type(SPATIAL_SPLIT), sah(split.splitSAH()) { new (&data) SpatialSplit::Split(split); }

    /*! construction from strand split */
    __forceinline Split (StrandSplit::Split& split) 
      : type(STRAND_SPLIT), sah(split.splitSAH()) { new (&data) StrandSplit::Split(split); }

    /*! calculates surface area heuristic for performing the split */
    __forceinline float splitSAH() const { return sah; }

    /*! single threaded splitting into two sets */
    void split(size_t threadIndex, PrimRefBlockAlloc<Bezier1>& alloc, 
	       BezierRefList& prims, 
	       BezierRefList& lprims_o, PrimInfo& linfo_o, 
	       BezierRefList& rprims_o, PrimInfo& rinfo_o) const
    {
      switch (type) {
      case OBJECT_SPLIT : ((ObjectPartition::Split*)&data)->split(threadIndex,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o); break;
      case SPATIAL_SPLIT: ((SpatialSplit::   Split*)&data)->split(threadIndex,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o); break;
      case STRAND_SPLIT : ((StrandSplit::    Split*)&data)->split(threadIndex,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o); break;
      default: throw std::runtime_error("internal error");
      }
    }
    
    /*! multi threaded splitting into two sets */
    void split_parallel(size_t threadIndex, size_t threadCount, PrimRefBlockAlloc<Bezier1>& alloc, 
			BezierRefList& prims, 
			BezierRefList& lprims_o, PrimInfo& linfo_o, 
			BezierRefList& rprims_o, PrimInfo& rinfo_o) const
    {
      switch (type) {
      case OBJECT_SPLIT : ((ObjectPartition::Split*)&data)->split_parallel(threadIndex,threadCount,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o); break;
      case SPATIAL_SPLIT: ((SpatialSplit::   Split*)&data)->split_parallel(threadIndex,threadCount,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o); break;
      case STRAND_SPLIT : ((StrandSplit::    Split*)&data)->split_parallel(threadIndex,threadCount,alloc,prims,lprims_o,linfo_o,rprims_o,rinfo_o); break;
      default: throw std::runtime_error("internal error");
      }
    }

  private:
    __aligned(16) char data[SIZE]; //!< stores the different split types
    Ty type;                       //!< the type of split stored
    float sah;                     //!< the SAH of the stored split
  };
}
