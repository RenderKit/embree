// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "heuristic_spatial.h"
#include "builders/primrefblock.h"

#include "builders_new/heuristic_binning_list_aligned.h"
#include "builders_new/heuristic_spatial_list.h"

namespace embree
{
  namespace isa
  {
    /*! Performs standard object binning */
    template<typename PrimRef, typename SplitPrimitive, size_t BINS = 32>
      struct HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH
      {
        typedef BinSplit<BINS> ObjectSplit;
        typedef SpatialBinSplit<16> SpatialSplit; // FIXME: also use 32 bins?
        typedef atomic_set<PrimRefBlockT<PrimRef> > Set;
        
        struct Split
        {
          __forceinline Split () {}

          __forceinline Split (const Split& other) 
          {
            spatial = other.spatial;
            sah = other.sah;
            if (spatial) spatialSplit = other.spatialSplit;
            else         objectSplit = other.objectSplit;
          }

          __forceinline Split& operator= (const Split& other) 
          {
            spatial = other.spatial;
            sah = other.sah;
            if (spatial) spatialSplit = other.spatialSplit;
            else         objectSplit = other.objectSplit;
            return *this;
          }

          __forceinline Split (const ObjectSplit& objectSplit, float sah)
              : spatial(false), sah(sah), objectSplit(objectSplit) {}

          __forceinline Split (const SpatialSplit& spatialSplit, float sah)
            : spatial(true), sah(sah), spatialSplit(spatialSplit) {}

          __forceinline float splitSAH() const { 
            return sah; 
          }

        public:
          bool spatial;
          float sah;
          union {
            ObjectSplit objectSplit;
            SpatialSplit spatialSplit;
          };
        };

        __forceinline HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH () {
        }

        /*! remember scene for later splits */
        __forceinline HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH (const SplitPrimitive& splitPrimitive) 
          : spatial_binning(splitPrimitive) {}
        
        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          const ObjectSplit  objectSplit  = object_binning.find(set,pinfo,logBlockSize);
          const SpatialSplit spatialSplit = spatial_binning.find(set,pinfo,logBlockSize);
          const float objectSplitSAH = objectSplit.splitSAH();
          const float spatialSplitSAH = spatialSplit.splitSAH();
          if (objectSplitSAH < spatialSplitSAH) return Split(objectSplit,objectSplitSAH);
          else                                  return Split(spatialSplit,spatialSplitSAH);
        }
        
        /*! splits a list of primitives */
        void split(const Split& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (split.spatial) return spatial_binning.split(split.spatialSplit,pinfo,set,left,lset,right,rset);
          else               return  object_binning.split(split.objectSplit ,pinfo,set,left,lset,right,rset);
        }

        __forceinline void deterministic_order(const Set& set) 
        {
          /* required as parallel partition destroys original primitive order */
          //std::sort(&prims[set.begin()],&prims[set.end()]);
        }

        void splitFallback(Set& prims, PrimInfo& linfo_o, Set& lprims_o, PrimInfo& rinfo_o, Set& rprims_o) {
          object_binning.splitFallback(prims,linfo_o,lprims_o,rinfo_o,rprims_o);
        }

      private:
        HeuristicListBinningSAH<PrimRef> object_binning;
        HeuristicSpatialBlockListBinningSAH<SplitPrimitive,PrimRef> spatial_binning;
      };
  }
}

