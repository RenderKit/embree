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
#include "primrefblock.h"

#include "heuristic_binning_list_aligned.h"
#include "heuristic_spatial_list.h"

namespace embree
{
  namespace isa
  {
    template<typename ObjectSplit, typename SpatialSplit>
      struct Split2
      {
        __forceinline Split2 () {}
        
        __forceinline Split2 (const Split2& other) 
        {
          spatial = other.spatial;
          sah = other.sah;
          if (spatial) spatialSplit() = other.spatialSplit();
          else         objectSplit()  = other.objectSplit();
        }
        
        __forceinline Split2& operator= (const Split2& other) 
        {
          spatial = other.spatial;
          sah = other.sah;
          if (spatial) spatialSplit() = other.spatialSplit();
          else         objectSplit()  = other.objectSplit();
          return *this;
        }
          
          __forceinline     ObjectSplit&  objectSplit()        { return *(      ObjectSplit*)data; }
        __forceinline const ObjectSplit&  objectSplit() const  { return *(const ObjectSplit*)data; }
        
        __forceinline       SpatialSplit& spatialSplit()       { return *(      SpatialSplit*)data; }
        __forceinline const SpatialSplit& spatialSplit() const { return *(const SpatialSplit*)data; }
        
        __forceinline Split2 (const ObjectSplit& objectSplit, float sah)
          : spatial(false), sah(sah) 
        {
          new (data) ObjectSplit(objectSplit);
        }
        
        __forceinline Split2 (const SpatialSplit& spatialSplit, float sah)
          : spatial(true), sah(sah) 
        {
          new (data) SpatialSplit(spatialSplit);
        }
        
        __forceinline float splitSAH() const { 
          return sah; 
        }
        
      public:
        bool spatial;
        float sah;
        __aligned(16) char data[sizeof(ObjectSplit) > sizeof(SpatialSplit) ? sizeof(ObjectSplit) : sizeof(SpatialSplit)];
        /*union {
          ObjectSplit objectSplit;
          SpatialSplit spatialSplit;
          };*/
      };
    
    /*! Performs standard object binning */
    template<typename PrimRef, typename SplitPrimitive, size_t BINS = 32>
      struct HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH
      {
        typedef BinSplit<BINS> ObjectSplit;
        typedef SpatialBinSplit<16> SpatialSplit; // FIXME: also use 32 bins?
        typedef atomic_set<PrimRefBlockT<PrimRef> > Set;
        typedef Split2<ObjectSplit,SpatialSplit> Split;
        
        __forceinline HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH () {
        }

        /*! remember scene for later splits */
        __forceinline HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH (const SplitPrimitive& splitPrimitive) 
          : spatial_binning(splitPrimitive) {}
        
        /*! finds the best split */
        const Split find(Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          SplitInfo oinfo;
          const ObjectSplit objectSplit  = object_binning.find(set,pinfo,logBlockSize,oinfo);
          const float objectSplitSAH = objectSplit.splitSAH();

          const BBox3fa overlap = intersect(oinfo.leftBounds,oinfo.rightBounds);
          if (safeArea(overlap) < 0.2f*safeArea(pinfo.geomBounds)) 
            return Split(objectSplit,objectSplitSAH);

          const SpatialSplit spatialSplit = spatial_binning.find(set,pinfo,logBlockSize);
          const float spatialSplitSAH = spatialSplit.splitSAH();
          if (objectSplitSAH < spatialSplitSAH) return Split(objectSplit,objectSplitSAH);
          else                                  return Split(spatialSplit,spatialSplitSAH);
        }
        
        /*! splits a list of primitives */
        void split(const Split& split, const PrimInfo& pinfo, Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          if (split.spatial) return spatial_binning.split(split.spatialSplit(),pinfo,set,left,lset,right,rset);
          else               return  object_binning.split(split.objectSplit() ,pinfo,set,left,lset,right,rset);
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

