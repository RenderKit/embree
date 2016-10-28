// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../common/primref2.h"
#include "heuristic_binning.h"

#define MBLUR_SPLIT_OVERLAP_THRESHOLD 0.1f
#define MBLUR_SPLIT_SAH_THRESHOLD 0.99f

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<size_t BINS>
      struct HeuristicMBlur
      {
        typedef BinSplit<BINS> Split;
        typedef BinSplit<BINS> ObjectSplit;
        typedef BinSplit<BINS> TemporalSplit;
        typedef BinInfo<BINS,PrimRef2> ObjectBinner;

        struct Set {
          std::shared_ptr<std::vector<PrimRef2>> prims;
          range<size_t> object_range;
          BBox1f time_range;
        };

        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 128;

        __forceinline HeuristicMBlur ()
          : prims(nullptr) {}
        
        /*! remember prim array */
        __forceinline HeuristicMBlur (PrimRef2* prims)
          : prims(prims) {}

         /*! finds the best split */
        const Split find(Set& set, PrimInfo& pinfo, const size_t logBlockSize)
        {
          /* first try standard object split */
          SplitInfo oinfo;
          const ObjectSplit object_split = object_find(set,pinfo,logBlockSize,oinfo);
          const float object_split_sah = object_split.splitSAH();
  
          /* do temporal splits only if the child bounds overlap */
          const BBox3fa overlap = intersect(oinfo.leftBounds, oinfo.rightBounds);
          if (safeArea(overlap) >= MBLUR_SPLIT_OVERLAP_THRESHOLD*safeArea(pinfo.geomBounds))
          {              
            const TemporalSplit temporal_split = temporal_find(set, pinfo, logBlockSize);
            const float temporal_split_sah = temporal_split.splitSAH();
            
            /* take temporal split if it improved SAH */
            if (temporal_split_sah < MBLUR_SPLIT_SAH_THRESHOLD*object_split_sah)
              return temporal_split;
          }

          return object_split;
        }

        /*! finds the best split */
        const ObjectSplit object_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<BINS> mapping(pinfo);
          binner.bin(set.prims->data(),set.begin(),set.end(),mapping);
          return binner.best(mapping,logBlockSize);
        }

        /*! finds the best split */
        const TemporalSplit temporal_find(const Set& set, const PrimInfo& pinfo, const size_t logBlockSize)
        {
          /* split time range */
          const float center_time = set.time_range.center();
          const BBox1f dt0(set.time_range.lower,center_time);
          const BBox1f dt1(center_time,set.time_range.upper);
          
          /* find linear bounds for both time segments */
          LBBox3fa bounds0 = empty;
          LBBox3fa bounds1 = empty;
          for (size_t i=set.object_range.begin; i<set.object_range.end; i++) {
            const unsigned geomID = prims[i].geomID();
            const unsigned primID = prims[i].primID();
            const LBBox3fa b0 = scene->linear_bounds(scene->get(geomID)->linearBounds(primID,dt0));
            const LBBox3fa b1 = scene->linear_bounds(scene->get(geomID)->linearBounds(primID,dt1));
            bounds0.extend(b0);
            bounds1.extend(b1);
          }

          /* calculate sah */
          const size_t lCount = (set.size()+(1 << logBlockSize)-1) >> int(blocks_shift), rCount = lCount;
          const float sah = b0.expectedHalfArea()*float(lCount) + b1.expectedHalfArea()*float(rCount);
          return TemporalSplit(bestSAH,-1);
        }
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          /* valid split */
          if (unlikely(!split.valid())) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }

          /* perform temporal split */
          if (unlikely(split.data != 0))
            temporal_split(split,set,left,lset,right,rset);
          
          /* perform object split */
          else 
            object_split(split,set,left,lset,right,rset);

          assert(lset.begin() == left.begin);
          assert(lset.end()   == left.end);
          assert(lset.size()  == left.size());
          assert(rset.begin() == right.begin);
          assert(rset.end()   == right.end);
          assert(rset.size()  == right.size());
        }

        /*! array partitioning */
        __forceinline void object_split(const ObjectSplit& split, const PrimInfo& pinfo, const Set& set, PrimInfo& left, Set& lset, PrimInfo& right, Set& rset) 
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          CentGeomBBox3fa local_left(empty);
          CentGeomBBox3fa local_right(empty);
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim; 

          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );
          auto isLeft = [&] (const PrimRef2 &ref) { return any(((vint4)split.mapping.bin_unsafe(center2(ref.bounds())) < vSplitPos) & vSplitMask); };

          size_t center = 0;
          center = serial_partitioning(prims,begin,end,local_left,local_right,isLeft,
                                       [] (CentGeomBBox3fa& pinfo,const PrimRef2& ref) { pinfo.extend(ref.bounds()); });          
          
          new (&left ) PrimInfo(begin,center,local_left.geomBounds,local_left.centBounds);
          new (&right) PrimInfo(center,end,local_right.geomBounds,local_right.centBounds);
          new (&lset) Set(set.prims,begin,center);
          new (&rset) Set(set.prims,center,end);
          assert(area(left.geomBounds) >= 0.0f);
          assert(area(right.geomBounds) >= 0.0f);
        }

        /*! array partitioning */
        __forceinline void temporal_split(const TemporalSplit& split, const PrimInfo& pinfo, const Set& set, PrimInfo& linfo, Set& lset, PrimInfo& rinfo, Set& rset) 
        {
          /* split time range */
          const float center_time = set.time_range.center();
          const BBox1f time_range0(set.time_range.lower,center_time);
          const BBox1f time_range1(center_time,set.time_range.upper);
          
          /* calculate primrefs for first time range */
          linfo = empty;
          std::shared_ptr<std::vector<PrimRef2>> lprims = new std::vector<PrimRef2>(lset.object_range.size());
          for (size_t i=set.object_range.begin; i<set.object_range.end; i++) {
            const unsigned geomID = set.prims[i].geomID();
            const unsigned primID = set.prims[i].primID();
            const BBox3fa lbounds = scene->linear_bounds(scene->get(geomID)->linearBounds(primID,dt0));
            (*lprims)[i-set.object_range.begin] = PrimRef2(lbounds,geomID,primID);
            linfo.add(lbounds.interpolate(0.5f));
          }
          lset = Set(lprims,time_range0);

          /* calculate primrefs for first time range */
          rinfo = empty;
          std::shared_ptr<std::vector<PrimRef2>> rprims = new std::vector<PrimRef2>(rset.object_range.size());
          for (size_t i=set.object_range.begin; i<set.object_range.end; i++) {
            const unsigned geomID = set.prims[i].geomID();
            const unsigned primID = set.prims[i].primID();
            const BBox3fa lbounds = scene->linear_bounds(scene->get(geomID)->linearBounds(primID,dt1));
            (*rprims)[i-set.object_range.begin] = PrimRef2(rbounds,geomID,primID);
            rinfo.add(lbounds.interpolate(0.5f));
          }
          rset = Set(rprims,time_range1);
        }

        void deterministic_order(const Set& set) 
        {
          /* required as parallel partition destroys original primitive order */
          std::sort(&prims[set.begin()],&prims[set.end()]);
        }

        void deterministic_order(const PrimInfo& pinfo) 
        {
          /* required as parallel partition destroys original primitive order */
          std::sort(&prims[pinfo.begin],&prims[pinfo.end]);
        }
        
        /*! array partitioning */
        void splitFallback(const PrimInfo& pinfo, PrimInfo& left, PrimInfo& right) 
        {
          Set lset,rset;
          Set set(pinfo.begin,pinfo.end);
          splitFallback(set,left,lset,right,rset);
        }
        
        void splitFallback(const Set& set, PrimInfo& linfo, Set& lset, PrimInfo& rinfo, Set& rset)
        {
          const size_t begin = set.begin();
          const size_t end   = set.end();
          const size_t center = (begin + end)/2;
          
          CentGeomBBox3fa left; left.reset();
          for (size_t i=begin; i<center; i++)
            left.extend(prims[i].bounds());
          new (&linfo) PrimInfo(begin,center,left.geomBounds,left.centBounds);
          
          CentGeomBBox3fa right; right.reset();
          for (size_t i=center; i<end; i++)
            right.extend(prims[i].bounds());	
          new (&rinfo) PrimInfo(center,end,right.geomBounds,right.centBounds);
          
          new (&lset) range<size_t>(begin,center);
          new (&rset) range<size_t>(center,end);
        }
      };
  }
}
