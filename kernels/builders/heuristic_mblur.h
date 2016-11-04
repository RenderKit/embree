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
#define MBLUR_TIME_SPLIT_THRESHOLD 1.10f

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<typename Mesh, size_t BINS>
      struct HeuristicMBlur
      {
        typedef BinSplit<BINS> Split;
        typedef BinSplit<BINS> ObjectSplit;
        typedef BinSplit<BINS> TemporalSplit;
        typedef BinInfoT<BINS,PrimRef2,LBBox3fa> ObjectBinner;

        struct Set 
        {
          __forceinline Set () {}

          __forceinline Set(const std::shared_ptr<avector<PrimRef2>>& prims, range<size_t> object_range, BBox1f time_range)
            : prims(prims), object_range(object_range), time_range(time_range) {}

          __forceinline Set(std::shared_ptr<avector<PrimRef2>>& prims, BBox1f time_range = BBox1f(0.0f,1.0f))
            : prims(prims), object_range(range<size_t>(0,prims->size())), time_range(time_range) {}

        public:
          std::shared_ptr<avector<PrimRef2>> prims;
          range<size_t> object_range;
          BBox1f time_range;
        };

        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARITION_BLOCK_SIZE = 128;

        HeuristicMBlur (Scene* scene)
          : scene(scene)
        {
            numTimeSegments = scene->getNumTimeSteps<Mesh,true>()-1;
        }

        __forceinline unsigned calculateNumOverlappingTimeSegments(unsigned geomID, BBox1f time_range)
        {
          const unsigned totalTimeSegments = scene->get(geomID)->numTimeSegments();
          const unsigned itime_lower = floor(1.0001f*time_range.lower*float(totalTimeSegments));
          const unsigned itime_upper = ceil (0.9999f*time_range.upper*float(totalTimeSegments));
          const unsigned numTimeSegments = itime_upper-itime_lower; 
          assert(numTimeSegments > 0);
          return numTimeSegments;
        }

        /*! finds the best split */
        const Split find(Set& set, PrimInfo2& pinfo, const size_t logBlockSize)
        {
          /* first try standard object split */
          //SplitInfo2 oinfo;
          const ObjectSplit object_split = object_find(set,pinfo,logBlockSize);
          const float object_split_sah = object_split.splitSAH();

          /* calculate number of timesegments */
          unsigned numTimeSegments = 0;
          for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++) {
            const PrimRef2& prim = (*set.prims)[i];
            unsigned segments = scene->get(prim.geomID())->numTimeSegments();
            numTimeSegments = max(numTimeSegments,segments);
          }
  
          /* do temporal splits only if the child bounds overlap */
          //const BBox3fa overlap = intersect(oinfo.leftBounds, oinfo.rightBounds);
          //if (safeArea(overlap) >= MBLUR_SPLIT_OVERLAP_THRESHOLD*safeArea(pinfo.geomBounds))
          if (set.time_range.size() > 1.99f/float(numTimeSegments))
          //if (set.time_range.size() > 1.01f/float(numTimeSegments))
          {
            TemporalSplit temporal_split = temporal_find(set, pinfo, logBlockSize, numTimeSegments);
            const float temporal_split_sah = temporal_split.splitSAH();

            /*PRINT(pinfo);
            PRINT(object_split_sah);
            PRINT(temporal_split_sah);*/

            //PRINT(pinfo.size());
            //if (pinfo.size() == 103872 && set.time_range.size() == 1.0f) {
            //  return temporal_split;
            //}
            
            //if (set.time_range.size() > 1.01f/float(time_segments))
            //  return temporal_split;

            /*float travCost = 1.0f;
            float intCost = 1.0f;
            float bestSAH = min(temporal_split_sah,object_split_sah);
            if (intCost*pinfo.leafSAH(logBlockSize) < travCost*expectedApproxHalfArea(pinfo.geomBounds)+intCost*bestSAH)
            {
              temporal_split.sah = float(neg_inf);
              return temporal_split;
              }*/

            /* force time split if object partitioning was not very successfull */
            /*float leafSAH = pinfo.leafSAH(logBlockSize);
            if (object_split_sah > 0.7f*leafSAH) {
              temporal_split.sah = float(neg_inf);
              return temporal_split;
              }*/

            /* force time split if linear bounds are bad approximation */
            /*if (Al > 500.0f*A0) {
              temporal_split.sah = float(neg_inf);
              return temporal_split;
              }*/

            /* take temporal split if it improved SAH */
            if (temporal_split_sah < object_split_sah)
              return temporal_split;
          }

          return object_split;
        }

        /*! finds the best split */
        const ObjectSplit object_find(const Set& set, const PrimInfo2& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<BINS> mapping(pinfo.centBounds,pinfo.size());
          binner.bin(set.prims->data(),set.object_range.begin(),set.object_range.end(),mapping);
          ObjectSplit osplit = binner.best(mapping,logBlockSize);
          osplit.sah *= pinfo.time_range.size();
          return osplit;
        }

        /*! finds the best split */
        const TemporalSplit temporal_find(const Set& set, const PrimInfo2& pinfo, const size_t logBlockSize, const unsigned numTimeSegments)
        {
          const float dt = 0.5f; 
          //const float dt = 0.125f;
          float bestSAH = inf;
          float bestPos = 0.0f;
          for (float t=dt; t<1.0f-dt/2.0f; t+=dt)
          {
            /* split time range */
            //const float center_time = set.time_range.center();
            float ct = lerp(set.time_range.lower,set.time_range.upper,t);
            //float ct = set.time_range.center();
            const float center_time = round(ct * float(numTimeSegments)) / float(numTimeSegments);
            if (center_time <= set.time_range.lower) continue;
            if (center_time >= set.time_range.upper) continue;
            const BBox1f dt0(set.time_range.lower,center_time);
            const BBox1f dt1(center_time,set.time_range.upper);
          
            /* find linear bounds for both time segments */
            size_t s0 = 0; LBBox3fa bounds0 = empty;
            size_t s1 = 0; LBBox3fa bounds1 = empty;
            for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++) 
            {
              const avector<PrimRef2>& prims = *set.prims;
              const unsigned geomID = prims[i].geomID();
              const unsigned primID = prims[i].primID();
              bounds0.extend(((Mesh*)scene->get(geomID))->linearBounds(primID,dt0));
              bounds1.extend(((Mesh*)scene->get(geomID))->linearBounds(primID,dt1));
              s0 += calculateNumOverlappingTimeSegments(geomID,dt0);
              s1 += calculateNumOverlappingTimeSegments(geomID,dt1);
            }
            
            /* calculate sah */
            const size_t lCount = (s0+(1 << logBlockSize)-1) >> int(logBlockSize);
            const size_t rCount = (s1+(1 << logBlockSize)-1) >> int(logBlockSize);
            const float sah = bounds0.expectedApproxHalfArea()*float(lCount)*dt0.size() + bounds1.expectedApproxHalfArea()*float(rCount)*dt1.size();
            if (sah < bestSAH) {
              bestSAH = sah;
              bestPos = center_time;
            }
          }
          return TemporalSplit(bestSAH*MBLUR_TIME_SPLIT_THRESHOLD,-1,numTimeSegments,bestPos);
        }
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfo2& pinfo, const Set& set, PrimInfo2& left, Set& lset, PrimInfo2& right, Set& rset) 
        {
          /* valid split */
          if (unlikely(!split.valid())) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }

          /* perform temporal split */
          if (unlikely(split.data != 0))
            temporal_split(split,pinfo,set,left,lset,right,rset);
          
          /* perform object split */
          else 
            object_split(split,pinfo,set,left,lset,right,rset);
        }

        /*! array partitioning */
        __forceinline void object_split(const ObjectSplit& split, const PrimInfo2& pinfo, const Set& set, PrimInfo2& left, Set& lset, PrimInfo2& right, Set& rset) 
        {
          const size_t begin = set.object_range.begin();
          const size_t end   = set.object_range.end();
          left = empty;
          right = empty;
          const unsigned int splitPos = split.pos;
          const unsigned int splitDim = split.dim;
          const unsigned int splitDimMask = (unsigned int)1 << splitDim; 

          const vint4 vSplitPos(splitPos);
          const vbool4 vSplitMask( (int)splitDimMask );
          auto isLeft = [&] (const PrimRef2 &ref) { return any(((vint4)split.mapping.bin_unsafe(ref) < vSplitPos) & vSplitMask); };
          auto reduction = [] (PrimInfo2& pinfo, const PrimRef2& ref) { pinfo.add_primref(ref); };

          size_t center = 0;
          center = serial_partitioning(set.prims->data(),begin,end,left,right,isLeft,reduction);
          left.begin  = begin; left.end = center; left.time_range = pinfo.time_range;
          right.begin = center; right.end = end;  right.time_range = pinfo.time_range;
          new (&lset) Set(set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) Set(set.prims,range<size_t>(center,end  ),set.time_range);
          //assert(area(left.geomBounds) >= 0.0f);
          //assert(area(right.geomBounds) >= 0.0f);
        }

        /*! array partitioning */
        __forceinline void temporal_split(const TemporalSplit& split, const PrimInfo2& pinfo, const Set& set, PrimInfo2& linfo, Set& lset, PrimInfo2& rinfo, Set& rset) 
        {
          //unsigned numTimeSegments = split.dim;
          float center_time = split.fpos;
          /* split time range */
          //const float center_time = set.time_range.center();
          //const float center_time = round(set.time_range.center() * float(numTimeSegments)) / float(numTimeSegments);
          const BBox1f time_range0(set.time_range.lower,center_time);
          const BBox1f time_range1(center_time,set.time_range.upper);
          
          /* calculate primrefs for first time range */
          linfo = empty;
          std::shared_ptr<avector<PrimRef2>> lprims(new avector<PrimRef2>(set.object_range.size()));
          for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++) 
          {
            const avector<PrimRef2>& prims = *set.prims;
            const unsigned geomID = prims[i].geomID();
            const unsigned primID = prims[i].primID();
            const LBBox3fa lbounds = ((Mesh*)scene->get(geomID))->linearBounds(primID,time_range0);
            const unsigned num_time_segments = calculateNumOverlappingTimeSegments(geomID,time_range0);
            const PrimRef2 prim(lbounds,num_time_segments,geomID,primID);
            (*lprims)[i-set.object_range.begin()] = prim;
            linfo.add_primref(prim);
          }
          linfo.time_range = time_range0;
          lset = Set(lprims,time_range0);

          /* calculate primrefs for second time range */
          rinfo = empty;
          std::shared_ptr<avector<PrimRef2>> rprims(new avector<PrimRef2>(set.object_range.size()));
          for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++) 
          {
            const avector<PrimRef2>& prims = *set.prims;
            const unsigned geomID = prims[i].geomID();
            const unsigned primID = prims[i].primID();
            const LBBox3fa lbounds = ((Mesh*)scene->get(geomID))->linearBounds(primID,time_range1);
            const unsigned num_time_segments = calculateNumOverlappingTimeSegments(geomID,time_range1);
            const PrimRef2 prim(lbounds,num_time_segments,geomID,primID);
            (*rprims)[i-set.object_range.begin()] = prim;
            rinfo.add_primref(prim);
          }
          rinfo.time_range = time_range1;
          rset = Set(rprims,time_range1);
        }

        void deterministic_order(const Set& set) 
        {
          /* required as parallel partition destroys original primitive order */
          PrimRef2* prims = set.prims->data();
          std::sort(&prims[set.object_range.begin()],&prims[set.object_range.end()]);
        }

        void splitFallback(const Set& set, PrimInfo2& linfo, Set& lset, PrimInfo2& rinfo, Set& rset) // FIXME: also perform time split here?
        {
          avector<PrimRef2>& prims = *set.prims;

          const size_t begin = set.object_range.begin();
          const size_t end   = set.object_range.end();
          const size_t center = (begin + end)/2;
          
          linfo = empty;
          for (size_t i=begin; i<center; i++)
            linfo.add_primref(prims[i]);
          linfo.begin = begin; linfo.end = center; linfo.time_range = set.time_range;
          
          rinfo = empty;
          for (size_t i=center; i<end; i++)
            rinfo.add_primref(prims[i]);	
          rinfo.begin = center; rinfo.end = end; rinfo.time_range = set.time_range;
          
          new (&lset) Set(set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) Set(set.prims,range<size_t>(center,end  ),set.time_range);
        }

      private:
        Scene* scene;
        int numTimeSegments;
      };
  }
}
