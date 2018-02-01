// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../common/primref_mb.h"

#define MBLUR_TIME_SPLIT_THRESHOLD 1.25f

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<typename PrimRefMB, typename RecalculatePrimRef, size_t BINS>
      struct HeuristicMBlurTemporalSplit
      {
        typedef BinSplit<MBLUR_NUM_OBJECT_BINS> Split;
        typedef mvector<PrimRefMB>* PrimRefVector;
        typedef typename PrimRefMB::BBox BBox; 

        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

        HeuristicMBlurTemporalSplit (MemoryMonitorInterface* device, const RecalculatePrimRef& recalculatePrimRef)
          : device(device), recalculatePrimRef(recalculatePrimRef) {}

        struct TemporalBinInfo
        {
          __forceinline TemporalBinInfo () {
          }

          __forceinline TemporalBinInfo (EmptyTy)
          {
            for (size_t i=0; i<BINS-1; i++)
            {
              count0[i] = count1[i] = 0;
              bounds0[i] = bounds1[i] = empty;
            }
          }
          
          void bin(const PrimRefMB* prims, size_t begin, size_t end, BBox1f time_range, size_t numTimeSegments, const RecalculatePrimRef& recalculatePrimRef)
          {
            for (int b=0; b<BINS-1; b++)
            {
              const float t = float(b+1)/float(BINS);
              const float ct = lerp(time_range.lower,time_range.upper,t);
              const float center_time = roundf(ct * float(numTimeSegments)) / float(numTimeSegments);
              if (center_time <= time_range.lower) continue;
              if (center_time >= time_range.upper) continue;
              const BBox1f dt0(time_range.lower,center_time);
              const BBox1f dt1(center_time,time_range.upper);
              
              /* find linear bounds for both time segments */
              for (size_t i=begin; i<end; i++) 
              {
                const LBBox3fa bn0 = recalculatePrimRef.linearBounds(prims[i],dt0);
                const LBBox3fa bn1 = recalculatePrimRef.linearBounds(prims[i],dt1);
#if MBLUR_BIN_LBBOX
                bounds0[b].extend(bn0);
                bounds1[b].extend(bn1);
#else
                bounds0[b].extend(bn0.interpolate(0.5f));
                bounds1[b].extend(bn1.interpolate(0.5f));
#endif
                count0[b] += getTimeSegmentRange(dt0,(float)prims[i].totalTimeSegments()).size();
                count1[b] += getTimeSegmentRange(dt1,(float)prims[i].totalTimeSegments()).size();
              }
            }
          }

          __forceinline void bin_parallel(const PrimRefMB* prims, size_t begin, size_t end, size_t blockSize, size_t parallelThreshold, BBox1f time_range, size_t numTimeSegments, const RecalculatePrimRef& recalculatePrimRef) 
          {
            if (likely(end-begin < parallelThreshold)) {
              bin(prims,begin,end,time_range,numTimeSegments,recalculatePrimRef);
            } 
            else 
            {
              auto bin = [&](const range<size_t>& r) -> TemporalBinInfo { 
                TemporalBinInfo binner(empty); binner.bin(prims, r.begin(), r.end(), time_range, numTimeSegments, recalculatePrimRef); return binner; 
              };
              *this = parallel_reduce(begin,end,blockSize,TemporalBinInfo(empty),bin,merge2);
            }
          }
          
          /*! merges in other binning information */
          __forceinline void merge (const TemporalBinInfo& other)
          {
            for (size_t i=0; i<BINS-1; i++) 
            {
              count0[i] += other.count0[i];
              count1[i] += other.count1[i];
              bounds0[i].extend(other.bounds0[i]);
              bounds1[i].extend(other.bounds1[i]);
            }
          }

          static __forceinline const TemporalBinInfo merge2(const TemporalBinInfo& a, const TemporalBinInfo& b) {
            TemporalBinInfo r = a; r.merge(b); return r;
          }
                    
          Split best(int logBlockSize, BBox1f time_range, size_t numTimeSegments)
          {
            float bestSAH = inf;
            float bestPos = 0.0f;
            for (int b=0; b<BINS-1; b++)
            {
              float t = float(b+1)/float(BINS);
              float ct = lerp(time_range.lower,time_range.upper,t);
              const float center_time = roundf(ct * float(numTimeSegments)) / float(numTimeSegments);
              if (center_time <= time_range.lower) continue;
              if (center_time >= time_range.upper) continue;
              const BBox1f dt0(time_range.lower,center_time);
              const BBox1f dt1(center_time,time_range.upper);
              
              /* calculate sah */
              const size_t lCount = (count0[b]+(size_t(1) << logBlockSize)-1) >> int(logBlockSize);
              const size_t rCount = (count1[b]+(size_t(1) << logBlockSize)-1) >> int(logBlockSize);
              const float sah0 = expectedApproxHalfArea(bounds0[b])*float(lCount)*dt0.size();
              const float sah1 = expectedApproxHalfArea(bounds1[b])*float(rCount)*dt1.size();
              const float sah = sah0+sah1;
              if (sah < bestSAH) {
                bestSAH = sah;
                bestPos = center_time;
              }
            }
            assert(bestSAH != float(inf));
            return Split(bestSAH*MBLUR_TIME_SPLIT_THRESHOLD,(unsigned)Split::SPLIT_TEMPORAL,0,bestPos);
          }
          
        public:
          size_t count0[BINS-1];
          size_t count1[BINS-1];
          BBox bounds0[BINS-1];
          BBox bounds1[BINS-1];
        };
        
        /*! finds the best split */
        const Split find(const SetMB& set, const size_t logBlockSize)
        {
          assert(set.object_range.size() > 0);
          unsigned numTimeSegments = unsigned(set.max_num_time_segments);
          TemporalBinInfo binner(empty);
          binner.bin_parallel(set.prims->data(),set.object_range.begin(),set.object_range.end(),PARALLEL_FIND_BLOCK_SIZE,PARALLEL_THRESHOLD,set.time_range,numTimeSegments,recalculatePrimRef);
          Split tsplit = binner.best((int)logBlockSize,set.time_range,numTimeSegments);
          if (!tsplit.valid()) tsplit.data = Split::SPLIT_FALLBACK; // use fallback split
          return tsplit;
        }

        __forceinline std::unique_ptr<mvector<PrimRefMB>> split(const Split& tsplit, const SetMB& set, SetMB& lset, SetMB& rset)
        {
          float center_time = tsplit.fpos;
          const BBox1f time_range0(set.time_range.lower,center_time);
          const BBox1f time_range1(center_time,set.time_range.upper);
          mvector<PrimRefMB>& prims = *set.prims;
          
          /* calculate primrefs for first time range */
          std::unique_ptr<mvector<PrimRefMB>> new_vector(new mvector<PrimRefMB>(device, set.object_range.size()));
          PrimRefVector lprims = new_vector.get();

          auto reduction_func0 = [&] ( const range<size_t>& r) {
            PrimInfoMB pinfo = empty;
            for (size_t i=r.begin(); i<r.end(); i++) 
            {
              const PrimRefMB& prim = recalculatePrimRef(prims[i],time_range0);
              (*lprims)[i-set.object_range.begin()] = prim;
              pinfo.add_primref(prim);
            }
            return pinfo;
          };        
          const PrimInfoMB linfo = parallel_reduce(set.object_range,PARALLEL_PARTITION_BLOCK_SIZE,PARALLEL_THRESHOLD,PrimInfoMB(empty),reduction_func0,PrimInfoMB::merge2);
          lset = SetMB(linfo,lprims,time_range0);

          /* calculate primrefs for second time range */
          auto reduction_func1 = [&] ( const range<size_t>& r) {
            PrimInfoMB pinfo = empty;
            for (size_t i=r.begin(); i<r.end(); i++) 
            {
              const PrimRefMB& prim = recalculatePrimRef(prims[i],time_range1);
              prims[i] = prim;
              pinfo.add_primref(prim);
            }
            return pinfo;
          };        
          const PrimInfoMB rinfo = parallel_reduce(set.object_range,PARALLEL_PARTITION_BLOCK_SIZE,PARALLEL_THRESHOLD,PrimInfoMB(empty),reduction_func1,PrimInfoMB::merge2);
          rset = SetMB(rinfo,&prims,set.object_range,time_range1);

          return new_vector;
        }

      private:
        MemoryMonitorInterface* device;              // device to report memory usage to
        const RecalculatePrimRef recalculatePrimRef;
      };
  }
}
