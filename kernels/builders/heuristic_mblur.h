// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
#include "heuristic_binning.h"

#define MBLUR_TIME_SPLIT_THRESHOLD 1.25f
#define MBLUR_TIME_SPLIT_LOCATIONS 1

namespace embree
{
  namespace isa
  { 
    /*! Performs standard object binning */
    template<typename RecalculatePrimRef, size_t BINS>
      struct HeuristicMBlur
      {
        typedef BinSplit<BINS> Split;
#if MBLUR_BIN_LBBOX
        typedef BinInfoT<BINS,PrimRefMB,LBBox3fa> ObjectBinner;
#else
        typedef BinInfoT<BINS,PrimRefMB,BBox3fa> ObjectBinner;
#endif
        typedef mvector<PrimRefMB>* PrimRefVector;

        enum
        {
          SPLIT_OBJECT   =  0,
          SPLIT_TEMPORAL = -1,
          SPLIT_FALLBACK = -2,
        };

        struct Set 
        {
          __forceinline Set() {}

          __forceinline Set(PrimRefVector prims, range<size_t> object_range, BBox1f time_range)
            : prims(prims), object_range(object_range), time_range(time_range) {}

          __forceinline Set(PrimRefVector prims, BBox1f time_range = BBox1f(0.0f,1.0f))
            : prims(prims), object_range(range<size_t>(0,prims->size())), time_range(time_range) {}

          __forceinline LBBox3fa linearBounds(const RecalculatePrimRef& recalculatePrimRef) const
          {
            auto reduce = [&](const range<size_t>& r) -> LBBox3fa
            {
              LBBox3fa cbounds(empty);
              for (size_t j = r.begin(); j < r.end(); j++)
              {
                PrimRefMB& ref = (*prims)[j];
                auto bn = recalculatePrimRef.linearBounds(ref, time_range);
                cbounds.extend(bn.first);
              };
              return cbounds;
            };

            return parallel_reduce(object_range.begin(), object_range.end(), PARALLEL_FIND_BLOCK_SIZE, PARALLEL_THRESHOLD, LBBox3fa(empty),
                                   reduce,
                                   [&](const LBBox3fa& b0, const LBBox3fa& b1) -> LBBox3fa { return merge(b0, b1); });
          }

        public:
          PrimRefVector prims;
          range<size_t> object_range;
          BBox1f time_range;
        };

        static const size_t PARALLEL_THRESHOLD = 3 * 1024;
        static const size_t PARALLEL_FIND_BLOCK_SIZE = 1024;
        static const size_t PARALLEL_PARTITION_BLOCK_SIZE = 128;

        HeuristicMBlur (MemoryMonitorInterface* device, const RecalculatePrimRef& recalculatePrimRef)
          : device(device), recalculatePrimRef(recalculatePrimRef) {}

        /*! finds the best split */
        const Split find(Set& set, PrimInfoMB& pinfo, const size_t logBlockSize)
        {
          /* first try standard object split */
          const Split object_split = object_find(set,pinfo,logBlockSize);
          const float object_split_sah = object_split.splitSAH();
  
          /* do temporal splits only if the the time range is big enough */
          if (set.time_range.size() > 1.01f/float(pinfo.max_num_time_segments))
          {
            const Split temporal_split = temporal_find(set, pinfo, logBlockSize);
            const float temporal_split_sah = temporal_split.splitSAH();

            /* take temporal split if it improved SAH */
            if (temporal_split_sah < object_split_sah)
              return temporal_split;
          }

          return object_split;
        }

        /*! finds the best split */
        const Split findFallback(Set& set, PrimInfoMB& pinfo, const size_t logBlockSize, const bool singleLeafTimeSegment)
        {
          /* test if one primitive has two time segments in time range, if so split time */
          if (singleLeafTimeSegment)
          {
            for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++) 
            {
              const PrimRefMB& prim = (*set.prims)[i];
              const range<int> itime_range = recalculatePrimRef(prim,pinfo.time_range).second;
              const int localTimeSegments = itime_range.size();
              assert(localTimeSegments > 0);
              if (localTimeSegments > 1) {
                const int icenter = (itime_range.begin() + itime_range.end())/2;
                const float splitTime = float(icenter)/float(prim.totalTimeSegments());
                return Split(1.0f,SPLIT_TEMPORAL,0,splitTime);
              }
            }
          }
          
          /* otherwise return fallback split */
          return Split(1.0f,SPLIT_FALLBACK);
        }

        /*! finds the best split */
        const Split object_find(const Set& set, const PrimInfoMB& pinfo, const size_t logBlockSize)
        {
          ObjectBinner binner(empty); // FIXME: this clear can be optimized away
          const BinMapping<BINS> mapping(pinfo.centBounds,pinfo.size());
          binner.bin_parallel(set.prims->data(),set.object_range.begin(),set.object_range.end(),PARALLEL_FIND_BLOCK_SIZE,PARALLEL_THRESHOLD,mapping);
          Split osplit = binner.best(mapping,logBlockSize);
          osplit.sah *= pinfo.time_range.size();
          if (!osplit.valid()) osplit.data = SPLIT_FALLBACK; // use fallback split
          return osplit;
        }

        template<int LOCATIONS>
        struct TemporalBinInfo
        {
          __forceinline TemporalBinInfo () {
          }
          
          __forceinline TemporalBinInfo (EmptyTy)
          {
            for (size_t i=0; i<LOCATIONS; i++)
            {
              count0[i] = count1[i] = 0;
              bounds0[i] = bounds1[i] = empty;
            }
          }
          
          void bin(const PrimRefMB* prims, size_t begin, size_t end, BBox1f time_range, size_t numTimeSegments, const RecalculatePrimRef& recalculatePrimRef)
          {
            for (int b=0; b<MBLUR_TIME_SPLIT_LOCATIONS; b++)
            {
              float t = float(b+1)/float(MBLUR_TIME_SPLIT_LOCATIONS+1);
              float ct = lerp(time_range.lower,time_range.upper,t);
              const float center_time = round(ct * float(numTimeSegments)) / float(numTimeSegments);
              if (center_time <= time_range.lower) continue;
              if (center_time >= time_range.upper) continue;
              const BBox1f dt0(time_range.lower,center_time);
              const BBox1f dt1(center_time,time_range.upper);
              
              /* find linear bounds for both time segments */
              for (size_t i=begin; i<end; i++) 
              {
                auto bn0 = recalculatePrimRef.linearBounds(prims[i],dt0);
                auto bn1 = recalculatePrimRef.linearBounds(prims[i],dt1);
#if MBLUR_BIN_LBBOX
                bounds0[b].extend(bn0.first);
                bounds1[b].extend(bn1.first);
#else
                bounds0[b].extend(bn0.first.interpolate(0.5f));
                bounds1[b].extend(bn1.first.interpolate(0.5f));
#endif
                count0[b] += bn0.second.size();
                count1[b] += bn1.second.size();
              }
            }
          }

          __forceinline void bin_parallel(const PrimRefMB* prims, size_t begin, size_t end, size_t blockSize, size_t parallelThreshold, BBox1f time_range, size_t numTimeSegments, const RecalculatePrimRef& recalculatePrimRef) 
          {
            if (likely(end-begin < parallelThreshold)) {
              bin(prims,begin,end,time_range,numTimeSegments,recalculatePrimRef);
            } else {
              TemporalBinInfo binner(empty);
              *this = parallel_reduce(begin,end,blockSize,binner,
                                      [&](const range<size_t>& r) -> TemporalBinInfo { TemporalBinInfo binner(empty); binner.bin(prims, r.begin(), r.end(), time_range, numTimeSegments, recalculatePrimRef); return binner; },
                                      [&](const TemporalBinInfo& b0, const TemporalBinInfo& b1) -> TemporalBinInfo { TemporalBinInfo r = b0; r.merge(b1); return r; });
            }
          }
          
          /*! merges in other binning information */
          __forceinline void merge (const TemporalBinInfo& other)
          {
            for (size_t i=0; i<LOCATIONS; i++) 
            {
              count0[i] += other.count0[i];
              count1[i] += other.count1[i];
              bounds0[i].extend(other.bounds0[i]);
              bounds1[i].extend(other.bounds1[i]);
            }
          }
          
          Split best(int logBlockSize, BBox1f time_range, size_t numTimeSegments)
          {
            float bestSAH = inf;
            float bestPos = 0.0f;
            for (int b=0; b<MBLUR_TIME_SPLIT_LOCATIONS; b++)
            {
              float t = float(b+1)/float(MBLUR_TIME_SPLIT_LOCATIONS+1);
              float ct = lerp(time_range.lower,time_range.upper,t);
              const float center_time = round(ct * float(numTimeSegments)) / float(numTimeSegments);
              if (center_time <= time_range.lower) continue;
              if (center_time >= time_range.upper) continue;
              const BBox1f dt0(time_range.lower,center_time);
              const BBox1f dt1(center_time,time_range.upper);
              
              /* calculate sah */
              const size_t lCount = (count0[b]+(1 << logBlockSize)-1) >> int(logBlockSize);
              const size_t rCount = (count1[b]+(1 << logBlockSize)-1) >> int(logBlockSize);
              const float sah0 = expectedApproxHalfArea(bounds0[b])*float(lCount)*dt0.size();
              const float sah1 = expectedApproxHalfArea(bounds1[b])*float(rCount)*dt1.size();
              const float sah = sah0+sah1;
              if (sah < bestSAH) {
                bestSAH = sah;
                bestPos = center_time;
              }
            }
            assert(bestSAH != float(inf));
            return Split(bestSAH*MBLUR_TIME_SPLIT_THRESHOLD,SPLIT_TEMPORAL,0,bestPos);
          }
          
        public:
          size_t count0[LOCATIONS];
          size_t count1[LOCATIONS];
#if MBLUR_BIN_LBBOX
          LBBox3fa bounds0[LOCATIONS];
          LBBox3fa bounds1[LOCATIONS];
#else
          BBox3fa bounds0[LOCATIONS];
          BBox3fa bounds1[LOCATIONS];
#endif
        };
        
        /*! finds the best split */
        const Split temporal_find(const Set& set, const PrimInfoMB& pinfo, const size_t logBlockSize)
        {
          assert(set.object_range.size() > 0);
          unsigned numTimeSegments = pinfo.max_num_time_segments;
          TemporalBinInfo<MBLUR_TIME_SPLIT_LOCATIONS> binner(empty);
          binner.bin_parallel(set.prims->data(),set.object_range.begin(),set.object_range.end(),PARALLEL_FIND_BLOCK_SIZE,PARALLEL_THRESHOLD,set.time_range,numTimeSegments,recalculatePrimRef);
          Split tsplit = binner.best(logBlockSize,set.time_range,numTimeSegments);
          if (!tsplit.valid()) tsplit.data = SPLIT_FALLBACK; // use fallback split
          return tsplit;
        }
        
        /*! array partitioning */
        void split(const Split& split, const PrimInfoMB& pinfo, const Set& set, PrimInfoMB& left, Set& lset, PrimInfoMB& right, Set& rset)
        {
          /* perform fallback split */
          //if (unlikely(!split.valid())) {
          if (unlikely(split.data == SPLIT_FALLBACK)) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }
          /* perform temporal split */
          else if (unlikely(split.data == SPLIT_TEMPORAL)) {
            temporal_split(split,pinfo,set,left,lset,right,rset);
          }
          /* perform object split */
          else {
            object_split(split,pinfo,set,left,lset,right,rset);
          }
        }

        /*! array partitioning */
        __forceinline void object_split(const Split& split, const PrimInfoMB& pinfo, const Set& set, PrimInfoMB& left, Set& lset, PrimInfoMB& right, Set& rset)
        {
          const size_t begin = set.object_range.begin();
          const size_t end   = set.object_range.end();
          left = empty;
          right = empty;
          const vint4 vSplitPos(split.pos);
          const vbool4 vSplitMask(1 << split.dim);
          auto isLeft = [&] (const PrimRefMB &ref) { return any(((vint4)split.mapping.bin_unsafe(ref) < vSplitPos) & vSplitMask); };
          auto reduction = [] (PrimInfoMB& pinfo, const PrimRefMB& ref) { pinfo.add_primref(ref); };
          auto reduction2 = [] (PrimInfoMB& pinfo0,const PrimInfoMB& pinfo1) { pinfo0.merge(pinfo1); };
          size_t center = parallel_partitioning(set.prims->data(),begin,end,empty,left,right,isLeft,reduction,reduction2,PARALLEL_PARTITION_BLOCK_SIZE,PARALLEL_THRESHOLD);
          left.begin  = begin;  left.end = center; left.time_range = pinfo.time_range;
          right.begin = center; right.end = end;   right.time_range = pinfo.time_range;
          new (&lset) Set(set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) Set(set.prims,range<size_t>(center,end  ),set.time_range);
        }

        /*! array partitioning */
        __forceinline void temporal_split(const Split& split, const PrimInfoMB& pinfo, const Set& set, PrimInfoMB& linfo, Set& lset, int side)
        {
          float center_time = split.fpos;
          const BBox1f time_range0(set.time_range.lower,center_time);
          const BBox1f time_range1(center_time,set.time_range.upper);
          const BBox1f time_range = side ? time_range1 : time_range0;
          
          /* calculate primrefs for first time range */
          PrimRefVector lprims = new mvector<PrimRefMB>(device, set.object_range.size());
          auto reduction_func0 = [&] ( const range<size_t>& r) {
            PrimInfoMB pinfo = empty;
            for (size_t i=r.begin(); i<r.end(); i++) 
            {
              mvector<PrimRefMB>& prims = *set.prims;
              auto bn0 = recalculatePrimRef(prims[i],time_range);
              const PrimRefMB& prim = bn0.first;
              (*lprims)[i-set.object_range.begin()] = prim;
              pinfo.add_primref(prim);
            }
            return pinfo;
          };        
          linfo = parallel_reduce(set.object_range.begin(),set.object_range.end(),PARALLEL_PARTITION_BLOCK_SIZE,PARALLEL_THRESHOLD,PrimInfoMB(empty),reduction_func0,
                                  [] (const PrimInfoMB& a, const PrimInfoMB& b) { return PrimInfoMB::merge(a,b); });
   
          linfo.time_range = time_range;
          lset = Set(lprims,time_range);
        }

        __forceinline void temporal_split(const Split& split, const PrimInfoMB& pinfo, const Set& set, PrimInfoMB& linfo, Set& lset, PrimInfoMB& rinfo, Set& rset)
        {
          temporal_split(split,pinfo,set,linfo,lset,0);
          temporal_split(split,pinfo,set,rinfo,rset,1);
        }

        void deterministic_order(const Set& set) 
        {
          /* required as parallel partition destroys original primitive order */
          PrimRefMB* prims = set.prims->data();
          std::sort(&prims[set.object_range.begin()],&prims[set.object_range.end()]);
        }

        void splitFallback(const Set& set, PrimInfoMB& linfo, Set& lset, PrimInfoMB& rinfo, Set& rset) // FIXME: also perform time split here?
        {
          mvector<PrimRefMB>& prims = *set.prims;

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
        MemoryMonitorInterface* device;              // device to report memory usage to
        const RecalculatePrimRef recalculatePrimRef;
      };

    template<typename BuildRecord, 
      typename RecalculatePrimRef, 
      typename ReductionTy, 
      typename Allocator, 
      typename CreateAllocFunc, 
      typename CreateNodeFunc, 
      typename UpdateNodeFunc, 
      typename CreateLeafFunc, 
      typename ProgressMonitor,
      typename PrimInfo>
      
      class GeneralBVHMBBuilder : public HeuristicMBlur<RecalculatePrimRef,NUM_OBJECT_BINS>
      {
        static const size_t MAX_BRANCHING_FACTOR = 8;        //!< maximal supported BVH branching factor
        static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree of we are that many levels before the maximal tree depth
        static const size_t SINGLE_THREADED_THRESHOLD = 1024;  //!< threshold to switch to single threaded build
        typedef HeuristicMBlur<RecalculatePrimRef,NUM_OBJECT_BINS> Heuristic;

        using Heuristic::SPLIT_OBJECT;
        using Heuristic::SPLIT_TEMPORAL;
        using Heuristic::SPLIT_FALLBACK;
        
      public:

        struct LocalChildList
        {
          struct SharedPrimRefVector
          {
            __forceinline SharedPrimRefVector() {}

            __forceinline SharedPrimRefVector(const BuildRecord& record, size_t refCount = 1)
              : prims(record.prims.prims), refCount(refCount) {}

            __forceinline void incRef()
            {
              refCount++;
            }

            __forceinline void decRef()
            {
              if (--refCount == 0)
                delete prims;
            }

            typename Heuristic::PrimRefVector prims;
            size_t refCount;
          };

          struct Child
          {
            __forceinline Child() {}

            __forceinline Child(const BuildRecord& record, SharedPrimRefVector* sharedPrimVec)
              : record(record), sharedPrimVec(sharedPrimVec) {}

            BuildRecord record;
            SharedPrimRefVector* sharedPrimVec;
          };

          __forceinline LocalChildList (GeneralBVHMBBuilder* builder, BuildRecord& record)
            : builder(builder), numChildren(1), numSharedPrimVecs(1), depth(record.depth)
          {
            /* the local root will be freed in the ancestor where it was created (thus refCount is 2) */
            SharedPrimRefVector* sharedPrimVec = new (&sharedPrimVecs[0]) SharedPrimRefVector(record, 2);
            new (&children[0]) Child(record, sharedPrimVec);
          }

          __forceinline ~LocalChildList()
          {
            for (size_t i = 0; i < numChildren; i++)
              children[i].sharedPrimVec->decRef();
          }

          __forceinline void split(size_t bestChild)
          {
            /* perform best found split */
            BuildRecord& brecord = children[bestChild].record;
            BuildRecord lrecord(depth+1);
            BuildRecord rrecord(depth+1);
            builder->partition(brecord,lrecord,rrecord);

            /* find new splits */
            lrecord.split = builder->find(lrecord);
            rrecord.split = builder->find(rrecord);

            /* add new children */
            SharedPrimRefVector* bsharedPrimVec = children[bestChild].sharedPrimVec;
            if (brecord.split.data == SPLIT_TEMPORAL)
            {
              SharedPrimRefVector* lsharedPrimVec = new (&sharedPrimVecs[numSharedPrimVecs  ]) SharedPrimRefVector(lrecord);
              SharedPrimRefVector* rsharedPrimVec = new (&sharedPrimVecs[numSharedPrimVecs+1]) SharedPrimRefVector(rrecord);
              numSharedPrimVecs += 2;
              new (&children[bestChild  ]) Child(lrecord, lsharedPrimVec);
              new (&children[numChildren]) Child(rrecord, rsharedPrimVec);
              bsharedPrimVec->decRef();
            }
            else
            {
              new (&children[bestChild  ]) Child(lrecord, bsharedPrimVec);
              new (&children[numChildren]) Child(rrecord, bsharedPrimVec);
              bsharedPrimVec->incRef();
            }
            numChildren++;
          }

          __forceinline void splitLargeLeaf(size_t bestChild, bool singleLeafTimeSegment)
          {
            /* perform best found split */
            BuildRecord& brecord = children[bestChild].record;
            BuildRecord lrecord(depth+1);
            BuildRecord rrecord(depth+1);
            builder->partition(brecord,lrecord,rrecord);

            /* find new splits */
            lrecord.split = builder->findFallback(lrecord,singleLeafTimeSegment);
            rrecord.split = builder->findFallback(rrecord,singleLeafTimeSegment);

            /* add new children */
            SharedPrimRefVector* bsharedPrimVec = children[bestChild].sharedPrimVec;
            if (brecord.split.data == SPLIT_TEMPORAL)
            {
              SharedPrimRefVector* lsharedPrimVec = new (&sharedPrimVecs[numSharedPrimVecs  ]) SharedPrimRefVector(lrecord);
              SharedPrimRefVector* rsharedPrimVec = new (&sharedPrimVecs[numSharedPrimVecs+1]) SharedPrimRefVector(rrecord);
              numSharedPrimVecs += 2;
              children[bestChild] = children[numChildren-1];
              new (&children[numChildren-1]) Child(lrecord, lsharedPrimVec);
              new (&children[numChildren+0]) Child(rrecord, rsharedPrimVec);
              bsharedPrimVec->decRef();
            }
            else
            {
              children[bestChild] = children[numChildren-1];
              new (&children[numChildren-1]) Child(lrecord, bsharedPrimVec);
              new (&children[numChildren+0]) Child(rrecord, bsharedPrimVec);
              bsharedPrimVec->incRef();
            }
            numChildren++;
          }

          __forceinline size_t size() const {
            return numChildren;
          }

          __forceinline BuildRecord& operator[] ( size_t i ) {
            return children[i].record;
          }

          __forceinline BuildRecord** childrenArray() 
          {
            for (size_t i=0; i<numChildren; i++) 
              childrenPtrs[i] = &children[i].record;
            return childrenPtrs;
          }

          bool hasTimeSplits() const {
            return false;
          }

          __forceinline void restore(size_t childID) {
          }

          __forceinline ssize_t best()
          {
            /*! find best child to split */
            float bestSAH = neg_inf;
            ssize_t bestChild = -1;
            for (size_t i=0; i<numChildren; i++) 
            {
              if (children[i].record.pinfo.size() <= builder->minLeafSize) continue;
              if (expectedApproxHalfArea(children[i].record.pinfo.geomBounds) > bestSAH) {
                bestChild = i; bestSAH = expectedApproxHalfArea(children[i].record.pinfo.geomBounds);
              } 
            }
            return bestChild;
          }

          __forceinline ssize_t bestLargeLeaf()
          {
            /* find best child with largest bounding box area */
            size_t bestChild = -1;
            size_t bestSize = 0;
            for (size_t i=0; i<numChildren; i++)
            {
              /* ignore leaves as they cannot get split */
              if (children[i].record.pinfo.size() <= builder->maxLeafSize && children[i].record.split.data != SPLIT_TEMPORAL)
                continue;

              /* remember child with largest size */
              if (children[i].record.pinfo.size() > bestSize) {
                bestSize = children[i].record.pinfo.size();
                bestChild = i;
              }
            }
            return bestChild;
          }

        public:
          GeneralBVHMBBuilder* builder;
          Child children[MAX_BRANCHING_FACTOR];
          BuildRecord* childrenPtrs[MAX_BRANCHING_FACTOR];
          SharedPrimRefVector sharedPrimVecs[MAX_BRANCHING_FACTOR*2];
          size_t numChildren;
          size_t numSharedPrimVecs;
          size_t depth;
        };

        GeneralBVHMBBuilder (MemoryMonitorInterface* device,
                             const RecalculatePrimRef recalculatePrimRef,
                             const ReductionTy& identity,
                             CreateAllocFunc& createAlloc, 
                             CreateNodeFunc& createNode, 
                             UpdateNodeFunc& updateNode, 
                             CreateLeafFunc& createLeaf,
                             ProgressMonitor& progressMonitor,
                             const PrimInfo& pinfo,
                             const size_t branchingFactor, const size_t maxDepth, 
                             const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize,
                             const float travCost, const float intCost, const bool singleLeafTimeSegment)
          : HeuristicMBlur<RecalculatePrimRef,NUM_OBJECT_BINS>(device, recalculatePrimRef),
          identity(identity), 
          createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
          progressMonitor(progressMonitor),
          pinfo(pinfo), 
          branchingFactor(branchingFactor), maxDepth(maxDepth),
          logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
          travCost(travCost), intCost(intCost), singleLeafTimeSegment(singleLeafTimeSegment)
        {
          if (branchingFactor > MAX_BRANCHING_FACTOR)
            throw_RTCError(RTC_UNKNOWN_ERROR,"bvh_builder: branching factor too large");
        }
        
        const ReductionTy createLargeLeaf(BuildRecord& current, Allocator alloc)
        {
          /* this should never occur but is a fatal error */
          if (current.depth > maxDepth) 
            throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");

          /* replace already found split by fallback split */
          current.split = findFallback(current,singleLeafTimeSegment);

          /* create leaf for few primitives */
          if (current.pinfo.size() <= maxLeafSize && current.split.data != SPLIT_TEMPORAL)
            return createLeaf(current,alloc);
          
          /* fill all children by always splitting the largest one */
          ReductionTy values[MAX_BRANCHING_FACTOR];
          LocalChildList children(this,current);
        
          do {  
            ssize_t bestChild = children.bestLargeLeaf();
            if (bestChild == -1) break;
            children.splitLargeLeaf(bestChild, singleLeafTimeSegment);
          } while (children.size() < branchingFactor);
          
          /* create node */
          auto node = createNode(current,children.childrenArray(),children.size(),alloc);
          
          /* recurse into each child  and perform reduction */
          for (size_t i=0; i<children.size(); i++)
            values[i] = createLargeLeaf(children[i],alloc);
          
          /* perform reduction */
          return updateNode(node,current.prims,values,children.size());
        }
        
        __forceinline const typename Heuristic::Split find(BuildRecord& current) {
          return Heuristic::find (current.prims,current.pinfo,logBlockSize);
        }

        __forceinline const typename Heuristic::Split findFallback(BuildRecord& current, bool singleLeafTimeSegment) {
          return Heuristic::findFallback (current.prims,current.pinfo,logBlockSize,singleLeafTimeSegment);
        }
        
        __forceinline void partition(BuildRecord& brecord, BuildRecord& lrecord, BuildRecord& rrecord) {
          Heuristic::split(brecord.split,brecord.pinfo,brecord.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
        }
        
        const ReductionTy recurse(BuildRecord& current, Allocator alloc, bool toplevel)
        {
          if (alloc == nullptr)
            alloc = createAlloc();

          /* call memory monitor function to signal progress */
          if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
            progressMonitor(current.size());
          
          /*! compute leaf and split cost */
          const float leafSAH  = intCost*current.pinfo.leafSAH(logBlockSize);
          const float splitSAH = travCost*current.pinfo.halfArea()+intCost*current.split.splitSAH();
          assert((current.pinfo.size() == 0) || ((leafSAH >= 0) && (splitSAH >= 0)));
          assert(current.pinfo.size() == current.prims.object_range.size());

          /*! create a leaf node when threshold reached or SAH tells us to stop */
          if (current.pinfo.size() <= minLeafSize || current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || (current.pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
            Heuristic::deterministic_order(current.prims);
            return createLargeLeaf(current,alloc);
          }
          
          /*! initialize child list */
          ReductionTy values[MAX_BRANCHING_FACTOR];
          LocalChildList children(this,current);
          
          /*! split until node is full or SAH tells us to stop */
          do {
            ssize_t bestChild = children.best();
            if (bestChild == -1) break;
            children.split(bestChild);
          } while (children.size() < branchingFactor);
          
          /* sort buildrecords for simpler shadow ray traversal */
          //std::sort(&children[0],&children[children.size()],std::greater<BuildRecord>()); // FIXME: reduces traversal performance of bvh8.triangle4 (need to verified) !!
          
          /*! create an inner node */
          auto node = createNode(current,children.childrenArray(),children.size(),alloc);
          
          /* spawn tasks */
          if (current.size() > SINGLE_THREADED_THRESHOLD && !children.hasTimeSplits()) 
          {
            /*! parallel_for is faster than spawing sub-tasks */
            parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  values[i] = recurse(children[i],nullptr,true); 
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
            /* perform reduction */
            return updateNode(node,current.prims,values,children.size());
          }
          /* recurse into each child */
          else 
          {
            //for (size_t i=0; i<numChildren; i++)
            for (ssize_t i=children.size()-1; i>=0; i--) {
              children.restore(i);
              values[i] = recurse(children[i],alloc,false);
            }
            
            /* perform reduction */
            return updateNode(node,current.prims,values,children.size());
          }
        }
        
        /*! builder entry function */
        __forceinline const ReductionTy operator() (BuildRecord& record)
        {
          //BuildRecord br(record);
          record.split = find(record); 
          ReductionTy ret = recurse(record,nullptr,true);
          _mm_mfence(); // to allow non-temporal stores during build
          return ret;
        }
        
      private:
        const ReductionTy identity;
        CreateAllocFunc& createAlloc;
        CreateNodeFunc& createNode;
        UpdateNodeFunc& updateNode;
        CreateLeafFunc& createLeaf;
        ProgressMonitor& progressMonitor;
        
      private:
        const PrimInfo& pinfo;
        const size_t branchingFactor;
        const size_t maxDepth;
        const size_t logBlockSize;
        const size_t minLeafSize;
        const size_t maxLeafSize;
        const float travCost;
        const float intCost;
        const bool singleLeafTimeSegment;
      };
  }
}
