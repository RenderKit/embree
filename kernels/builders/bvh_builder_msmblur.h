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

#define NUM_TEMPORAL_BINS 2

#include "../common/primref_mb.h"
#include "heuristic_binning_array_aligned.h"
#include "heuristic_timesplit_array.h"

namespace embree
{
  namespace isa
  { 
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
      
      class GeneralBVHMBBuilder
      {
        static const size_t MAX_BRANCHING_FACTOR = 8;        //!< maximal supported BVH branching factor
        static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree of we are that many levels before the maximal tree depth
        static const size_t SINGLE_THREADED_THRESHOLD = 1024;  //!< threshold to switch to single threaded build

        typedef BinSplit<NUM_OBJECT_BINS> Split;
        typedef mvector<PrimRefMB>* PrimRefVector;

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

            PrimRefVector prims;
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
            if (brecord.split.data == Split::SPLIT_TEMPORAL)
            {
              new (&children[bestChild  ]) Child(lrecord, new (&sharedPrimVecs[numSharedPrimVecs++]) SharedPrimRefVector(lrecord));
              new (&children[numChildren]) Child(rrecord, new (&sharedPrimVecs[numSharedPrimVecs++]) SharedPrimRefVector(rrecord));
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
            if (brecord.split.data == Split::SPLIT_TEMPORAL)
            {
              children[bestChild] = children[numChildren-1];
              new (&children[numChildren-1]) Child(lrecord, new (&sharedPrimVecs[numSharedPrimVecs++]) SharedPrimRefVector(lrecord));
              new (&children[numChildren+0]) Child(rrecord, new (&sharedPrimVecs[numSharedPrimVecs++]) SharedPrimRefVector(rrecord));
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
              if (children[i].record.pinfo.size() <= builder->maxLeafSize && children[i].record.split.data != Split::SPLIT_TEMPORAL)
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
          : recalculatePrimRef(recalculatePrimRef),
          heuristicObjectSplit(),
          heuristicTemporalSplit(device, recalculatePrimRef),
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
        
        __forceinline const Split find(BuildRecord& current) {
          return find (current.prims,current.pinfo,logBlockSize);
        }

        /*! finds the best split */
        const Split find(SetMB& set, PrimInfoMB& pinfo, const size_t logBlockSize)
        {
          /* first try standard object split */
          const Split object_split = heuristicObjectSplit.find(set,pinfo,logBlockSize);
          const float object_split_sah = object_split.splitSAH();
          
          /* do temporal splits only if the the time range is big enough */
          if (set.time_range.size() > 1.01f/float(pinfo.max_num_time_segments))
          {
            const Split temporal_split = heuristicTemporalSplit.find(set, pinfo, logBlockSize);
            const float temporal_split_sah = temporal_split.splitSAH();

            /* take temporal split if it improved SAH */
            if (temporal_split_sah < object_split_sah)
              return temporal_split;
          }

          return object_split;
        }

        __forceinline void partition(BuildRecord& brecord, BuildRecord& lrecord, BuildRecord& rrecord) {
          split(brecord.split,brecord.pinfo,brecord.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
        }

        /*! array partitioning */
        void split(const Split& split, const PrimInfoMB& pinfo, const SetMB& set, PrimInfoMB& left, SetMB& lset, PrimInfoMB& right, SetMB& rset)
        {
          /* perform fallback split */
          //if (unlikely(!split.valid())) {
          if (unlikely(split.data == Split::SPLIT_FALLBACK)) {
            deterministic_order(set);
            return splitFallback(set,left,lset,right,rset);
          }
          /* perform temporal split */
          else if (unlikely(split.data == Split::SPLIT_TEMPORAL)) {
            heuristicTemporalSplit.split(split,pinfo,set,left,lset,right,rset);
          }
          /* perform object split */
          else {
            heuristicObjectSplit.split(split,pinfo,set,left,lset,right,rset);
          }
        }

        __forceinline Split findFallback(BuildRecord& current, bool singleLeafTimeSegment) {
          return findFallback (current.prims,current.pinfo,logBlockSize,singleLeafTimeSegment);
        }
             
        /*! finds the best split */
        const Split findFallback(SetMB& set, PrimInfoMB& pinfo, const size_t logBlockSize, const bool singleLeafTimeSegment)
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
                return Split(1.0f,Split::SPLIT_TEMPORAL,0,splitTime);
              }
            }
          }
          
          /* otherwise return fallback split */
          return Split(1.0f,Split::SPLIT_FALLBACK);
        }

        void splitFallback(const SetMB& set, PrimInfoMB& linfo, SetMB& lset, PrimInfoMB& rinfo, SetMB& rset) // FIXME: also perform time split here?
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
          
          new (&lset) SetMB(set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) SetMB(set.prims,range<size_t>(center,end  ),set.time_range);
        }

        void deterministic_order(const SetMB& set) 
        {
          /* required as parallel partition destroys original primitive order */
          PrimRefMB* prims = set.prims->data();
          std::sort(&prims[set.object_range.begin()],&prims[set.object_range.end()]);
        }

        const ReductionTy createLargeLeaf(BuildRecord& current, Allocator alloc)
        {
          /* this should never occur but is a fatal error */
          if (current.depth > maxDepth) 
            throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");

          /* replace already found split by fallback split */
          current.split = findFallback(current,singleLeafTimeSegment);

          /* create leaf for few primitives */
          if (current.pinfo.size() <= maxLeafSize && current.split.data != Split::SPLIT_TEMPORAL)
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
            deterministic_order(current.prims);
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
        const RecalculatePrimRef recalculatePrimRef;
        HeuristicArrayBinningMB<NUM_OBJECT_BINS> heuristicObjectSplit;
        HeuristicMBlurTemporalSplit<RecalculatePrimRef,NUM_TEMPORAL_BINS> heuristicTemporalSplit;
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
