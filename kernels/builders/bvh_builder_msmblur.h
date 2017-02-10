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
    template<typename T>
      struct SharedVector
      {
        __forceinline SharedVector() {}
        
        __forceinline SharedVector(T* ptr, size_t refCount = 1)
          : prims(ptr), refCount(refCount) {}
        
        __forceinline void incRef() {
          refCount++;
        }
        
        __forceinline void decRef()
        {
          if (--refCount == 0)
            delete prims;
        }
        
        T* prims;
        size_t refCount;
      };
    
    template<typename BuildRecord, int MAX_BRANCHING_FACTOR>
      struct LocalChildListT
      {
        typedef SharedVector<mvector<PrimRefMB>> SharedPrimRefVector;
        
        __forceinline LocalChildListT (BuildRecord& record)
          : numChildren(1), numSharedPrimVecs(1), depth(record.depth)
        {
          /* the local root will be freed in the ancestor where it was created (thus refCount is 2) */
          children[0] = record;
          primvecs[0] = new (&sharedPrimVecs[0]) SharedPrimRefVector(record.prims.prims, 2);
        }
        
        __forceinline ~LocalChildListT()
        {
          for (size_t i = 0; i < numChildren; i++)
            primvecs[i]->decRef();
        }
        
        __forceinline BuildRecord& operator[] ( const size_t i ) {
          return children[i];
        }
        
        __forceinline size_t size() const {
          return numChildren;
        }
        
        __forceinline void split(int bestChild, BuildRecord& lrecord, BuildRecord& rrecord, std::unique_ptr<mvector<PrimRefMB>> new_vector)
        {
          SharedPrimRefVector* bsharedPrimVec = primvecs[bestChild];
          if (lrecord.prims.prims == bsharedPrimVec->prims) {
            primvecs[bestChild] = bsharedPrimVec;
            bsharedPrimVec->incRef();
          }
          else {
            primvecs[bestChild] = new (&sharedPrimVecs[numSharedPrimVecs++]) SharedPrimRefVector(lrecord.prims.prims);
          }
          
          if (rrecord.prims.prims == bsharedPrimVec->prims) {
            primvecs[numChildren] = bsharedPrimVec;
            bsharedPrimVec->incRef();
          }
          else {
            primvecs[numChildren] = new (&sharedPrimVecs[numSharedPrimVecs++]) SharedPrimRefVector(rrecord.prims.prims);
          }
          bsharedPrimVec->decRef();
          new_vector.release();
          
          children[bestChild] = lrecord;
          children[numChildren] = rrecord;
          numChildren++;
        }
        
      public:
        array_t<BuildRecord,MAX_BRANCHING_FACTOR> children;
        array_t<SharedPrimRefVector*,MAX_BRANCHING_FACTOR> primvecs;
        array_t<SharedPrimRefVector,2*MAX_BRANCHING_FACTOR> sharedPrimVecs;
        size_t numChildren;
        size_t numSharedPrimVecs;
        size_t depth;
      };
    
    template<typename Mesh>
      struct RecalculatePrimRef
      {
        Scene* scene;
        
        __forceinline RecalculatePrimRef (Scene* scene)
          : scene(scene) {}
        
        __forceinline std::pair<PrimRefMB,range<int>> operator() (const PrimRefMB& prim, const BBox1f time_range) const
        {
          const unsigned geomID = prim.geomID();
          const unsigned primID = prim.primID();
          const Mesh* mesh = (Mesh*)scene->get(geomID);
          const LBBox3fa lbounds = mesh->linearBounds(primID, time_range);
          const unsigned num_time_segments = mesh->numTimeSegments();
          const range<int> tbounds = getTimeSegmentRange(time_range, num_time_segments);
          assert(tbounds.size() > 0);
          const PrimRefMB prim2(lbounds, tbounds.size(), num_time_segments, geomID, primID);
          return std::make_pair(prim2, tbounds);
        }
        
        __forceinline std::pair<LBBox3fa,range<int>> linearBounds(const PrimRefMB& prim, const BBox1f time_range) const
        {
          const unsigned geomID = prim.geomID();
          const unsigned primID = prim.primID();
          const Mesh* mesh = (Mesh*)scene->get(geomID);
          const LBBox3fa lbounds = mesh->linearBounds(primID, time_range);
          const unsigned num_time_segments = mesh->numTimeSegments();
          const range<int> tbounds = getTimeSegmentRange(time_range, num_time_segments);
          assert(tbounds.size() > 0);
          return std::make_pair(lbounds, tbounds);
        }
      };
    
    struct BVHMBuilderMSMBlur
    {
      struct Settings
      {
        Settings () 
        : branchingFactor(2), maxDepth(32), logBlockSize(0), minLeafSize(1), maxLeafSize(8), travCost(1.0f), intCost(1.0f), singleLeafTimeSegment(false) {}

        size_t branchingFactor;
        size_t maxDepth;
        size_t logBlockSize;
        size_t minLeafSize;
        size_t maxLeafSize;
        float travCost;
        float intCost;
        bool singleLeafTimeSegment;
      };

      struct BuildRecord3
      {
      public:
	__forceinline BuildRecord3 () {}
        
        __forceinline BuildRecord3 (size_t depth) 
          : depth(depth) {}
        
        __forceinline BuildRecord3 (const SetMB& prims, size_t depth) 
          : depth(depth), prims(prims) {}
        
        __forceinline friend bool operator< (const BuildRecord3& a, const BuildRecord3& b) { return a.prims.size() < b.prims.size(); }
	__forceinline friend bool operator> (const BuildRecord3& a, const BuildRecord3& b) { return a.prims.size() > b.prims.size();  }
        
        
        __forceinline size_t size() const { return this->prims.size(); }
        
      public:
	size_t depth;                     //!< Depth of the root of this subtree.
	SetMB prims;                      //!< The list of primitives.
	BinSplit<NUM_OBJECT_BINS> split;  //!< The best split for the primitives.
      };
      
      template<typename RecalculatePrimRef, 
        typename NodeTy, 
        typename Allocator, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitor>
        
        class GeneralBVHMBBuilder : private Settings
      {
        static const size_t MAX_BRANCHING_FACTOR = 8;        //!< maximal supported BVH branching factor
        static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree of we are that many levels before the maximal tree depth
        static const size_t SINGLE_THREADED_THRESHOLD = 1024;  //!< threshold to switch to single threaded build
        
        typedef BinSplit<NUM_OBJECT_BINS> Split;
        typedef mvector<PrimRefMB>* PrimRefVector;
        typedef SharedVector<mvector<PrimRefMB>> SharedPrimRefVector;
        typedef LocalChildListT<BuildRecord3,MAX_BRANCHING_FACTOR> LocalChildList;
        
      public:
        
        GeneralBVHMBBuilder (MemoryMonitorInterface* device,
                             RecalculatePrimRef& recalculatePrimRef,
                             CreateAllocFunc& createAlloc, 
                             CreateNodeFunc& createNode, 
                             UpdateNodeFunc& updateNode, 
                             CreateLeafFunc& createLeaf,
                             ProgressMonitor& progressMonitor,
                             const Settings& settings)
          : Settings(settings),
          heuristicObjectSplit(),
          heuristicTemporalSplit(device, recalculatePrimRef),
          recalculatePrimRef(recalculatePrimRef), createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
          progressMonitor(progressMonitor)
       {
          if (branchingFactor > MAX_BRANCHING_FACTOR)
            throw_RTCError(RTC_UNKNOWN_ERROR,"bvh_builder: branching factor too large");
        }
        
        __forceinline const Split find(BuildRecord3& current) {
          return find (current.prims,logBlockSize);
        }
        
        /*! finds the best split */
        const Split find(SetMB& set, const size_t logBlockSize)
        {
          /* first try standard object split */
          const Split object_split = heuristicObjectSplit.find(set,logBlockSize);
          const float object_split_sah = object_split.splitSAH();
          
          /* do temporal splits only if the the time range is big enough */
          if (set.time_range.size() > 1.01f/float(set.max_num_time_segments)) // FIXME: test temporal splits only when object split was bad
          {
            const Split temporal_split = heuristicTemporalSplit.find(set, logBlockSize);
            const float temporal_split_sah = temporal_split.splitSAH();
            
            /* take temporal split if it improved SAH */
            if (temporal_split_sah < object_split_sah)
              return temporal_split;
          }
          
          return object_split;
        }
        
        /*! array partitioning */
        __forceinline std::unique_ptr<mvector<PrimRefMB>> partition(BuildRecord3& brecord, BuildRecord3& lrecord, BuildRecord3& rrecord) 
        {
          /* perform fallback split */
          //if (unlikely(!brecord.split.valid())) {
          if (unlikely(brecord.split.data == Split::SPLIT_FALLBACK)) {
            deterministic_order(brecord.prims);
            splitFallback(brecord.prims,lrecord.prims,rrecord.prims);
            return nullptr;
          }
          /* perform temporal split */
          else if (unlikely(brecord.split.data == Split::SPLIT_TEMPORAL)) {
            return heuristicTemporalSplit.split(brecord.split,brecord.prims,lrecord.prims,rrecord.prims);
          }
          /* perform object split */
          else {
            heuristicObjectSplit.split(brecord.split,brecord.prims,lrecord.prims,rrecord.prims);
            return nullptr;
          }
        }
        
        /*! finds the best fallback split */
        __forceinline Split findFallback(BuildRecord3& current, bool singleLeafTimeSegment) // FIXME: remove singleLeafTimeSegment parameter
        {
          /* if a leaf can only hold a single time-segment, we might have to do additional temporal splits */
          if (singleLeafTimeSegment)
          {
            /* test if one primitive has more than one time segment in time range, if so split time */
            for (size_t i=current.prims.object_range.begin(); i<current.prims.object_range.end(); i++) 
            {
              const PrimRefMB& prim = (*current.prims.prims)[i];
              const range<int> itime_range = getTimeSegmentRange(current.prims.time_range,prim.totalTimeSegments());
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
        
        void splitFallback(const SetMB& set, SetMB& lset, SetMB& rset) // FIXME: also perform time split here?
        {
          mvector<PrimRefMB>& prims = *set.prims;
          
          const size_t begin = set.object_range.begin();
          const size_t end   = set.object_range.end();
          const size_t center = (begin + end)/2;
          
          PrimInfoMB linfo = empty;
          for (size_t i=begin; i<center; i++)
            linfo.add_primref(prims[i]);
          
          PrimInfoMB rinfo = empty;
          for (size_t i=center; i<end; i++)
            rinfo.add_primref(prims[i]);	
          
          new (&lset) SetMB(linfo,set.prims,range<size_t>(begin,center),set.time_range);
          new (&rset) SetMB(rinfo,set.prims,range<size_t>(center,end  ),set.time_range);
        }
        
        void deterministic_order(const SetMB& set) 
        {
          /* required as parallel partition destroys original primitive order */
          PrimRefMB* prims = set.prims->data();
          std::sort(&prims[set.object_range.begin()],&prims[set.object_range.end()]);
        }
        
        const std::tuple<NodeTy,LBBox3fa,BBox1f> createLargeLeaf(BuildRecord3& current, Allocator alloc)
        {
          /* this should never occur but is a fatal error */
          if (current.depth > maxDepth) 
            throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
          
          /* replace already found split by fallback split */
          current.split = findFallback(current,singleLeafTimeSegment);
          
          /* create leaf for few primitives */
          if (current.prims.size() <= maxLeafSize && current.split.data != Split::SPLIT_TEMPORAL)
            return createLeaf(current,alloc);
          
          /* fill all children by always splitting the largest one */
          std::tuple<NodeTy,LBBox3fa,BBox1f> values[MAX_BRANCHING_FACTOR];
          LocalChildList children(current);
          
          do {  
            /* find best child with largest bounding box area */
            size_t bestChild = -1;
            size_t bestSize = 0;
            for (size_t i=0; i<children.size(); i++)
            {
              /* ignore leaves as they cannot get split */
              if (children[i].prims.size() <= maxLeafSize && children[i].split.data != Split::SPLIT_TEMPORAL)
                continue;
              
              /* remember child with largest size */
              if (children[i].prims.size() > bestSize) {
                bestSize = children[i].prims.size();
                bestChild = i;
              }
            }
            if (bestChild == -1) break;
            
            /* perform best found split */
            BuildRecord3& brecord = children[bestChild];
            BuildRecord3 lrecord(current.depth+1);
            BuildRecord3 rrecord(current.depth+1);
            std::unique_ptr<mvector<PrimRefMB>> new_vector = partition(brecord,lrecord,rrecord);
            
            /* find new splits */
            lrecord.split = findFallback(lrecord,singleLeafTimeSegment);
            rrecord.split = findFallback(rrecord,singleLeafTimeSegment);
            children.split(bestChild,lrecord,rrecord,std::move(new_vector));
            
          } while (children.size() < branchingFactor);
          
          /* check if we did some time split */
          bool hasTimeSplits = false;
          for (size_t i=0; i<children.size() && !hasTimeSplits; i++)
            hasTimeSplits |= current.prims.time_range != children[i].prims.time_range;
          
          /* create node */
          auto node = createNode(hasTimeSplits,alloc);
          
          /* recurse into each child  and perform reduction */
          for (size_t i=0; i<children.size(); i++) {
            values[i] = createLargeLeaf(children[i],alloc);
            updateNode(node,i,values[i]);
          }
          
          /* calculate geometry bounds and time bounds of this node */
          LBBox3fa gbounds = empty;
          BBox1f tbounds = empty;
          if (!hasTimeSplits)
          {
            for (size_t i=0; i<children.size(); i++)
              gbounds.extend(std::get<1>(values[i]));
            tbounds = std::get<2>(values[0]);
          }
          else
          {
            gbounds = current.prims.linearBounds(recalculatePrimRef);
            tbounds = current.prims.time_range;
          }
          return std::make_tuple(node,gbounds,tbounds);
        }
        
        const std::tuple<NodeTy,LBBox3fa,BBox1f> recurse(BuildRecord3& current, Allocator alloc, bool toplevel)
        {
          if (alloc == nullptr)
            alloc = createAlloc();
          
          /* call memory monitor function to signal progress */
          if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
            progressMonitor(current.size());
          
          /*! compute leaf and split cost */
          const float leafSAH  = intCost*current.prims.leafSAH(logBlockSize);
          const float splitSAH = travCost*current.prims.halfArea()+intCost*current.split.splitSAH();
          assert((current.prims.size() == 0) || ((leafSAH >= 0) && (splitSAH >= 0)));
          assert(current.prims.size() == current.prims.object_range.size());
          
          /*! create a leaf node when threshold reached or SAH tells us to stop */
          if (current.prims.size() <= minLeafSize || current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || (current.prims.size() <= maxLeafSize && leafSAH <= splitSAH)) {
            deterministic_order(current.prims);
            return createLargeLeaf(current,alloc);
          }
          
          /*! initialize child list */
          std::tuple<NodeTy,LBBox3fa,BBox1f> values[MAX_BRANCHING_FACTOR];
          LocalChildList children(current);
          
          /*! split until node is full or SAH tells us to stop */
          do {
            /*! find best child to split */
            float bestSAH = neg_inf;
            ssize_t bestChild = -1;
            for (size_t i=0; i<children.size(); i++) 
            {
              if (children[i].prims.size() <= minLeafSize) continue;
              if (expectedApproxHalfArea(children[i].prims.geomBounds) > bestSAH) {
                bestChild = i; bestSAH = expectedApproxHalfArea(children[i].prims.geomBounds);
              } 
            }
            if (bestChild == -1) break;
            
            /* perform best found split */
            BuildRecord3& brecord = children[bestChild];
            BuildRecord3 lrecord(current.depth+1);
            BuildRecord3 rrecord(current.depth+1);
            std::unique_ptr<mvector<PrimRefMB>> new_vector = partition(brecord,lrecord,rrecord);            
            
            /* find new splits */
            lrecord.split = find(lrecord);
            rrecord.split = find(rrecord);
            children.split(bestChild,lrecord,rrecord,std::move(new_vector));
            
          } while (children.size() < branchingFactor);
          
          /* sort buildrecords for simpler shadow ray traversal */
          //std::sort(&children[0],&children[children.size()],std::greater<BuildRecord3>()); // FIXME: reduces traversal performance of bvh8.triangle4 (need to verified) !!
          
          /* check if we did some time split */
          bool hasTimeSplits = false;
          for (size_t i=0; i<children.size() && !hasTimeSplits; i++)
            hasTimeSplits |= current.prims.time_range != children[i].prims.time_range;
          
          /*! create an inner node */
          auto node = createNode(hasTimeSplits,alloc);
          
          /* spawn tasks */
          if (current.size() > SINGLE_THREADED_THRESHOLD) 
          {
            /*! parallel_for is faster than spawing sub-tasks */
            parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  values[i] = recurse(children[i],nullptr,true); 
                  updateNode(node,i,values[i]);
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
          }
          /* recurse into each child */
          else 
          {
            //for (size_t i=0; i<children.size(); i++)
            for (ssize_t i=children.size()-1; i>=0; i--) {
              values[i] = recurse(children[i],alloc,false);
              updateNode(node,i,values[i]);
            }
          }
          
          /* calculate geometry bounds and time bounds of this node */
          LBBox3fa gbounds = empty;
          BBox1f tbounds = empty;
          if (!hasTimeSplits)
          {
            for (size_t i=0; i<children.size(); i++)
              gbounds.extend(std::get<1>(values[i]));
            tbounds = std::get<2>(values[0]);
          }
          else
          {
            gbounds = current.prims.linearBounds(recalculatePrimRef);
            tbounds = current.prims.time_range;
          }
          return std::make_tuple(node,gbounds,tbounds);
        }
        
        /*! builder entry function */
        __forceinline const std::tuple<NodeTy,LBBox3fa,BBox1f> operator() (BuildRecord3& record)
        {
          record.split = find(record); 
          auto ret = recurse(record,nullptr,true);
          _mm_mfence(); // to allow non-temporal stores during build
          return ret;
        }
        
      private:
        HeuristicArrayBinningMB<PrimRefMB,NUM_OBJECT_BINS> heuristicObjectSplit;
        HeuristicMBlurTemporalSplit<PrimRefMB,RecalculatePrimRef,NUM_TEMPORAL_BINS> heuristicTemporalSplit;
        RecalculatePrimRef& recalculatePrimRef;
        CreateAllocFunc& createAlloc;
        CreateNodeFunc& createNode;
        UpdateNodeFunc& updateNode;
        CreateLeafFunc& createLeaf;
        ProgressMonitor& progressMonitor;
      };
      
      template<typename NodeTy, 
        typename RecalculatePrimRef, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitor>
        
        static const std::tuple<NodeTy,LBBox3fa,BBox1f> build(BuildRecord3& record,
                                                                   MemoryMonitorInterface* device,
                                                                   RecalculatePrimRef& recalculatePrimRef,
                                                                   CreateAllocFunc& createAlloc, 
                                                                   CreateNodeFunc& createNode, 
                                                                   UpdateNodeFunc& updateNode, 
                                                                   CreateLeafFunc& createLeaf,
                                                                   ProgressMonitor& progressMonitor,
                                                                   const Settings& settings)
      {
        typedef GeneralBVHMBBuilder<
          RecalculatePrimRef,
          NodeTy,
          decltype(createAlloc()),
          decltype(createAlloc),
          decltype(createNode),
          decltype(updateNode),
          decltype(createLeaf),
          decltype(progressMonitor)> Builder;
        
        /* instantiate builder */
        Builder builder(device,
                        recalculatePrimRef,
                        createAlloc,
                        createNode,
                        updateNode,
                        createLeaf,
                        progressMonitor,
                        settings);
        
        return builder(record);
      }
    };
  }
}

