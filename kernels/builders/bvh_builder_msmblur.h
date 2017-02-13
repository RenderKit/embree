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
        
        __forceinline LocalChildListT (const BuildRecord& record)
          : numChildren(1), numSharedPrimVecs(1)
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
        
        __forceinline void split(int bestChild, const BuildRecord& lrecord, const BuildRecord& rrecord, std::unique_ptr<mvector<PrimRefMB>> new_vector)
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
        size_t numChildren;

        array_t<SharedPrimRefVector,2*MAX_BRANCHING_FACTOR> sharedPrimVecs;
        size_t numSharedPrimVecs;
      };
    
    template<typename Mesh>
      struct RecalculatePrimRef
      {
        Scene* scene;
        
        __forceinline RecalculatePrimRef (Scene* scene)
          : scene(scene) {}
        
        __forceinline PrimRefMB operator() (const PrimRefMB& prim, const BBox1f time_range) const
        {
          const unsigned geomID = prim.geomID();
          const unsigned primID = prim.primID();
          const Mesh* mesh = scene->get<Mesh>(geomID);
          const LBBox3fa lbounds = mesh->linearBounds(primID, time_range);
          const unsigned num_time_segments = mesh->numTimeSegments();
          const range<int> tbounds = getTimeSegmentRange(time_range, num_time_segments);
          return PrimRefMB (lbounds, tbounds.size(), num_time_segments, geomID, primID);
        }
        
        __forceinline LBBox3fa linearBounds(const PrimRefMB& prim, const BBox1f time_range) const {
          return scene->get<Mesh>(prim.geomID())->linearBounds(prim.primID(), time_range);
        }
      };
    
    struct BVHMBuilderMSMBlur
    {
      /*! settings for msmblur builder */
      struct Settings
      {
        /*! default settings */
        Settings () 
        : branchingFactor(2), maxDepth(32), logBlockSize(0), minLeafSize(1), maxLeafSize(8), 
          travCost(1.0f), intCost(1.0f), singleLeafTimeSegment(false), 
          singleThreadThreshold(1024) {}

      public:
        size_t branchingFactor;  //!< branching factor of BVH to build
        size_t maxDepth;         //!< maximal depth of BVH to build
        size_t logBlockSize;     //!< log2 of blocksize for SAH heuristic
        size_t minLeafSize;      //!< minimal size of a leaf
        size_t maxLeafSize;      //!< maximal size of a leaf
        float travCost;          //!< estimated cost of one traversal step
        float intCost;           //!< estimated cost of one primitive intersection
        bool singleLeafTimeSegment; //!< split time to single time range
        size_t singleThreadThreshold; //!< threshold when we switch to single threaded build
      };

      struct BuildRecord
      {
      public:
	__forceinline BuildRecord () {}
        
        __forceinline BuildRecord (size_t depth)  // FIXME: remove
          : depth(depth) {}
        
        __forceinline BuildRecord (const SetMB& prims, size_t depth)  // FIXME: remoe
          : depth(depth), prims(prims) {}

        __forceinline BuildRecord (const SetMB& prims, const BinSplit<NUM_OBJECT_BINS>& split, size_t depth) 
          : depth(depth), prims(prims), split(split) {}
        
        __forceinline friend bool operator< (const BuildRecord& a, const BuildRecord& b) { return a.prims.size() < b.prims.size(); }
	__forceinline friend bool operator> (const BuildRecord& a, const BuildRecord& b) { return a.prims.size() > b.prims.size();  }
                
        __forceinline size_t size() const { return this->prims.size(); }
        
      public:
	size_t depth;                     //!< Depth of the root of this subtree.
	SetMB prims;                      //!< The list of primitives.
	BinSplit<NUM_OBJECT_BINS> split;  //!< The best split for the primitives.
      };
      
      template<
        typename NodeTy, 
        typename RecalculatePrimRef,         
        typename Allocator, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitor>
        
        class BuilderT : private Settings
      {
        static const size_t MAX_BRANCHING_FACTOR = 8;        //!< maximal supported BVH branching factor
        static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree if we are that many levels before the maximal tree depth
        
        typedef BinSplit<NUM_OBJECT_BINS> Split;
        typedef mvector<PrimRefMB>* PrimRefVector;
        typedef SharedVector<mvector<PrimRefMB>> SharedPrimRefVector;
        typedef LocalChildListT<BuildRecord,MAX_BRANCHING_FACTOR> LocalChildList;
        
      public:
        
        BuilderT (MemoryMonitorInterface* device,
                  const RecalculatePrimRef recalculatePrimRef,
                  const CreateAllocFunc createAlloc, 
                  const CreateNodeFunc createNode, 
                  const UpdateNodeFunc updateNode, 
                  const CreateLeafFunc createLeaf,
                  const ProgressMonitor progressMonitor,
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
        
        /*! finds the best split */
        const Split find(const SetMB& set)
        {
          /* first try standard object split */
          const Split object_split = heuristicObjectSplit.find(set,logBlockSize);
          const float object_split_sah = object_split.splitSAH();

          /* test temporal splits only when object split was bad */
          const float leaf_sah = set.leafSAH(logBlockSize);
          if (object_split_sah < 0.50f*leaf_sah)
            return object_split;
          
          /* do temporal splits only if the the time range is big enough */
          if (set.time_range.size() > 1.01f/float(set.max_num_time_segments))
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
        __forceinline std::unique_ptr<mvector<PrimRefMB>> partition(const BuildRecord& brecord, BuildRecord& lrecord, BuildRecord& rrecord) 
        {
          /* perform fallback split */
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
        __forceinline Split findFallback(const SetMB& set)
        {
          /* if a leaf can only hold a single time-segment, we might have to do additional temporal splits */
          if (singleLeafTimeSegment)
          {
            /* test if one primitive has more than one time segment in time range, if so split time */
            for (size_t i=set.object_range.begin(); i<set.object_range.end(); i++) 
            {
              const PrimRefMB& prim = (*set.prims)[i];
              const range<int> itime_range = getTimeSegmentRange(set.time_range,prim.totalTimeSegments());
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
        
        /* required as parallel partition destroys original primitive order */
        void deterministic_order(const SetMB& set) 
        {
          PrimRefMB* prims = set.prims->data();
          std::sort(&prims[set.object_range.begin()],&prims[set.object_range.end()]);
        }
        
        const std::tuple<NodeTy,LBBox3fa,BBox1f> createLargeLeaf(const BuildRecord& in, Allocator alloc)
        {
          /* this should never occur but is a fatal error */
          if (in.depth > maxDepth) 
            throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
          
          /* replace already found split by fallback split */
          const BuildRecord current(in.prims,findFallback(in.prims),in.depth);
          
          /* create leaf for few primitives */
          if (current.size() <= maxLeafSize && current.split.data != Split::SPLIT_TEMPORAL)
            return createLeaf(current,alloc);
          
          /* fill all children by always splitting the largest one */
          bool hasTimeSplits = false;
          std::tuple<NodeTy,LBBox3fa,BBox1f> values[MAX_BRANCHING_FACTOR];
          LocalChildList children(current);
          
          do {  
            /* find best child with largest bounding box area */
            size_t bestChild = -1;
            size_t bestSize = 0;
            for (size_t i=0; i<children.size(); i++)
            {
              /* ignore leaves as they cannot get split */
              if (children[i].size() <= maxLeafSize && children[i].split.data != Split::SPLIT_TEMPORAL)
                continue;
              
              /* remember child with largest size */
              if (children[i].size() > bestSize) {
                bestSize = children[i].size();
                bestChild = i;
              }
            }
            if (bestChild == -1) break;
            
            /* perform best found split */
            BuildRecord& brecord = children[bestChild];
            BuildRecord lrecord(current.depth+1);
            BuildRecord rrecord(current.depth+1);
            std::unique_ptr<mvector<PrimRefMB>> new_vector = partition(brecord,lrecord,rrecord);
            hasTimeSplits |= new_vector != nullptr;
            
            /* find new splits */
            lrecord.split = findFallback(lrecord.prims);
            rrecord.split = findFallback(rrecord.prims);
            children.split(bestChild,lrecord,rrecord,std::move(new_vector));
            
          } while (children.size() < branchingFactor);
          
          /* create node */
          auto node = createNode(hasTimeSplits,alloc);
          
          /* recurse into each child and perform reduction */
          LBBox3fa gbounds = empty;
          for (size_t i=0; i<children.size(); i++) {
            values[i] = createLargeLeaf(children[i],alloc);
            gbounds.extend(std::get<1>(values[i]));
            updateNode(node,i,values[i]);
          }

          /* calculate geometry bounds of this node */
          if (hasTimeSplits)
            return std::make_tuple(node,current.prims.linearBounds(recalculatePrimRef),current.prims.time_range);
          else
            return std::make_tuple(node,gbounds,current.prims.time_range);
        }
        
        const std::tuple<NodeTy,LBBox3fa,BBox1f> recurse(const BuildRecord& current, Allocator alloc, bool toplevel)
        {
          if (alloc == nullptr)
            alloc = createAlloc();
          
          /* call memory monitor function to signal progress */
          if (toplevel && current.size() <= singleThreadThreshold)
            progressMonitor(current.size());
          
          /*! compute leaf and split cost */
          const float leafSAH  = intCost*current.prims.leafSAH(logBlockSize);
          const float splitSAH = travCost*current.prims.halfArea()+intCost*current.split.splitSAH();
          assert((current.size() == 0) || ((leafSAH >= 0) && (splitSAH >= 0)));
          
          /*! create a leaf node when threshold reached or SAH tells us to stop */
          if (current.size() <= minLeafSize || current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || (current.size() <= maxLeafSize && leafSAH <= splitSAH)) {
            deterministic_order(current.prims);
            return createLargeLeaf(current,alloc);
          }
          
          /*! initialize child list */
          bool hasTimeSplits = false;
          std::tuple<NodeTy,LBBox3fa,BBox1f> values[MAX_BRANCHING_FACTOR];
          LocalChildList children(current);
          
          /*! split until node is full or SAH tells us to stop */
          do {
            /*! find best child to split */
            float bestSAH = neg_inf;
            ssize_t bestChild = -1;
            for (size_t i=0; i<children.size(); i++) 
            {
              if (children[i].size() <= minLeafSize) continue;
              if (expectedApproxHalfArea(children[i].prims.geomBounds) > bestSAH) {
                bestChild = i; bestSAH = expectedApproxHalfArea(children[i].prims.geomBounds);
              } 
            }
            if (bestChild == -1) break;
            
            /* perform best found split */
            BuildRecord& brecord = children[bestChild];
            BuildRecord lrecord(current.depth+1);
            BuildRecord rrecord(current.depth+1);
            std::unique_ptr<mvector<PrimRefMB>> new_vector = partition(brecord,lrecord,rrecord);            
            hasTimeSplits |= new_vector != nullptr;
            
            /* find new splits */
            lrecord.split = find(lrecord.prims);
            rrecord.split = find(rrecord.prims);
            children.split(bestChild,lrecord,rrecord,std::move(new_vector));
            
          } while (children.size() < branchingFactor);
          
          /* sort buildrecords for simpler shadow ray traversal */
          //std::sort(&children[0],&children[children.size()],std::greater<BuildRecord>()); // FIXME: reduces traversal performance of bvh8.triangle4 (need to verified) !!
          
          /*! create an inner node */
          auto node = createNode(hasTimeSplits,alloc);
          LBBox3fa gbounds = empty;
          
          /* spawn tasks */
          if (current.size() > singleThreadThreshold) 
          {
            /*! parallel_for is faster than spawing sub-tasks */
            parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  values[i] = recurse(children[i],nullptr,true); 
                  updateNode(node,i,values[i]);
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });

            
            for (size_t i=0; i<children.size(); i++)
              gbounds.extend(std::get<1>(values[i]));
          }
          /* recurse into each child */
          else 
          {
            //for (size_t i=0; i<children.size(); i++)
            for (ssize_t i=children.size()-1; i>=0; i--) {
              values[i] = recurse(children[i],alloc,false);
              gbounds.extend(std::get<1>(values[i]));
              updateNode(node,i,values[i]);
            }
          }
          
          /* calculate geometry bounds of this node */
          if (hasTimeSplits)
            return std::make_tuple(node,current.prims.linearBounds(recalculatePrimRef),current.prims.time_range);
          else
            return std::make_tuple(node,gbounds,current.prims.time_range);
        }
        
        /*! builder entry function */
        __forceinline const std::tuple<NodeTy,LBBox3fa,BBox1f> operator() (mvector<PrimRefMB>& prims, const PrimInfoMB& pinfo)
        {
          const SetMB set(pinfo,&prims);
          const BuildRecord record(set,find(set),1);
          auto ret = recurse(record,nullptr,true);
          _mm_mfence(); // to allow non-temporal stores during build
          return ret;
        }
        
      private:
        HeuristicArrayBinningMB<PrimRefMB,NUM_OBJECT_BINS> heuristicObjectSplit;
        HeuristicMBlurTemporalSplit<PrimRefMB,RecalculatePrimRef,NUM_TEMPORAL_BINS> heuristicTemporalSplit;
        const RecalculatePrimRef recalculatePrimRef;
        const CreateAllocFunc createAlloc;
        const CreateNodeFunc createNode;
        const UpdateNodeFunc updateNode;
        const CreateLeafFunc createLeaf;
        const ProgressMonitor progressMonitor;
      };
      
      template<typename NodeTy, 
        typename RecalculatePrimRef, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitorFunc>
        
        static const std::tuple<NodeTy,LBBox3fa,BBox1f> build(mvector<PrimRefMB>& prims,
                                                              PrimInfoMB pinfo,
                                                              MemoryMonitorInterface* device,
                                                              const RecalculatePrimRef recalculatePrimRef,
                                                              const CreateAllocFunc createAlloc, 
                                                              const CreateNodeFunc createNode, 
                                                              const UpdateNodeFunc updateNode, 
                                                              const CreateLeafFunc createLeaf,
                                                              const ProgressMonitorFunc progressMonitor,
                                                              const Settings& settings)
      {
        typedef BuilderT<
          NodeTy,
          RecalculatePrimRef,
          decltype(createAlloc()),
          CreateAllocFunc,
          CreateNodeFunc,
          UpdateNodeFunc,
          CreateLeafFunc,
          ProgressMonitorFunc> Builder;
        
        Builder builder(device,
                        recalculatePrimRef,
                        createAlloc,
                        createNode,
                        updateNode,
                        createLeaf,
                        progressMonitor,
                        settings);
        
        
        return builder(prims,pinfo);
      }
    };
  }
}

