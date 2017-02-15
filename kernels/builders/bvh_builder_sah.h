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

#include "heuristic_binning_array_aligned.h"
#include "heuristic_spatial_array.h"

#if defined(__AVX512F__)
#  define NUM_OBJECT_BINS 16
#  define NUM_SPATIAL_BINS 16
#else
#  define NUM_OBJECT_BINS 32
#  define NUM_SPATIAL_BINS 16
#endif

namespace embree
{
  namespace isa
  {  
    struct GeneralBVHBuilder
    {
      /*! settings for msmblur builder */
      struct Settings
      {
        /*! default settings */
        Settings () 
        : branchingFactor(2), maxDepth(32), logBlockSize(0), minLeafSize(1), maxLeafSize(8), 
          travCost(1.0f), intCost(1.0f), singleThreadThreshold(1024) {}
        
        Settings (size_t sahBlockSize, size_t minLeafSize, size_t maxLeafSize, float travCost, float intCost, size_t singleThreadThreshold)
        : branchingFactor(2), maxDepth(32), logBlockSize(__bsr(sahBlockSize)), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), 
          travCost(travCost), intCost(intCost), singleThreadThreshold(singleThreadThreshold) {}
        
      public:
        size_t branchingFactor;  //!< branching factor of BVH to build
        size_t maxDepth;         //!< maximal depth of BVH to build
        size_t logBlockSize;     //!< log2 of blocksize for SAH heuristic
        size_t minLeafSize;      //!< minimal size of a leaf
        size_t maxLeafSize;      //!< maximal size of a leaf
        float travCost;          //!< estimated cost of one traversal step
        float intCost;           //!< estimated cost of one primitive intersection
        size_t singleThreadThreshold; //!< threshold when we switch to single threaded build
      };
      
      template<typename Set, typename Split, typename PrimInfo>
        struct BuildRecordT 
        {
        public:
          __forceinline BuildRecordT () {}
          
          __forceinline BuildRecordT (size_t depth) 
            : depth(depth), pinfo(empty) {}
          
          __forceinline BuildRecordT (const PrimInfo& pinfo, size_t depth) 
            : depth(depth), pinfo(pinfo) {}
          
          __forceinline BuildRecordT (const PrimInfo& pinfo, size_t depth, const Set &prims) 
            : depth(depth), prims(prims), pinfo(pinfo) {}
          
          __forceinline BBox3fa bounds() const { return pinfo.geomBounds; }
          
          __forceinline friend bool operator< (const BuildRecordT& a, const BuildRecordT& b) { return a.pinfo.size() < b.pinfo.size(); }
          __forceinline friend bool operator> (const BuildRecordT& a, const BuildRecordT& b) { return a.pinfo.size() > b.pinfo.size();  }
          
          
          __forceinline size_t size() const { return this->pinfo.size(); }
          
        public:
          size_t depth;     //!< Depth of the root of this subtree.
          Set prims;        //!< The list of primitives.
          PrimInfo pinfo;   //!< Bounding info of primitives.
          Split split;      //!< The best split for the primitives.
        };
      
      template<typename BuildRecord, 
        typename Heuristic, 
        typename ReductionTy, 
        typename Allocator, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitor,
        typename PrimInfo>
        
        class BuilderT : private Settings
      {
        static const size_t MAX_BRANCHING_FACTOR = 8;        //!< maximal supported BVH branching factor
        static const size_t MIN_LARGE_LEAF_LEVELS = 8;        //!< create balanced tree of we are that many levels before the maximal tree depth
        
      public:
        
        BuilderT (Heuristic& heuristic, 
                  CreateAllocFunc& createAlloc, 
                  CreateNodeFunc& createNode, 
                  UpdateNodeFunc& updateNode, 
                  const CreateLeafFunc& createLeaf,
                  const ProgressMonitor& progressMonitor,
                  const PrimInfo& pinfo,
                  const Settings& settings)
          
          : Settings(settings),
          heuristic(heuristic), 
          createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
          progressMonitor(progressMonitor),
          pinfo(pinfo)
          {
            if (branchingFactor > MAX_BRANCHING_FACTOR)
              throw_RTCError(RTC_UNKNOWN_ERROR,"bvh_builder: branching factor too large");
          }
        
        const ReductionTy createLargeLeaf(BuildRecord& current, Allocator alloc)
        {
          /* this should never occur but is a fatal error */
          if (current.depth > maxDepth) 
            throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
          
          /* create leaf for few primitives */
          if (current.pinfo.size() <= maxLeafSize)
            return createLeaf(current,alloc);
          
          /* fill all children by always splitting the largest one */
          ReductionTy values[MAX_BRANCHING_FACTOR];
          BuildRecord children[MAX_BRANCHING_FACTOR];
          size_t numChildren = 1;
          children[0] = current;
          do {
            
            /* find best child with largest bounding box area */
            size_t bestChild = -1;
            size_t bestSize = 0;
            for (size_t i=0; i<numChildren; i++)
            {
              /* ignore leaves as they cannot get split */
              if (children[i].pinfo.size() <= maxLeafSize)
                continue;
              
              /* remember child with largest size */
              if (children[i].pinfo.size() > bestSize) { 
                bestSize = children[i].pinfo.size();
                bestChild = i;
              }
            }
            if (bestChild == (size_t)-1) break;
            
            /*! split best child into left and right child */
            BuildRecord left(current.depth+1);
            BuildRecord right(current.depth+1);
            heuristic.splitFallback(children[bestChild].prims,left.pinfo,left.prims,right.pinfo,right.prims);
            left .split = find(left );
            right.split = find(right);
            
            /* add new children left and right */
            children[bestChild] = children[numChildren-1];
            children[numChildren-1] = left;
            children[numChildren+0] = right;
            numChildren++;
            
          } while (numChildren < branchingFactor);
          
          /* create node */
          auto node = createNode(current,children,numChildren,alloc);
          
          /* recurse into each child  and perform reduction */
          for (size_t i=0; i<numChildren; i++)
            values[i] = createLargeLeaf(children[i],alloc);
          
          /* perform reduction */
          return updateNode(node,values,numChildren);
        }
        
        __forceinline const typename Heuristic::Split find(BuildRecord& current) {
          return heuristic.find (current.prims,current.pinfo,logBlockSize);
        }
        
        __forceinline void partition(BuildRecord& brecord, BuildRecord& lrecord, BuildRecord& rrecord) {
          heuristic.split(brecord.split,brecord.pinfo,brecord.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
        }
        
        const ReductionTy recurse(BuildRecord& current, Allocator alloc, bool toplevel)
        {
          if (alloc == nullptr)
            alloc = createAlloc();
          
          /* call memory monitor function to signal progress */
          if (toplevel && current.size() <= singleThreadThreshold)
            progressMonitor(current.size());
          
          /*! compute leaf and split cost */
          const float leafSAH  = intCost*current.pinfo.leafSAH(logBlockSize);
          const float splitSAH = travCost*halfArea(current.pinfo.geomBounds)+intCost*current.split.splitSAH();
          assert((current.pinfo.size() == 0) || ((leafSAH >= 0) && (splitSAH >= 0)));
          
          /*! create a leaf node when threshold reached or SAH tells us to stop */
          if (current.pinfo.size() <= minLeafSize || current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || (current.pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
            heuristic.deterministic_order(current.prims);
            return createLargeLeaf(current,alloc);
          }
          
          /*! initialize child list */
          ReductionTy values[MAX_BRANCHING_FACTOR];
          BuildRecord children[MAX_BRANCHING_FACTOR];
          children[0] = current;
          size_t numChildren = 1;
          
          /*! split until node is full or SAH tells us to stop */
          do {
            
            /*! find best child to split */
#if 1
            float bestSAH = neg_inf;
            ssize_t bestChild = -1;
            for (size_t i=0; i<numChildren; i++) 
            {
              if (children[i].pinfo.size() <= minLeafSize) continue; 
              if (area(children[i].pinfo.geomBounds) > bestSAH) { bestChild = i; bestSAH = area(children[i].pinfo.geomBounds); } // FIXME: measure over all scenes if this line creates better tree
            }
#else
            float bestSAH = 0;
            ssize_t bestChild = -1;
            for (size_t i=0; i<numChildren; i++) 
            {
              float dSAH = children[i].split.splitSAH()-children[i].pinfo.leafSAH(logBlockSize);
              if (children[i].pinfo.size() <= minLeafSize) continue; 
              if (children[i].pinfo.size() > maxLeafSize) dSAH = min(dSAH,0.0f); //< force split for large jobs
              if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
            }
            
#endif
            
            if (bestChild == -1) break;
            
            
            /* perform best found split */
            BuildRecord& brecord = children[bestChild];
            BuildRecord lrecord(current.depth+1);
            BuildRecord rrecord(current.depth+1);
            partition(brecord,lrecord,rrecord);
            
            /* find new splits */
            lrecord.split = find(lrecord);
            rrecord.split = find(rrecord);
            children[bestChild  ] = lrecord;
            children[numChildren] = rrecord;
            numChildren++;
            
          } while (numChildren < branchingFactor);
          
          /* sort buildrecords for simpler shadow ray traversal */
          std::sort(&children[0],&children[numChildren],std::greater<BuildRecord>()); // FIXME: reduces traversal performance of bvh8.triangle4 (need to verified) !!
          
          /*! create an inner node */
          auto node = createNode(current,children,numChildren,alloc);
          
          /* spawn tasks */
          if (current.size() > singleThreadThreshold) 
          {
            /*! parallel_for is faster than spawing sub-tasks */
            parallel_for(size_t(0), numChildren, [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  values[i] = recurse(children[i],nullptr,true); 
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
            /* perform reduction */
            return updateNode(node,values,numChildren);
          }
          /* recurse into each child */
          else 
          {
            //for (size_t i=0; i<numChildren; i++)
            for (ssize_t i=numChildren-1; i>=0; i--)
              values[i] = recurse(children[i],alloc,false);
            
            /* perform reduction */
            return updateNode(node,values,numChildren);
          }
        }
        
        /*! builder entry function */
        __forceinline const ReductionTy operator() (BuildRecord& record)
        {
          record.split = find(record); 
          ReductionTy ret = recurse(record,nullptr,true);
          _mm_mfence(); // to allow non-temporal stores during build
          return ret;
        }
        
      private:
        Heuristic& heuristic;
        CreateAllocFunc& createAlloc;
        CreateNodeFunc& createNode;
        UpdateNodeFunc& updateNode;
        const CreateLeafFunc& createLeaf;
        const ProgressMonitor& progressMonitor;
        const PrimInfo& pinfo;
      };
      
      template<
      typename ReductionTy, 
        typename Heuristic,
        typename Set,
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitor>
        
        static ReductionTy build(Heuristic& heuristic,
                                 const Set& set,
                                 CreateAllocFunc createAlloc, 
                                 CreateNodeFunc createNode, UpdateNodeFunc updateNode, 
                                 const CreateLeafFunc& createLeaf, 
                                 const ProgressMonitor& progressMonitor,
                                 const PrimInfo& pinfo, 
                                 const Settings& settings)
      {
        typedef BuildRecordT<Set,typename Heuristic::Split,PrimInfo> BuildRecord;
        
        typedef BuilderT<
          BuildRecord,
          Heuristic,
          ReductionTy,
          decltype(createAlloc()),
          CreateAllocFunc,
          CreateNodeFunc,
          UpdateNodeFunc,
          CreateLeafFunc,
          ProgressMonitor,
          PrimInfo> Builder;
        
        /* instantiate builder */
        Builder builder(heuristic,
                        createAlloc,
                        createNode,
                        updateNode,
                        createLeaf,
                        progressMonitor,
                        pinfo,
                        settings);
        
        /* build hierarchy */
        BuildRecord br(pinfo,1,set);
        return builder(br);
      }
    };
    
    /* SAH builder that operates on an array of BuildRecords */
    struct BVHBuilderBinnedSAH
    {
      typedef range<size_t> Set;
      typedef HeuristicArrayBinningSAH<PrimRef,NUM_OBJECT_BINS> Heuristic;
      typedef GeneralBVHBuilder::BuildRecordT<Set,typename Heuristic::Split,PrimInfo> BuildRecord;
      typedef typename GeneralBVHBuilder::Settings Settings;
      
      /*! special builder that propagates reduction over the tree */
      template<
      typename ReductionTy, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename ProgressMonitor>
        
        static ReductionTy build(CreateAllocFunc createAlloc, 
                                 CreateNodeFunc createNode, UpdateNodeFunc updateNode, 
                                 const CreateLeafFunc& createLeaf, 
                                 const ProgressMonitor& progressMonitor,
                                 PrimRef* prims, const PrimInfo& pinfo, 
                                 const Settings& settings)
      {
        Heuristic heuristic(prims);
        return GeneralBVHBuilder::build<ReductionTy,Heuristic,Set>(
          heuristic,
          Set(0,pinfo.size()),
          createAlloc,
          createNode,
          updateNode,
          createLeaf,
          progressMonitor,
          pinfo,
          settings);
      }
    };
    
    /* Spatial SAH builder that operates on an double-buffered array of BuildRecords */
    struct BVHBuilderBinnedFastSpatialSAH
    {
      typedef extended_range<size_t> Set;
      typedef Split2<BinSplit<NUM_OBJECT_BINS>,SpatialBinSplit<NUM_SPATIAL_BINS> > Split;
      typedef GeneralBVHBuilder::BuildRecordT<Set,Split,PrimInfo> BuildRecord;
      typedef typename GeneralBVHBuilder::Settings Settings;
      
      /*! special builder that propagates reduction over the tree */
      template<
      typename ReductionTy, 
        typename CreateAllocFunc, 
        typename CreateNodeFunc, 
        typename UpdateNodeFunc, 
        typename CreateLeafFunc, 
        typename SplitPrimitiveFunc, 
        typename ProgressMonitor>
        
        static ReductionTy build(CreateAllocFunc createAlloc, 
                                 CreateNodeFunc createNode, 
                                 UpdateNodeFunc updateNode, 
                                 const CreateLeafFunc& createLeaf, 
                                 SplitPrimitiveFunc splitPrimitive,
                                 ProgressMonitor progressMonitor,
                                 PrimRef* prims, 
                                 const size_t extSize,
                                 const PrimInfo& pinfo, 
                                 const Settings& settings)
      {
        typedef HeuristicArraySpatialSAH<SplitPrimitiveFunc,PrimRef,NUM_OBJECT_BINS,NUM_SPATIAL_BINS> Heuristic;
        Heuristic heuristic(splitPrimitive,prims,pinfo);

        return GeneralBVHBuilder::build<ReductionTy,Heuristic,Set>(
          heuristic,
          Set(0,pinfo.size(),extSize),
          createAlloc,
          createNode,
          updateNode,
          createLeaf,
          progressMonitor,
          pinfo,
          settings);
      }
    };
  }
}
