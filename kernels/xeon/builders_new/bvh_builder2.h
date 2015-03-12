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

#include "tasking/taskscheduler_new.h"
#include "builders_new/heuristic_binning_array_aligned.h"
#include "builders_new/heuristic_spatial_binning_list.h"

namespace embree
{
  namespace isa
  {
    /*! the build record stores all information to continue the build of some subtree */
    template<typename NodeRef, typename Set = range<size_t> >
      struct BuildRecord2 
      {
      public:
	__forceinline BuildRecord2 () {}
        
	__forceinline BuildRecord2 (size_t depth) : depth(depth), pinfo(empty) {}
        
        __forceinline BuildRecord2 (const PrimInfo& pinfo, size_t depth, NodeRef* parent) 
          : pinfo(pinfo), depth(depth), parent(parent) {}

      public:
	NodeRef*   parent;      //!< Pointer to the parent node's reference to us
	size_t     depth;    //!< Depth of the root of this subtree.
	Set prims;            //!< The list of primitives.
	PrimInfo   pinfo;    //!< Bounding info of primitives.
      };

    template<typename Set, typename NodeRef, typename Heuristic, typename ReductionTy, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor>
      class BVHBuilderSAH2
    {
      static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth
      static const size_t SINGLE_THREADED_THRESHOLD = 4096;

      struct BuildRecord : public BuildRecord2<NodeRef,Set>
      {
      public:
	__forceinline BuildRecord () {}
        
	__forceinline BuildRecord (size_t depth) 
	  : BuildRecord2<NodeRef,Set>(depth) {}
        
        __forceinline BuildRecord (const PrimInfo& pinfo, size_t depth, NodeRef* parent) 
	  : BuildRecord2<NodeRef,Set>(pinfo,depth,parent) {}

	__forceinline BuildRecord(const BuildRecord2<NodeRef,Set>& other)
	  : BuildRecord2<NodeRef,Set>(other) {}

	__forceinline friend bool operator< (const BuildRecord& a, const BuildRecord& b) { return a.pinfo.size() < b.pinfo.size(); }
	__forceinline friend bool operator> (const BuildRecord& a, const BuildRecord& b) { return a.pinfo.size() > b.pinfo.size(); }

        __forceinline size_t size() const { return this->pinfo.size(); }
        
        struct Greater {
          __forceinline bool operator()(const BuildRecord& a, const BuildRecord& b) {
            return a.size() > b.size();
          }
        };

      public:
	typename Heuristic::Split split;    //!< The best split for the primitives.
      };

    public:

      BVHBuilderSAH2 (Heuristic& heuristic, 
		      const ReductionTy& identity,
		      CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, UpdateNodeFunc& updateNode, CreateLeafFunc& createLeaf,
                      ProgressMonitor& progressMonitor,
                      const PrimInfo& pinfo,
                      const size_t branchingFactor, const size_t maxDepth, 
                      const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize,
                      const float travCost, const float intCost)
        : heuristic(heuristic), 
	identity(identity), 
	createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
        progressMonitor(progressMonitor),
        pinfo(pinfo), 
        branchingFactor(branchingFactor), maxDepth(maxDepth),
        logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
        travCost(travCost), intCost(intCost)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
      }

      const ReductionTy createLargeLeaf(BuildRecord& current, Allocator alloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.pinfo.size() <= maxLeafSize)
          return createLeaf(current,alloc);

        /* fill all children by always splitting the largest one */
	ReductionTy values[MAX_BRANCHING_FACTOR];
	BuildRecord2<NodeRef,Set>* pchildren[MAX_BRANCHING_FACTOR];
        BuildRecord children[MAX_BRANCHING_FACTOR];
        size_t numChildren = 1;
        children[0] = current;
	pchildren[0] = &children[0];
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          int bestSize = 0;
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
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          BuildRecord left(current.depth+1);
          BuildRecord right(current.depth+1);
          heuristic.splitFallback(children[bestChild].prims,left.pinfo,left.prims,right.pinfo,right.prims);
          
          /* add new children left and right */
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
	  pchildren[numChildren] = &children[numChildren];
          numChildren++;
          
        } while (numChildren < branchingFactor);

        /* create node */
        auto node = createNode(current,pchildren,numChildren,alloc);

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
	if (alloc == NULL)
          alloc = createAlloc();

        /* call memory monitor function to signal progress */
        if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
          progressMonitor(current.size());

        /*! compute leaf and split cost */
        const float leafSAH  = intCost*current.pinfo.leafSAH(logBlockSize);
        const float splitSAH = travCost*halfArea(current.pinfo.geomBounds)+intCost*current.split.splitSAH();
        assert(current.pinfo.size() == 0 || leafSAH >= 0 && splitSAH >= 0);
        
        /*! create a leaf node when threshold reached or SAH tells us to stop */
        if (current.pinfo.size() <= minLeafSize || current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || (current.pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
	  heuristic.deterministic_order(current.prims);
          return createLargeLeaf(current,alloc);
        }
        
        /*! initialize child list */
	ReductionTy values[MAX_BRANCHING_FACTOR];
	BuildRecord2<NodeRef,Set>* pchildren[MAX_BRANCHING_FACTOR];
        BuildRecord children[MAX_BRANCHING_FACTOR];
        children[0] = current;
	pchildren[0] = &children[0];
        size_t numChildren = 1;
        
        /*! split until node is full or SAH tells us to stop */
        do {
          
          /*! find best child to split */
          float bestSAH = 0;
          ssize_t bestChild = -1;
          for (size_t i=0; i<numChildren; i++) 
          {
            float dSAH = children[i].split.splitSAH()-children[i].pinfo.leafSAH(logBlockSize);
            if (children[i].pinfo.size() <= minLeafSize) continue; 
            if (children[i].pinfo.size() > maxLeafSize) dSAH = min(dSAH,0.0f); //< force split for large jobs
            if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
            //if (area(children[i].pinfo.geomBounds) > bestSAH) { bestChild = i; bestSAH = area(children[i].pinfo.geomBounds); } // FIXME: measure over all scenes if this line creates better tree
          }
          if (bestChild == -1) break;
          
          /* perform best found split */
          BuildRecord& brecord = children[bestChild];
          BuildRecord lrecord(current.depth+1);
          BuildRecord rrecord(current.depth+1);
	  partition(brecord,lrecord,rrecord);
          if (lrecord.size() + rrecord.size() != brecord.size())
            throw std::runtime_error("error");
          
          /* find new splits */
          lrecord.split = find(lrecord);
          rrecord.split = find(rrecord);
          children[bestChild  ] = lrecord;
          children[numChildren] = rrecord;
	  pchildren[numChildren] = &children[numChildren];
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
	/* sort buildrecords for optimal cache locality */
	//std::sort(&children[0],&children[numChildren]); // FIXME: reduces traversal performance of bvh8.triangle4 !!

        /*! create an inner node */
        auto node = createNode(current,pchildren,numChildren,alloc);

	/* spawn tasks */
	if (current.size() > SINGLE_THREADED_THRESHOLD) 
	{
	  SPAWN_BEGIN;
	  for (ssize_t i=numChildren-1; i>=0; i--)  // FIXME: this should be better!
            //for (size_t i=0; i<numChildren; i++) 
	    SPAWN(([&,i] { values[i] = recurse(children[i],NULL,true); }));
	  SPAWN_END;
	  
	  /* perform reduction */
	  return updateNode(node,values,numChildren);
	}
	/* recurse into each child */
	else 
	{
          for (ssize_t i=numChildren-1; i>=0; i--) {
            //for (size_t i=0; i<numChildren; i++) {
	    values[i] = recurse(children[i],alloc,false);
          }
	  
	  /* perform reduction */
	  return updateNode(node,values,numChildren);
	}
      }
      
      /*! builder entry function */
      __forceinline const ReductionTy operator() (BuildRecord2<NodeRef,Set>& record)
      {
	BuildRecord br(record);
        br.split = find(br); 
	return recurse(br,NULL,true);
      }
      
    private:
      Heuristic& heuristic;
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
    };
    
    template<typename NodeRef, typename CreateAllocFunc, typename ReductionTy, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor>
      NodeRef bvh_builder_reduce_binned_sah2_internal(CreateAllocFunc createAlloc, 
						      const ReductionTy& identity, 
						      CreateNodeFunc createNode, UpdateNodeFunc updateNode, CreateLeafFunc createLeaf, 
                                                      ProgressMonitor& progressMonitor,
						      PrimRef* prims, const PrimInfo& pinfo, 
						      const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
						      const float travCost, const float intCost)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      
      BVHBuilderSAH2<range<size_t>,NodeRef,decltype(heuristic),ReductionTy,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,UpdateNodeFunc,CreateLeafFunc,ProgressMonitor> builder
        (heuristic,identity,createAlloc,createNode,updateNode,createLeaf,progressMonitor,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

      NodeRef root;
      BuildRecord2<NodeRef> br(pinfo,1,&root);
      br.prims = range<size_t>(0,pinfo.size());
      builder(br); // FIXME: return reduced value
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename ReductionTy, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc, typename SplitPrimitiveFunc, typename ProgressMonitor>
      NodeRef bvh_builder_reduce_spatial_sah2_internal(Scene* scene, CreateAllocFunc createAlloc, 
                                                       const ReductionTy& identity, 
                                                       CreateNodeFunc createNode, UpdateNodeFunc updateNode, CreateLeafFunc createLeaf, SplitPrimitiveFunc splitPrimitive,
                                                       ProgressMonitor& progressMonitor,
                                                       PrimRefList& prims, const PrimInfo& pinfo, 
                                                       const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                                       const float travCost, const float intCost)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);

      //HeuristicListBinningSAH<PrimRef> heuristic;
      HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH<PrimRef,SplitPrimitiveFunc> heuristic(splitPrimitive);
      
      //auto updateNode = [] (int node, int*, size_t) -> int { return 0; };
      BVHBuilderSAH2<PrimRefList,NodeRef,decltype(heuristic),ReductionTy,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,UpdateNodeFunc,CreateLeafFunc,ProgressMonitor> builder
        (heuristic,identity,createAlloc,createNode,updateNode,createLeaf,progressMonitor,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

      NodeRef root;
      BuildRecord2<NodeRef,PrimRefList> br(pinfo,1,&root);
      br.prims = prims;
      builder(br);
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor>
      NodeRef bvh_builder_binned_sah2_internal(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                               ProgressMonitor& progressMonitor, 
                                               PrimRef* prims, const PrimInfo& pinfo, 
                                               const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                               const float travCost, const float intCost)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      
      auto updateNode = [] (int node, int*, size_t) -> int { return 0; };
      BVHBuilderSAH2<range<size_t>,NodeRef,decltype(heuristic),int,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,decltype(updateNode),CreateLeafFunc,ProgressMonitor> builder
        (heuristic,0,createAlloc,createNode,updateNode,createLeaf,progressMonitor,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

      NodeRef root;
      BuildRecord2<NodeRef> br(pinfo,1,&root);
      br.prims = range<size_t>(0,pinfo.size());
      builder(br);
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc, typename SplitPrimitiveFunc, typename ProgressMonitor>
      NodeRef bvh_builder_spatial_sah2_internal(Scene* scene, CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, SplitPrimitiveFunc splitPrimitive,
                                                ProgressMonitor& progressMonitor,
                                                PrimRefList& prims, const PrimInfo& pinfo, 
                                                const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                                const float travCost, const float intCost) // FIXME: move these constants into struct!
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);

      //HeuristicListBinningSAH<PrimRef> heuristic;
      HeuristicSpatialSplitAndObjectSplitBlockListBinningSAH<PrimRef,SplitPrimitiveFunc> heuristic(splitPrimitive);
      
      auto updateNode = [] (int node, int*, size_t) -> int { return 0; };
      BVHBuilderSAH2<PrimRefList,NodeRef,decltype(heuristic),int,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,decltype(updateNode),CreateLeafFunc,ProgressMonitor> builder
        (heuristic,0,createAlloc,createNode,updateNode,createLeaf,progressMonitor,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

      NodeRef root;
      BuildRecord2<NodeRef,PrimRefList> br(pinfo,1,&root);
      br.prims = prims;
      builder(br);
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor> // FIXME: remove this function
      NodeRef bvh_builder_binned_sah2(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                      ProgressMonitor& progressMonitor,
                                     PrimRef* prims, const PrimInfo& pinfo, 
                                      const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                      const float travCost, const float intCost)
    {
      return bvh_builder_binned_sah2_internal<NodeRef>(createAlloc,createNode,createLeaf,progressMonitor,prims,pinfo,
                                                       branchingFactor,maxDepth,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
    }
  }
}
