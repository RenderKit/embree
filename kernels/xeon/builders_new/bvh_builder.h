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

namespace embree
{
  namespace isa
  {
    template<typename NodeRef>
      class __aligned(64) BuildRecord : public PrimInfo
    {
    public:
      __forceinline BuildRecord() {}
      
      __forceinline BuildRecord(const PrimInfo& pinfo, const size_t depth, NodeRef* parent) 
        : PrimInfo(pinfo), depth(depth), parent(parent), area(embree::area(pinfo.geomBounds)) {}
      
      __forceinline void init(size_t depth)
      {
        parent = NULL;
        this->depth = depth;
        area = embree::area(geomBounds);
      }
      
      __forceinline bool operator< (const BuildRecord &br) const { 
	return size() < br.size(); 
      } 

    public:
      range<size_t> prims;
      unsigned depth;         //!< depth from the root of the tree
      float    area;          //!< surface area of bounding box
      NodeRef* parent;        //!< reference pointing to us
    };
    
    template<typename NodeRef, typename Heuristic, typename ReductionTy, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor>
      class BVHBuilderSAH
    {
      typedef typename Heuristic::Split Split;
      static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth

    public:

      BVHBuilderSAH (Heuristic& heuristic,
		     const ReductionTy& identity,
		     CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, UpdateNodeFunc& updateNode, CreateLeafFunc& createLeaf,
                     ProgressMonitor& progressMonitor,
		     const PrimInfo& pinfo,
		     const size_t branchingFactor, const size_t maxDepth, 
		     const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize)
        : heuristic(heuristic), 
	  identity(identity), 
	  createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
          progressMonitor(progressMonitor),
          pinfo(pinfo), 
          branchingFactor(branchingFactor), maxDepth(maxDepth),
          logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
      }

      __forceinline void split(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        const PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        const Split split = heuristic.find(current.prims,pinfo,logBlockSize);
	heuristic.split(split, current, current.prims, leftChild, leftChild.prims, rightChild, rightChild.prims);
      }

      const ReductionTy createLargeLeaf(const BuildRecord<NodeRef>& current, Allocator alloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");

        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize)
          return createLeaf(current,alloc);

        /* fill all children by always splitting the largest one */
	ReductionTy values[MAX_BRANCHING_FACTOR];
	BuildRecord<NodeRef>* pchildren[MAX_BRANCHING_FACTOR];
        BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
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
            if (children[i].size() <= maxLeafSize)
              continue;
            
            /* remember child with largest size */
            if (children[i].size() > bestSize) { 
              bestSize = children[i].size();
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          __aligned(64) BuildRecord<NodeRef> left, right;
          heuristic.splitFallback(children[bestChild].prims,left,left.prims,right,right.prims);
          
          /* add new children left and right */
          left.init(current.depth+1); 
          right.init(current.depth+1);
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
	  pchildren[numChildren] = &children[numChildren];
          numChildren++;
          
        } while (numChildren < branchingFactor);

        /* create node */
        auto node = createNode(current,pchildren,numChildren,alloc);

	/* recurse into each child */
	for (size_t i=0; i<numChildren; i++)
	  values[i] = createLargeLeaf(children[i],alloc);
	
	/* perform reduction */
	return updateNode(node,values,numChildren);
      }

      const ReductionTy recurse(const BuildRecord<NodeRef>& current, Allocator alloc)
      {
        bool topLevel = (bool) alloc;
	if (alloc == NULL) 
          alloc = createAlloc();
	
	ReductionTy values[MAX_BRANCHING_FACTOR];
	BuildRecord<NodeRef>* pchildren[MAX_BRANCHING_FACTOR];
        __aligned(64) BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
	if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
	  heuristic.deterministic_order(current.prims);
	  return createLargeLeaf(current,alloc);
	}
                
        /* fill all children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = current;
        pchildren[0] = &children[0];

        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          float bestArea = neg_inf;
          for (size_t i=0; i<numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].size() <= minLeafSize)
              continue;
            
            /* remember child with largest area */
            if (children[i].area > bestArea) { 
              bestArea = children[i].area;
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          __aligned(64) BuildRecord<NodeRef> left, right;
          split(children[bestChild],left,right);
          
          /* add new children left and right */
          left.init(current.depth+1); 
          right.init(current.depth+1);
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
	  pchildren[numChildren] = &children[numChildren];
          numChildren++;
          
        } while (numChildren < branchingFactor);
        assert(numChildren > 1);
	
	/* sort buildrecords for optimal cache locality */
	std::sort(&children[0],&children[numChildren]);
        
	/* create node */
	auto node = createNode(current,pchildren,numChildren,alloc);

	/* spawn tasks */
	if (current.size() > 4096) 
	{
	  SPAWN_BEGIN;
	  //for (ssize_t i=numChildren-1; i>=0; i--)  // FIXME: this should be better!
	  for (size_t i=0; i<numChildren; i++) 
	    SPAWN(([&,i] { values[i] = recurse(children[i],NULL); }));
	  SPAWN_END;
	  
	  /* passed reduced values to node */
	  return updateNode(node,values,numChildren);
	}
	/* recurse into each child */
	else 
	{
          /* call memory monitor function to signal progress */
          if (topLevel)
            progressMonitor(current.size());

	  for (size_t i=0; i<numChildren; i++)
	    values[i] = recurse(children[i],alloc);

	  /* perform reduction */
	  return updateNode(node,values,numChildren);
	}
      }

      /*! builder entry function */
      __forceinline const ReductionTy operator() (BuildRecord<NodeRef>& br) { 
	return recurse(br,NULL);
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
    };

    template<typename NodeRef, typename CreateAllocFunc, typename ReductionTy, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor>
      NodeRef bvh_builder_reduce_binned_sah_internal(CreateAllocFunc createAlloc, 
						      const ReductionTy& identity, 
						      CreateNodeFunc createNode, UpdateNodeFunc updateNode, CreateLeafFunc createLeaf,
                                                      ProgressMonitor& progressMonitor,
						      PrimRef* prims, const PrimInfo& pinfo, 
						      const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      
      BVHBuilderSAH<NodeRef,decltype(heuristic),ReductionTy,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,UpdateNodeFunc,CreateLeafFunc,ProgressMonitor> builder
        (heuristic,identity,createAlloc,createNode,updateNode,createLeaf,progressMonitor,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);

      NodeRef root;
      BuildRecord<NodeRef> br(pinfo,1,&root);
      br.prims = range<size_t>(0,pinfo.size());
      builder(br); // FIXME: return reduced value
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor>
      NodeRef bvh_builder_binned_sah_internal(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf,
                                              ProgressMonitor progressMonitor,
                                              PrimRef* prims, const PrimInfo& pinfo, 
                                              const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      
      auto updateNode = [] (int node, int*, size_t) -> int { return 0; };
      BVHBuilderSAH<NodeRef,decltype(heuristic),int,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,decltype(updateNode),CreateLeafFunc,ProgressMonitor> builder
        (heuristic,0,createAlloc,createNode,updateNode,createLeaf,progressMonitor,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);

      NodeRef root;
      BuildRecord<NodeRef> br(pinfo,1,&root);
      br.prims = range<size_t>(0,pinfo.size());
      builder(br);
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc, typename ProgressMonitor> // FIXME: remove this function
      NodeRef bvh_builder_binned_sah(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                     ProgressMonitor progressMonitor,
                                     PrimRef* prims, const PrimInfo& pinfo, 
                                     const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize)
    {
      return bvh_builder_binned_sah_internal<NodeRef>(createAlloc,createNode,createLeaf,progressMonitor,prims,pinfo,
                                                      branchingFactor,maxDepth,blockSize,minLeafSize,maxLeafSize);
    }
  }
}
