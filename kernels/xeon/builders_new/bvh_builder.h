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

#include "builders_new/heuristic_binning.h"
#include "algorithms/parallel_create_tree.h"

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
    
    template<typename NodeRef, typename Heuristic, typename ReductionTy, typename ReductionFunc, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc>
      class BVHBuilderSAH
    {
      typedef typename Heuristic::Split Split;
      static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth

    public:

      BVHBuilderSAH (Heuristic& heuristic,
		     const ReductionTy& identity, ReductionFunc& reduce,
		     CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, UpdateNodeFunc& updateNode, CreateLeafFunc& createLeaf,
		     PrimRef* prims, const PrimInfo& pinfo,
		     const size_t branchingFactor, const size_t maxDepth, 
		     const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize)
        : heuristic(heuristic), 
	  identity(identity), reduce(reduce),
	  createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
          prims(prims), pinfo(pinfo), 
          branchingFactor(branchingFactor), maxDepth(maxDepth),
          logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
      }

      __forceinline void splitSequential(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        const PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        const Split split = heuristic.find(current.prims,pinfo,logBlockSize);
	heuristic.split(split, current.prims, leftChild, leftChild.prims, rightChild, rightChild.prims);
      }

      void splitParallel(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        const PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        const Split split = heuristic.parallel_find(current.prims,pinfo,logBlockSize);
	heuristic.parallel_split(split, current.prims, leftChild, leftChild.prims, rightChild, rightChild.prims);
      }

      const ReductionTy createLargeLeaf(const BuildRecord<NodeRef>& current, Allocator alloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize)
          return createLeaf(current,prims,alloc);

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

	/* recurse into each child  and perform reduction */
	ReductionTy v = identity;
	for (size_t i=0; i<numChildren; i++)
	  v = reduce(v, values[i] = createLargeLeaf(children[i],alloc));
	
	/* passed reduced values to node */
	updateNode(node,v,values,numChildren);
	return v;
      }

      const ReductionTy recurse(const BuildRecord<NodeRef>& current, Allocator alloc)
      {
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
          if (children[bestChild].size() > 10000) splitParallel  (children[bestChild],left,right);
          else                                    splitSequential(children[bestChild],left,right);
          
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
	  
	  /* perform reduction */
	  ReductionTy v = values[0];
	  for (size_t i=1; i<numChildren; i++)
	    v = reduce(v,values[i]);

	  /* passed reduced values to node */
	  updateNode(node,v,values,numChildren);
	  return v;
	}
	/* recurse into each child */
	else 
	{
	  /* perform reduction */
	  ReductionTy v = identity;
	  for (size_t i=0; i<numChildren; i++)
	    v = reduce(v, values[i] = recurse(children[i],alloc));
	  
	  /* passed reduced values to node */
	  updateNode(node,v,values,numChildren);
	  return v;
	}
      }

      /*! builder entry function */
      __forceinline const ReductionTy operator() (BuildRecord<NodeRef>& br) { 
	return recurse(br,NULL);
      }

    private:
      Heuristic& heuristic;
      const ReductionTy identity;
      ReductionFunc& reduce;
      CreateAllocFunc& createAlloc;
      CreateNodeFunc& createNode;
      UpdateNodeFunc& updateNode;
      CreateLeafFunc& createLeaf;
      
    private:
      PrimRef* prims;
      const PrimInfo& pinfo;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
    };

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      NodeRef bvh_builder_binned_sah_internal(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                              PrimRef* prims, const PrimInfo& pinfo, 
                                              const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      
      auto plus = std::plus<int>();
      auto updateNode = [] (int node, int, int*, size_t) {};
      BVHBuilderSAH<NodeRef,decltype(heuristic),int,decltype(plus),decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,decltype(updateNode),CreateLeafFunc> builder
        (heuristic,0,plus,createAlloc,createNode,updateNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);

      NodeRef root;
      BuildRecord<NodeRef> br(pinfo,1,&root);
      br.prims = range<size_t>(0,pinfo.size());
      builder(br);
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      NodeRef bvh_builder_binned_sah(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                     PrimRef* prims, const PrimInfo& pinfo, 
                                     const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize)
    {
      NodeRef root;
      SPAWN_ROOT(([&] {
	    root = bvh_builder_binned_sah_internal<NodeRef>(createAlloc,createNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,blockSize,minLeafSize,maxLeafSize);
	  }));
      return root;
    }
  }
}
