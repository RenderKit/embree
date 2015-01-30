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

    template<typename NodeRef, typename Heuristic, typename ReductionTy, typename ReductionFunc, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename UpdateNodeFunc, typename CreateLeafFunc>
      class BVHBuilderSAH2
    {
      static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth

      struct BuildRecord : public BuildRecord2<NodeRef>
      {
      public:
	__forceinline BuildRecord () {}
        
	__forceinline BuildRecord (size_t depth) 
	  : BuildRecord2<NodeRef>(depth) {}
        
        __forceinline BuildRecord (const PrimInfo& pinfo, size_t depth, NodeRef* parent) 
	  : BuildRecord2<NodeRef>(pinfo,depth,parent) {}

	__forceinline BuildRecord(const BuildRecord2<NodeRef>& other)
	  : BuildRecord2<NodeRef>(other) {}

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
		      const ReductionTy& identity, ReductionFunc& reduce,
		      CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, UpdateNodeFunc& updateNode, CreateLeafFunc& createLeaf,
                      PrimRef* prims, const PrimInfo& pinfo,
                      const size_t branchingFactor, const size_t maxDepth, 
                      const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize,
                      const float travCost, const float intCost)
        : heuristic(heuristic), 
	identity(identity), reduce(reduce),
	createAlloc(createAlloc), createNode(createNode), updateNode(updateNode), createLeaf(createLeaf), 
        prims(prims), pinfo(pinfo), 
        branchingFactor(branchingFactor), maxDepth(maxDepth),
        logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
        travCost(travCost), intCost(intCost)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
      }

      const ReductionTy createLargeLeaf(const BuildRecord& current, Allocator alloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.pinfo.size() <= maxLeafSize)
          return createLeaf(current,prims,alloc);

        /* fill all children by always splitting the largest one */
	ReductionTy values[MAX_BRANCHING_FACTOR];
	BuildRecord2<NodeRef>* pchildren[MAX_BRANCHING_FACTOR];
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
	ReductionTy v = identity;
	for (size_t i=0; i<numChildren; i++)
	  v = reduce(v, values[i] = createLargeLeaf(children[i],alloc));
	
	/* passed reduced values to node */
	updateNode(node,v,values,numChildren);
	return v;
      }

      __forceinline const typename Heuristic::Split find(BuildRecord& current) 
      {
        if (current.size() > 10000) return heuristic.parallel_find(current.prims,current.pinfo,logBlockSize);
        else                        return heuristic.find         (current.prims,current.pinfo,logBlockSize);
      }

      __forceinline void partition(const BuildRecord& brecord, BuildRecord& lrecord, BuildRecord& rrecord) 
      {
	if (brecord.size() > 10000) heuristic.parallel_split(brecord.split,brecord.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
	else                        heuristic.split         (brecord.split,brecord.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
      }

      const ReductionTy recurse(const BuildRecord& current, Allocator alloc)
      {
	if (alloc == NULL) 
          alloc = createAlloc();

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
	BuildRecord2<NodeRef>* pchildren[MAX_BRANCHING_FACTOR];
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
            if (children[i].pinfo.size() > maxLeafSize) dSAH = min(0.0f,dSAH); //< force split for large jobs
            if (dSAH <= bestSAH) { bestChild = i; bestSAH = dSAH; }
            //if (area(children[i].pinfo.geomBounds) > bestSAH) { bestChild = i; bestSAH = area(children[i].pinfo.geomBounds); }
          }
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
	  pchildren[numChildren] = &children[numChildren];
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /*! create an inner node */
        auto node = createNode(current,pchildren,numChildren,alloc);

	/* sort buildrecords for optimal cache locality */
	std::sort(&children[0],&children[numChildren]);

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
      __forceinline const ReductionTy operator() (BuildRecord2<NodeRef>& record)
      {
	BuildRecord br(record);
        br.split = find(br); 
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
      const float travCost;
      const float intCost;
    };
    
    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      NodeRef bvh_builder_binned_sah2_internal(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                               PrimRef* prims, const PrimInfo& pinfo, 
                                               const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                               const float travCost, const float intCost)
    {
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      
      auto plus = std::plus<int>();
      auto updateNode = [] (int node, int, int*, size_t) {};
      BVHBuilderSAH2<NodeRef,decltype(heuristic),int,decltype(plus),decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,decltype(updateNode),CreateLeafFunc> builder
        (heuristic,0,plus,createAlloc,createNode,updateNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);

      NodeRef root;
      BuildRecord2<NodeRef> br(pinfo,1,&root);
      br.prims = range<size_t>(0,pinfo.size());
      builder(br);
      return root;
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      NodeRef bvh_builder_binned_sah2(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                     PrimRef* prims, const PrimInfo& pinfo, 
                                      const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                      const float travCost, const float intCost)
    {
      NodeRef root;
      SPAWN_ROOT(([&] {
	    root = bvh_builder_binned_sah2_internal<NodeRef>(createAlloc,createNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
	  }));
      return root;
    }
  }
}
