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
      
#if defined(_MSC_VER)
      __forceinline BuildRecord& operator=(const BuildRecord &arg) { 
        memcpy(this, &arg, sizeof(BuildRecord));    
        return *this;
      }
#endif

      __forceinline void init(size_t depth)
      {
        parent = NULL;
        this->depth = depth;
        area = embree::area(geomBounds);
      }
      
      __forceinline bool operator<(const BuildRecord &br) const { return size() < br.size(); } 
      __forceinline bool operator>(const BuildRecord &br) const { return size() > br.size(); } 
      
      struct Greater {
        __forceinline bool operator()(const BuildRecord& a, const BuildRecord& b) {
          return a.size() > b.size();
        }
      };

    public:
      range<size_t> prims;
      unsigned depth;         //!< depth from the root of the tree
      float    area;          //!< surface area of bounding box
      NodeRef* parent;        //!< reference pointing to us
    };
    
    template<typename NodeRef, typename Heuristic, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      class BVHBuilderSAH
    {
      typedef typename Heuristic::Split Split;
      static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth

    public:

      BVHBuilderSAH (Heuristic& heuristic,CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, CreateLeafFunc& createLeaf,
                         PrimRef* prims, const PrimInfo& pinfo,
                         const size_t branchingFactor, const size_t maxDepth, 
                         const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize)
        : heuristic(heuristic), createAlloc(createAlloc), createNode(createNode), createLeaf(createLeaf), 
          prims(prims), pinfo(pinfo), 
          branchingFactor(branchingFactor), maxDepth(maxDepth),
          logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
      }

      __forceinline void splitSequential(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        /* calculate binning function */
        PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        Split split = heuristic.find(current.prims,pinfo,logBlockSize);

        /* if we cannot find a valid split, enforce an arbitrary split */
        if (unlikely(!split.valid())) heuristic.splitFallback(current.prims,leftChild,leftChild.prims,rightChild,rightChild.prims);
        
        /* partitioning of items */
        else heuristic.split(split, current.prims, leftChild, leftChild.prims, rightChild, rightChild.prims);
      }

      void splitParallel(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild, Allocator& alloc)
      {
        /* calculate binning function */
        PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        Split split = heuristic.parallel_find(current.prims,pinfo,logBlockSize);
        
        /* if we cannot find a valid split, enforce an arbitrary split */
        if (unlikely(!split.valid())) heuristic.splitFallback(current.prims,leftChild,leftChild.prims,rightChild,rightChild.prims);
        
        /* partitioning of items */
        else heuristic.parallel_split(split, current.prims, leftChild, leftChild.prims, rightChild, rightChild.prims);
      }

      void createLargeLeaf(const BuildRecord<NodeRef>& current, Allocator& nodeAlloc, Allocator& leafAlloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize) {
          createLeaf(current,prims,leafAlloc);
          return;
        }

        /* fill all children by always splitting the largest one */
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
        createNode(current,pchildren,numChildren,nodeAlloc);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          createLargeLeaf(children[i],nodeAlloc,leafAlloc);
      }

      template<bool toplevel, typename Spawn>
        inline void recurse(const BuildRecord<NodeRef>& current, Allocator& alloc, Spawn& spawn)
      {
	BuildRecord<NodeRef>* pchildren[MAX_BRANCHING_FACTOR];
        __aligned(64) BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (!toplevel) {
          if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
            createLargeLeaf(current,alloc,alloc); // FIXME: alloc,alloc
            return;
          }
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
          if (toplevel) splitParallel(children[bestChild],left,right,alloc);
          else          splitSequential(children[bestChild],left,right);
          
          /* add new children left and right */
          left.init(current.depth+1); 
          right.init(current.depth+1);
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
	  pchildren[numChildren] = &children[numChildren];
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /* create leaf node if no split is possible */
        if (!toplevel && numChildren == 1) {
          createLargeLeaf(current,alloc,alloc);
          return;
        }
        
        /* create node */
        createNode(current,pchildren,numChildren,alloc);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          spawn(children[i]);
      }

      /*! builder entry function */
      __forceinline void operator() (BuildRecord<NodeRef>& br)
      { 
#if 0
        sequential_create_tree(br, createAlloc, 
                               [&](const BuildRecord<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord<NodeRef> >& cont) { recurse<false>(br,alloc,cont); });
#else   
        parallel_create_tree<50000,128>(br, createAlloc, 
          [&](const BuildRecord<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord<NodeRef> >& cont) { recurse<true>(br,alloc,cont); } ,
          [&](const BuildRecord<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord<NodeRef> >& cont) { recurse<false>(br,alloc,cont); });
#endif
      }

    private:
      Heuristic& heuristic;
      CreateAllocFunc& createAlloc;
      CreateNodeFunc& createNode;
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
      
      BVHBuilderSAH<NodeRef,decltype(heuristic),decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,CreateLeafFunc> builder
        (heuristic,createAlloc,createNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);

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
      HeuristicArrayBinningSAH<PrimRef> heuristic(prims);
      const size_t logBlockSize = __bsr(blockSize);
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      return execute_closure([&]() -> NodeRef 
      {
          BVHBuilderSAH<NodeRef,HeuristicArrayBinningSAH<PrimRef>,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,CreateLeafFunc> builder
            (heuristic,createAlloc,createNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);

	  NodeRef root;
	  BuildRecord<NodeRef> br(pinfo,1,&root);
	  br.prims = range<size_t>(0,pinfo.size());
          builder(br);
	  return root;
        });
    }
  }
}
