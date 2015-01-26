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
    template<typename NodeRef>
      struct BuildRecord2 
      {
      public:
	__forceinline BuildRecord2 () {}
        
	__forceinline BuildRecord2 (size_t depth) : depth(depth), pinfo(empty) {}
        
        __forceinline BuildRecord2 (const PrimInfo& pinfo, size_t depth, NodeRef* parent) 
          : pinfo(pinfo), depth(depth), parent(parent) {}

	__forceinline friend bool operator< (const BuildRecord2& a, const BuildRecord2& b) { return a.pinfo.size() < b.pinfo.size(); }
	__forceinline friend bool operator> (const BuildRecord2& a, const BuildRecord2& b) { return a.pinfo.size() > b.pinfo.size(); }

        __forceinline size_t size() const { return pinfo.size(); }
        
        struct Greater {
          __forceinline bool operator()(const BuildRecord2& a, const BuildRecord2& b) {
            return a.size() > b.size();
          }
        };

      public:
	NodeRef*   parent;      //!< Pointer to the parent node's reference to us
	size_t     depth;    //!< Depth of the root of this subtree.
	//PrimRefList prims;    //!< The list of primitives.
	PrimInfo   pinfo;    //!< Bounding info of primitives.
	BinSplit<32>      split;    //!< The best split for the primitives.
	// FIXME: BinSplit
      };

    template<typename Heuristic, typename NodeRef, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      class BVHBuilderSAH2
    {
      static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth

    public:

      BVHBuilderSAH2 (CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, CreateLeafFunc& createLeaf,
                      PrimRef* prims, const PrimInfo& pinfo,
                      const size_t branchingFactor, const size_t maxDepth, 
                      const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize,
                      const float travCost, const float intCost)
        : createAlloc(createAlloc), createNode(createNode), createLeaf(createLeaf), 
        prims(prims), pinfo(pinfo), 
        branchingFactor(branchingFactor), maxDepth(maxDepth),
        logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
        travCost(travCost), intCost(intCost)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh_builder: branching factor too large");
      }

      void createLargeLeaf(const BuildRecord2<NodeRef>& current, Allocator& alloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.pinfo.size() <= maxLeafSize) {
          createLeaf(current,prims,alloc);
          return;
        }

        /* fill all children by always splitting the largest one */
        BuildRecord2<NodeRef> children[MAX_BRANCHING_FACTOR];
        size_t numChildren = 1;
        children[0] = current;
        
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
          BuildRecord2<NodeRef> left(current.depth+1);
          BuildRecord2<NodeRef> right(current.depth+1);
          splitFallback(children[bestChild],left,right);
          
          /* add new children left and right */
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor);

        /* create node */
        createNode(current,children,numChildren,alloc);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          createLargeLeaf(children[i],alloc);
      }

      void splitFallback(const BuildRecord2<NodeRef>& current, BuildRecord2<NodeRef>& leftChild, BuildRecord2<NodeRef>& rightChild)
      {
        const size_t center = (current.pinfo.begin + current.pinfo.end)/2;
        
        CentGeomBBox3fa left; left.reset();
        for (size_t i=current.pinfo.begin; i<center; i++)
          left.extend(prims[i].bounds());
        new (&leftChild.pinfo) PrimInfo(current.pinfo.begin,center,left.geomBounds,left.centBounds);
        
        CentGeomBBox3fa right; right.reset();
        for (size_t i=center; i<current.pinfo.end; i++)
          right.extend(prims[i].bounds());	
        new (&rightChild.pinfo) PrimInfo(center,current.pinfo.end,right.geomBounds,right.centBounds);
      }

      template<bool toplevel>
        __forceinline const typename Heuristic::Split find(BuildRecord2<NodeRef>& current) {
        if (toplevel) return Heuristic::parallel_find(prims,current.pinfo.begin,current.pinfo.end,current.pinfo,logBlockSize);
        else          return Heuristic::find(prims,current.pinfo.begin,current.pinfo.end,current.pinfo,logBlockSize);
      }

      template<bool toplevel>
      __forceinline void partition(const BuildRecord2<NodeRef>& brecord, BuildRecord2<NodeRef>& lrecord, BuildRecord2<NodeRef>& rrecord) {
        if (brecord.split.sah == float(inf)) splitFallback(brecord,lrecord,rrecord);
        else {
          if (toplevel) Heuristic::parallel_split(brecord.split,prims,brecord.pinfo.begin,brecord.pinfo.end,lrecord.pinfo,rrecord.pinfo);
          else          Heuristic::split         (brecord.split,prims,brecord.pinfo.begin,brecord.pinfo.end,lrecord.pinfo,rrecord.pinfo);
        }
      }

      template<bool toplevel, typename Spawn>
        inline void recurse(const BuildRecord2<NodeRef>& record, Allocator& alloc, Spawn& spawn)
      {
        /*! compute leaf and split cost */
        const float leafSAH  = intCost*record.pinfo.leafSAH(logBlockSize);
        const float splitSAH = travCost*halfArea(record.pinfo.geomBounds)+intCost*record.split.splitSAH();
        //PRINT(record.pinfo);
        //PRINT3(record.depth,leafSAH,splitSAH);
        assert(record.pinfo.size() == 0 || leafSAH >= 0 && splitSAH >= 0);
        
        /*! create a leaf node when threshold reached or SAH tells us to stop */
        if (record.pinfo.size() <= minLeafSize || record.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || (record.pinfo.size() <= maxLeafSize && leafSAH <= splitSAH)) {
          //PRINT("leaf");
          createLargeLeaf(record,alloc); return;
        }
        
        /*! initialize child list */
        BuildRecord2<NodeRef> children[MAX_BRANCHING_FACTOR];
        children[0] = record;
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
          //PRINT(bestChild);
          
          /* perform best found split */
          BuildRecord2<NodeRef>& brecord = children[bestChild];
          BuildRecord2<NodeRef> lrecord(record.depth+1);
          BuildRecord2<NodeRef> rrecord(record.depth+1);
          partition<toplevel>(brecord,lrecord,rrecord);
          
          /* find new splits */
          lrecord.split = find<toplevel>(lrecord);
          rrecord.split = find<toplevel>(rrecord);
          //PRINT2(lrecord.split,lrecord.pinfo);
          //PRINT2(rrecord.split,rrecord.pinfo);
          children[bestChild  ] = lrecord;
          children[numChildren] = rrecord;
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        //for (size_t i=0; i<numChildren; i++) PRINT2(i,children[i].split);
        //for (size_t i=0; i<numChildren; i++) PRINT2(i,children[i].pinfo);
        
        /*! create an inner node */
        createNode(record,children,numChildren,alloc);
        
        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          spawn(children[i]);
      }
      
      /*! builder entry function */
      __forceinline NodeRef operator() ()
      {
        /* create initial build record */
        NodeRef root;
        BuildRecord2<NodeRef> br(pinfo,1,&root);
        br.split = find<true>(br);
        
#if 0
        sequential_create_tree(br, createAlloc, 
                               [&](const BuildRecord2<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord2<NodeRef> >& cont) { recurse<false>(br,alloc,cont); });
#else   
        parallel_create_tree<50000,128>(br, createAlloc, 
                                        [&](const BuildRecord2<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord2<NodeRef> >& cont) { recurse<true>(br,alloc,cont); } ,
                                        [&](const BuildRecord2<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord2<NodeRef> >& cont) { recurse<false>(br,alloc,cont); });
#endif
        
        return root;
      }
      
    private:
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
      typedef ObjectPartitionNew<PrimRef> Heuristic;
      BVHBuilderSAH2<Heuristic,NodeRef,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,CreateLeafFunc> builder
        (createAlloc,createNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);
      return builder();
    }

    template<typename NodeRef, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      NodeRef bvh_builder_binned_sah2(CreateAllocFunc createAlloc, CreateNodeFunc createNode, CreateLeafFunc createLeaf, 
                                     PrimRef* prims, const PrimInfo& pinfo, 
                                      const size_t branchingFactor, const size_t maxDepth, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize,
                                      const float travCost, const float intCost)
    {
      const size_t logBlockSize = __bsr(blockSize);
      typedef ObjectPartitionNew<PrimRef> Heuristic;
      assert((blockSize ^ (1L << logBlockSize)) == 0);
      return execute_closure([&]() -> NodeRef {
          BVHBuilderSAH2<Heuristic,NodeRef,decltype(createAlloc()),CreateAllocFunc,CreateNodeFunc,CreateLeafFunc> builder
            (createAlloc,createNode,createLeaf,prims,pinfo,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize,travCost,intCost);
          return builder();
        });
    }
  }
}
