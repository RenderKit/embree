// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "builders/heuristic_object_partition.h"
#include "builders/workstack.h"

#include "algorithms/parallel_create_tree.h"

namespace embree
{
  namespace isa
  {
    template<typename NodeRef>
     class __aligned(64) BuildRecord : public PrimInfo
      {
      public:
	unsigned depth;         //!< depth from the root of the tree
	float sArea;
	NodeRef* parent; 
	
        BuildRecord() {}

        BuildRecord(const PrimInfo& pinfo, const size_t depth, NodeRef* parent) 
          : PrimInfo(pinfo), depth(depth), parent(parent), sArea(area(pinfo.geomBounds)) {}

#if defined(_MSC_VER)
        BuildRecord& operator=(const BuildRecord &arg) { 
          memcpy(this, &arg, sizeof(BuildRecord));    
          return *this;
        }
#endif

        //__forceinline bool final() const {
        //  return size() < 128;
        //}

	__forceinline void init(size_t depth)
	{
          parent = NULL;
	  this->depth = depth;
	  sArea = area(geomBounds);
	}

	__forceinline void init(const CentGeomBBox3fa& _bounds, const size_t _begin, const size_t _end)
	{
          parent = NULL;
	  geomBounds = _bounds.geomBounds;
	  centBounds = _bounds.centBounds;
	  begin  = _begin;
	  end    = _end;
	  sArea = area(geomBounds);
	}
	
	__forceinline float sceneArea() {
	  return sArea;
	}
	
	__forceinline bool operator<(const BuildRecord &br) const { return size() < br.size(); } 
	__forceinline bool operator>(const BuildRecord &br) const { return size() > br.size(); } 
	
	struct Greater {
	  bool operator()(const BuildRecord& a, const BuildRecord& b) {
	    return a > b;
	  }
	};
      };

    template<typename NodeRef, typename Allocator, typename CreateAllocFunc, typename CreateNodeFunc, typename CreateLeafFunc>
      class BVHBuilderGeneric
    {
      static const size_t MAX_BRANCHING_FACTOR = 16;
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;

    public:

      BVHBuilderGeneric (CreateAllocFunc& createAlloc, CreateNodeFunc& createNode, CreateLeafFunc& createLeaf,
                         PrimRef* prims, const PrimInfo& pinfo,
                         const size_t branchingFactor, const size_t maxDepth, 
                         const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize)
        : parallelBinner(NULL),
          createAlloc(createAlloc), createNode(createNode), createLeaf(createLeaf), 
          prims(prims), pinfo(pinfo), 
          branchingFactor(branchingFactor), maxDepth(maxDepth),
          logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh4_builder: branching factor too large");
      }

      void splitFallback(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        const size_t center = (current.begin + current.end)/2;
        
        CentGeomBBox3fa left; left.reset();
        for (size_t i=current.begin; i<center; i++)
          left.extend(prims[i].bounds());
        leftChild.init(left,current.begin,center);
        
        CentGeomBBox3fa right; right.reset();
        for (size_t i=center; i<current.end; i++)
          right.extend(prims[i].bounds());	
        rightChild.init(right,center,current.end);
      }

      __forceinline void splitSequential(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        /* calculate binning function */
        PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        ObjectPartition::Split split = ObjectPartition::find(prims,current.begin,current.end,pinfo,logBlockSize);
        
        /* if we cannot find a valid split, enforce an arbitrary split */
        if (unlikely(!split.valid())) splitFallback(current,leftChild,rightChild);
        
        /* partitioning of items */
        else split.partition(prims, current.begin, current.end, leftChild, rightChild);
      }

      void splitParallel(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild, Allocator& alloc)
      {
        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t threadCount = scheduler->getNumThreads();

        /* use primitive array temporarily for parallel splits */
        PrimInfo pinfo(current.begin,current.end,current.geomBounds,current.centBounds);
        
        PrimRef* tmp = (PrimRef*) alloc.curPtr(); // FIXME: malloc(1) should be malloc(0) or getPtr()

        /* parallel binning of centroids */
        const float sah = parallelBinner->find(pinfo,prims,tmp,logBlockSize,0,threadCount,scheduler); // FIXME: hardcoded threadIndex=0
        
        /* if we cannot find a valid split, enforce an arbitrary split */
        if (unlikely(sah == float(inf))) splitFallback(current,leftChild,rightChild);
        
        /* parallel partitioning of items */
        else parallelBinner->partition(pinfo,tmp,prims,leftChild,rightChild,0,threadCount,scheduler);
      }

      void createLargeLeaf(const BuildRecord<NodeRef>& current, Allocator& nodeAlloc, Allocator& leafAlloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize) {
          *current.parent = createLeaf(current,prims,leafAlloc);
          return;
        }

        /* fill all children by always splitting the largest one */
        BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        size_t numChildren = 1;
        children[0] = current;
        
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
          splitFallback(children[bestChild],left,right);
          
          /* add new children left and right */
          left.init(current.depth+1); 
          right.init(current.depth+1);
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor);

        /* create node */
        *current.parent = createNode(children,numChildren,nodeAlloc);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          createLargeLeaf(children[i],nodeAlloc,leafAlloc);
      }

      template<bool toplevel, typename Spawn>
        inline void recurse(const BuildRecord<NodeRef>& current, Allocator& alloc, Spawn& spawn)
      {
        __aligned(64) BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
          createLargeLeaf(current,alloc,alloc);
          return;
        }
        
        /* fill all children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = current;
        
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
            if (children[i].sceneArea() > bestArea) { 
              bestArea = children[i].sceneArea();
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
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /* create leaf node if no split is possible */
        if (numChildren == 1) {
          createLargeLeaf(current,alloc,alloc);
          return;
        }
        
        /* create node */
        *current.parent = createNode(children,numChildren,alloc);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          spawn(children[i]);
      }

      /*! builder entry function */
      NodeRef operator() ()
      {
        /* create initial build record */
        NodeRef root;
        BuildRecord<NodeRef> br(pinfo,1,&root);
        
#if 0
        sequential_create_tree(br, createAlloc, 
          [&](const BuildRecord<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord<NodeRef> >& cont) { recurse<false>(br,alloc,cont); });
#else   
        parallelBinner = new ObjectPartition::ParallelBinner;
        parallel_create_tree<50000,128>(br, createAlloc, 
          [&](const BuildRecord<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord<NodeRef> >& cont) { recurse<true>(br,alloc,cont); } ,
          [&](const BuildRecord<NodeRef>& br, Allocator& alloc, ParallelContinue<BuildRecord<NodeRef> >& cont) { recurse<false>(br,alloc,cont); });
        delete parallelBinner;
#endif

        return root;
      }

    private:
      ObjectPartition::ParallelBinner* parallelBinner;
      CreateAllocFunc& createAlloc;
      CreateNodeFunc& createNode;
      CreateLeafFunc& createLeaf;
      PrimRef* prims;
      const PrimInfo& pinfo;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
    };
  }
}
