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

#include "algorithms/parallel_continue.h"

namespace embree
{
  namespace isa
  {
    template<typename NodeRef>
     class __aligned(64) BuildRecord : public PrimInfo
      {
        static const size_t THRESHOLD_FOR_SUBTREE_RECURSION = 128;

      public:
	unsigned depth;         //!< depth from the root of the tree
	float sArea;
	NodeRef* parent; 
	
        BuildRecord() {}

#if defined(_MSC_VER)
        BuildRecord& operator=(const BuildRecord &arg) { 
          memcpy(this, &arg, sizeof(BuildRecord));    
          return *this;
        }
#endif

        __forceinline bool final() const {
          return size() < THRESHOLD_FOR_SUBTREE_RECURSION;
        }

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

    template<typename NodeRef, typename CreateNodeFunc, typename CreateLeafFunc>
      class BVHBuilderGeneric
    {
      static const size_t MAX_BRANCHING_FACTOR = 16;
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;
      static const size_t SIZE_WORK_STACK = 64;
      static const size_t THRESHOLD_FOR_SUBTREE_RECURSION = 128;
      static const size_t THRESHOLD_FOR_SINGLE_THREADED = 50000; 

    public:

      struct GlobalState
      {
        ALIGNED_CLASS;
      public:

        GlobalState () : numThreads(getNumberOfLogicalThreads()) {
	  threadStack = new WorkStack<BuildRecord<NodeRef>,SIZE_WORK_STACK>[numThreads]; 
        }
        
        ~GlobalState () {
          delete[] threadStack;
        }

      public:
	size_t numThreads;
	WorkHeap<BuildRecord<NodeRef>> heap;
        __aligned(64) WorkStack<BuildRecord<NodeRef>,SIZE_WORK_STACK>* threadStack;
        ObjectPartition::ParallelBinner parallelBinner;
      };
      
    public:
      
      BVHBuilderGeneric (CreateNodeFunc& createNode, CreateLeafFunc& createLeaf,
                         PrimRef* prims, PrimRef* tmp, const PrimInfo& pinfo,
                         const size_t branchingFactor, const size_t maxDepth, 
                         const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize)
        : createNode(createNode), createLeaf(createLeaf), 
        state(NULL), prims(prims), tmp(tmp), pinfo(pinfo), 
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

      void createLargeLeaf(const BuildRecord<NodeRef>& current)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize) {
          *current.parent = createLeaf(current,prims);
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
        *current.parent = createNode(children,numChildren);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          createLargeLeaf(children[i]);
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

      void splitParallel(const BuildRecord<NodeRef>& current, BuildRecord<NodeRef>& leftChild, BuildRecord<NodeRef>& rightChild)
      {
        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t threadCount = scheduler->getNumThreads();

        /* use primitive array temporarily for parallel splits */
        PrimInfo pinfo(current.begin,current.end,current.geomBounds,current.centBounds);
        
        /* parallel binning of centroids */
        const float sah = state->parallelBinner.find(pinfo,prims,tmp,logBlockSize,0,threadCount,scheduler);
        
        /* if we cannot find a valid split, enforce an arbitrary split */
        if (unlikely(sah == float(inf))) splitFallback(current,leftChild,rightChild);
        
        /* parallel partitioning of items */
        else state->parallelBinner.partition(pinfo,tmp,prims,leftChild,rightChild,0,threadCount,scheduler);
      }

      template<typename Spawn>
        inline void recurse(const BuildRecord<NodeRef>& current, Spawn& spawn)
      {
        __aligned(64) BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
          createLargeLeaf(current);
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
          splitSequential(children[bestChild],left,right);
          
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
          createLargeLeaf(current);
          return;
        }
        
        /* create node */
        *current.parent = createNode(children,numChildren);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          spawn(children[i]);
      }

      struct Recursion { 
        BVHBuilderGeneric* parent;
        __forceinline Recursion(BVHBuilderGeneric* parent) : parent(parent) {}
        __forceinline void operator() (BuildRecord<NodeRef>& br) { parent->recurse(br,*this); } 
      };

      void recurseTop(const BuildRecord<NodeRef>& current)
      {
        __aligned(64) BuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        //if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
        //  createLargeLeaf(current);
        //  return;
        //}
        
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
          splitParallel(children[bestChild],left,right);
          
          /* add new children left and right */
          left.init(current.depth+1); 
          right.init(current.depth+1);
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /* create leaf node if no split is possible */
        //if (numChildren == 1) {
        //  createLargeLeaf(current);
        //  return;
        //}
        
        /* create node */
        *current.parent = createNode(children,numChildren);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          state->heap.push(children[i]);
      }

      void buildSubTrees(size_t threadID, size_t numThreads)
    {
      //__aligned(64) Allocator nodeAlloc(&bvh->alloc);
      //__aligned(64) Allocator leafAlloc(&bvh->alloc);
      
      while (true) 
      {
	BuildRecord<NodeRef> br;
	if (!state->heap.pop(br))
        {
          /* global work queue empty => try to steal from neighboring queues */	  
          bool success = false;
          for (size_t i=0; i<numThreads; i++)
          {
            if (state->threadStack[(threadID+i)%numThreads].pop(br)) {
              success = true;
              break;
            }
          }
          /* found nothing to steal ? */
          if (!success) break; // FIXME: may loose threads
        }
        
        /* process local work queue */
        //CreateNodeFunc& createNode = *(CreateNodeFunc*)cNode;
        //CreateLeafFunc& createLeaf = *(CreateLeafFunc*)cLeaf;

        /*else if (mode == RECURSE_PARALLEL && current.size() > THRESHOLD_FOR_SUBTREE_RECURSION) {
        if (!state->threadStack[threadID].push(current))
          recurse(current,nodeAlloc,leafAlloc,RECURSE_SEQUENTIAL,threadID,numThreads);
      }
      else
      recurse(current,nodeAlloc,leafAlloc,mode,threadID,numThreads);*/

        Recursion recurse(this); recurse(br);
	while (state->threadStack[threadID].pop(br)) {
          Recursion recurse(this); recurse(br);
        }
      }
      _mm_sfence(); // make written leaves globally visible
    }
      
      static void task_buildSubTrees (void* data, const size_t threadID, const size_t numThreads) {
        ((BVHBuilderGeneric*)data)->buildSubTrees(threadID,numThreads);                     
      }

      /*! builder entry function */
      NodeRef operator() ()
      {
        /* create initial build record */
        NodeRef root;
        BuildRecord<NodeRef> br;
        br.init(pinfo,0,pinfo.size());
        br.depth = 1;
        br.parent = &root;
        
#if 0
        /* build BVH */
        Recursion recurse(this); recurse(br);

#endif

#if 1
        /* push initial build record to global work stack */
        state = new GlobalState;
        state->heap.reset();
        state->heap.push(br);

        /* work in multithreaded toplevel mode until sufficient subtasks got generated */
        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t threadCount = scheduler->getNumThreads();
        while (state->heap.size() < 2*threadCount)
        {
          BuildRecord<NodeRef> br;

          /* pop largest item for better load balancing */
          if (!state->heap.pop(br)) 
            break;
          
          /* guarantees to create no leaves in this stage */
          if (br.size() <= max(minLeafSize,THRESHOLD_FOR_SINGLE_THREADED)) {
            state->heap.push(br);
            break;
          }
          
          recurseTop(br);
        }
        _mm_sfence(); // make written leaves globally visible
        
        std::sort(state->heap.begin(),state->heap.end(),BuildRecord<NodeRef>::Greater());

#if 1
        parallel_continue( state->heap.begin(), state->heap.size(), [&](const BuildRecord<NodeRef>& br, ParallelContinue<BuildRecord<NodeRef> >& cont) {
          recurse(br,cont);
        });
#else
        /* now process all created subtasks on multiple threads */
        scheduler->dispatchTask(task_buildSubTrees, this, 0, threadCount ); // FIXME: threadIdx=0

#endif

        delete state; state = NULL;
#endif

        return root;
      }

    private:
      CreateNodeFunc& createNode;
      CreateLeafFunc& createLeaf;
      GlobalState* state;
      PrimRef* prims;
      PrimRef* tmp;
      const PrimInfo pinfo;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
    };
  }
}
