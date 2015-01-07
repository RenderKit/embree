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

namespace embree
{
  namespace isa
  {
    template<typename NodeRef>
      class BVHBuilderGeneric
    {
      static const size_t MAX_BRANCHING_FACTOR = 16;
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;

    public:
      
      class __aligned(64) BuildRecord : public PrimInfo
      {
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
      
    public:
      
      BVHBuilderGeneric (PrimRef* prims, const PrimInfo& pinfo,
                         const size_t branchingFactor, const size_t maxDepth, 
                         const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize)
        : prims(prims), pinfo(pinfo), branchingFactor(branchingFactor), logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), maxDepth(maxDepth)
      {
        if (branchingFactor > MAX_BRANCHING_FACTOR)
          THROW_RUNTIME_ERROR("bvh4_builder: branching factor too large");
      }

      void splitFallback(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
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

      template<typename CreateNodeFunc, typename CreateLeafFunc>
        void createLargeLeaf(CreateNodeFunc& createNode, CreateLeafFunc& createLeaf, BuildRecord& current)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize) {
          *current.parent = createLeaf(current,prims);
          return;
        }

        /* fill all children by always splitting the largest one */
        BuildRecord children[MAX_BRANCHING_FACTOR];
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
          __aligned(64) BuildRecord left, right;
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
          createLargeLeaf(createNode,createLeaf,children[i]);
      }
            
      __forceinline void splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
      {
        /* calculate binning function */
        PrimInfo pinfo(current.size(),current.geomBounds,current.centBounds);
        ObjectPartition::Split split = ObjectPartition::find(prims,current.begin,current.end,pinfo,logBlockSize);
        
        /* if we cannot find a valid split, enforce an arbitrary split */
        if (unlikely(!split.valid())) splitFallback(current,leftChild,rightChild);
        
        /* partitioning of items */
        else split.partition(prims, current.begin, current.end, leftChild, rightChild);
      }

      template<typename CreateNodeFunc, typename CreateLeafFunc>
        void recurse(CreateNodeFunc& createNode, CreateLeafFunc& createLeaf, BuildRecord& current)
      {
        __aligned(64) BuildRecord children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
          createLargeLeaf(createNode,createLeaf,current);
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
          __aligned(64) BuildRecord left, right;
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
          createLargeLeaf(createNode,createLeaf,current);
          return;
        }
        
        /* create node */
        *current.parent = createNode(children,numChildren);

        /* recurse into each child */
        for (size_t i=0; i<numChildren; i++) 
          recurse(createNode,createLeaf,children[i]);
      }
      
      /*! builder entry function */
      template<typename CreateNodeFunc, typename CreateLeafFunc>
        NodeRef operator() (CreateNodeFunc& createNode, CreateLeafFunc& createLeaf)
      {
        /* create initial build record */
        NodeRef root;
        BuildRecord br;
        br.init(pinfo,0,pinfo.size());
        br.depth = 1;
        br.parent = &root;
        
        /* build BVH */
        recurse(createNode,createLeaf,br);
        return root;
      }

    private:
      PrimRef* prims;
      const PrimInfo pinfo;
      const size_t branchingFactor;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const size_t maxDepth;
    };
  }
}
