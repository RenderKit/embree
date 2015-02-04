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

#include "geometry/primitive.h"
#include "geometry/bezier1v.h"
#include "builders_new/heuristic_binning.h"

namespace embree
{
  namespace isa
  {
    template<typename CreateAllocFunc, typename CreateAlignedNodeFunc, typename CreateUnalignedNodeFunc, typename CreateLeafFunc>
      class BVH4BuilderHairNew 
    {
      ALIGNED_CLASS;

       static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
       static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth
    
    public:
      
       /*! Constructor. */
       BVH4BuilderHairNew (BezierPrim* prims, 
                           const CreateAllocFunc& createAlloc, const CreateAlignedNodeFunc& createAlignedNode, const CreateUnalignedNodeFunc& createUnalignedNode, const CreateLeafFunc& createLeaf,
                           const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize )
         : prims(prims), 
           branchingFactor(branchingFactor), maxDepth(maxDepth), logBlockSize(logBlockSize), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
           alignedHeuristic(prims), unalignedHeuristic(prims), 
           createAlloc(createAlloc), createAlignedNode(createAlignedNode), createUnalignedNode(createUnalignedNode), createLeaf(createLeaf) {}

       BVH4::NodeRef operator() (const PrimInfo& pinfo) {
        return recurse(1,pinfo,NULL);
       }

    private:
      typedef LinearAllocatorPerThread::ThreadAllocator Allocator;
      
      /*! creates a large leaf that could be larger than supported by the BVH */
      BVH4::NodeRef createLargeLeaf(size_t depth, const PrimInfo& pinfo, FastAllocator::ThreadLocal2* alloc)
      {
        if (depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
      
        /* create leaf for few primitives */
        if (pinfo.size() <= maxLeafSize)
          return createLeaf(depth,pinfo,alloc);
        
        /* fill all children by always splitting the largest one */
        PrimInfo children[MAX_BRANCHING_FACTOR];
        size_t numChildren = 1;
        children[0] = pinfo;
        
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
          __aligned(64) PrimInfo left, right;
          alignedHeuristic.splitFallback(children[bestChild],left,right);
          
          /* add new children left and right */
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /* create node */
        auto node = createAlignedNode(children,numChildren,alignedHeuristic,alloc);
        return BVH4::encodeNode(node);
      }
            
      /*! performs split */
      bool split(const PrimInfo& pinfo, PrimInfo& linfo, PrimInfo& rinfo)
      {
        /* variable to track the SAH of the best splitting approach */
        float bestSAH = inf;
        const float leafSAH = BVH4::intCost*float(pinfo.size())*halfArea(pinfo.geomBounds);
        
        /* perform standard binning in aligned space */
        float alignedObjectSAH = inf;
        HeuristicArrayBinningSAH<BezierPrim>::Split alignedObjectSplit;
        alignedObjectSplit = alignedHeuristic.find(pinfo,0);
        alignedObjectSAH = BVH4::travCostAligned*halfArea(pinfo.geomBounds) + BVH4::intCost*alignedObjectSplit.splitSAH();
        bestSAH = min(alignedObjectSAH,bestSAH);
        
        /* perform standard binning in unaligned space */
        UnalignedHeuristicArrayBinningSAH<BezierPrim>::Split unalignedObjectSplit;
        LinearSpace3fa uspace;
        float unalignedObjectSAH = inf;
        if (alignedObjectSAH > 0.7f*leafSAH) {
          uspace = unalignedHeuristic.computeAlignedSpace(pinfo); 
          const PrimInfo       sinfo = unalignedHeuristic.computePrimInfo(pinfo,uspace);
          unalignedObjectSplit = unalignedHeuristic.find(sinfo,0,uspace);    	
          unalignedObjectSAH = BVH4::travCostUnaligned*halfArea(pinfo.geomBounds) + BVH4::intCost*unalignedObjectSplit.splitSAH();
          bestSAH = min(unalignedObjectSAH,bestSAH);
        }
        
        /* perform aligned split if this is best */
        if (bestSAH == alignedObjectSAH) {
          alignedHeuristic.split(alignedObjectSplit,pinfo,linfo,rinfo);
          return true;
        }
        /* perform unaligned split if this is best */
        else if (bestSAH == unalignedObjectSAH) {
          unalignedHeuristic.split(unalignedObjectSplit,uspace,pinfo,linfo,rinfo);
          return false;
        }
        /* otherwise perform fallback split */
        else {
          alignedHeuristic.deterministic_order(pinfo);
          alignedHeuristic.splitFallback(pinfo,linfo,rinfo);
          return true;
        }
      }
      
      /*! recursive build */
      BVH4::NodeRef recurse(size_t depth, const PrimInfo& pinfo, FastAllocator::ThreadLocal2* alloc)
      {
        if (alloc == NULL) 
          alloc = createAlloc();
	
        PrimInfo children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || pinfo.size() <= minLeafSize) {
          alignedHeuristic.deterministic_order(pinfo);
          return createLargeLeaf(depth,pinfo,alloc);
        }
        
        /* fill all children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = pinfo;
        bool aligned = true;
        
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
            if (area(children[i].geomBounds) > bestArea) { 
              bestArea = area(children[i].geomBounds);
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          PrimInfo left, right;
          aligned &= split(children[bestChild],left,right);
          
          /* add new children left and right */
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor); 
        assert(numChildren > 1);
	
        /* create aligned node */
        if (aligned) 
        {
          auto node = createAlignedNode(children,numChildren,alignedHeuristic,alloc);

          /* spawn tasks or ... */
          if (pinfo.size() > 4096)
          {
            SPAWN_BEGIN;
            for (size_t i=0; i<numChildren; i++) 
              SPAWN(([&,i] { node->child(i) = recurse(depth+1,children[i],NULL); }));
            SPAWN_END;
          }
          /* ... continue sequential */
          else {
            for (size_t i=0; i<numChildren; i++) 
              node->child(i) = recurse(depth+1,children[i],alloc);
          }
          return BVH4::encodeNode(node);
        }
        
        /* create unaligned node */
        else 
        {
          auto node = createUnalignedNode(children,numChildren,unalignedHeuristic,alloc);
          
          /* spawn tasks or ... */
          if (pinfo.size() > 4096)
          {
            SPAWN_BEGIN;
            for (size_t i=0; i<numChildren; i++) 
              SPAWN(([&,i] { node->child(i) = recurse(depth+1,children[i],NULL); }));
            SPAWN_END;
          }
          /* ... continue sequential */
          else {
            for (size_t i=0; i<numChildren; i++) 
              node->child(i) = recurse(depth+1,children[i],alloc);
          }
          return BVH4::encodeNode(node);
        }
      }

    public:
      BezierPrim* prims;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      
      HeuristicArrayBinningSAH<BezierPrim> alignedHeuristic;
      UnalignedHeuristicArrayBinningSAH<BezierPrim> unalignedHeuristic;
      const CreateAllocFunc& createAlloc;
      const CreateAlignedNodeFunc& createAlignedNode;
      const CreateUnalignedNodeFunc& createUnalignedNode;
      const CreateLeafFunc& createLeaf;
    };

    template<typename CreateAllocFunc, typename CreateAlignedNodeFunc, typename CreateUnalignedNodeFunc, typename CreateLeafFunc>
      BVH4::NodeRef bvh_obb_builder_binned_sah_internal (const CreateAllocFunc& createAlloc, const CreateAlignedNodeFunc& createAlignedNode, const CreateUnalignedNodeFunc& createUnalignedNode, const CreateLeafFunc& createLeaf, 
                                                         BezierPrim* prims, const PrimInfo& pinfo,
                                                         const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, const size_t minLeafSize, const size_t maxLeafSize) 
    {
      BVH4BuilderHairNew<CreateAllocFunc,CreateAlignedNodeFunc,CreateUnalignedNodeFunc,CreateLeafFunc> builder(prims,createAlloc,createAlignedNode,createUnalignedNode,createLeaf,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);
      return builder(pinfo);
    }
  }
}
