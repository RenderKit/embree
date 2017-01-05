// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "bvh.h"
#include "../geometry/primitive.h"
#include "../builders/bvh_builder_sah.h"
#include "../builders/heuristic_binning_array_aligned.h"
#include "../builders/heuristic_binning_array_unaligned.h"
#include "../builders/heuristic_strand_array.h"

namespace embree
{
  namespace isa
  {
    template<int N,
             typename CreateAllocFunc,
             typename CreateAlignedNodeFunc, 
             typename CreateUnalignedNodeFunc, 
             typename CreateLeafFunc, 
             typename ProgressMonitor>

      class BVHNBuilderHair
    {
      ALIGNED_CLASS;

      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef FastAllocator::ThreadLocal2* Allocator;
      typedef HeuristicArrayBinningSAH<BezierPrim,NUM_OBJECT_BINS> HeuristicBinningSAH;

      static const size_t MAX_BRANCHING_FACTOR = 16;         //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;         //!< create balanced tree if we are that many levels before the maximal tree depth
      static const size_t SINGLE_THREADED_THRESHOLD = 4096;  //!< threshold to switch to single threaded build

      static const size_t travCostAligned = 1;
      static const size_t travCostUnaligned = 5;
      static const size_t intCost = 6;

    public:
      
      BVHNBuilderHair (BezierPrim* prims,
                       const CreateAllocFunc& createAlloc, 
                       const CreateAlignedNodeFunc& createAlignedNode, 
                       const CreateUnalignedNodeFunc& createUnalignedNode, 
                       const CreateLeafFunc& createLeaf,
                       const ProgressMonitor& progressMonitor,
                       const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, 
                       const size_t minLeafSize, const size_t maxLeafSize )
        : createAlloc(createAlloc), 
        createAlignedNode(createAlignedNode), 
        createUnalignedNode(createUnalignedNode), 
        createLeaf(createLeaf),
        progressMonitor(progressMonitor),
        prims(prims), 
        branchingFactor(branchingFactor), maxDepth(maxDepth), logBlockSize(logBlockSize), 
        minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
        alignedHeuristic(prims), unalignedHeuristic(prims), strandHeuristic(prims) {}
       
      /*! entry point into builder */
      NodeRef operator() (const PrimInfo& pinfo) {
        NodeRef root = recurse(1,pinfo,nullptr,true);
        _mm_mfence(); // to allow non-temporal stores during build
        return root;
      }
      
    private:
      
      /*! creates a large leaf that could be larger than supported by the BVH */
      NodeRef createLargeLeaf(size_t depth, const PrimInfo& pinfo, Allocator alloc)
      {
        /* this should never occur but is a fatal error */
        if (depth > maxDepth) 
          throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
        
        /* create leaf for few primitives */
        if (pinfo.size() <= maxLeafSize)
          return createLeaf(depth,pinfo,alloc);
        
        /* fill all children by always splitting the largest one */
        PrimInfo children[MAX_BRANCHING_FACTOR];
        unsigned numChildren = 1;
        children[0] = pinfo;
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          size_t bestSize = 0;
          for (unsigned i=0; i<numChildren; i++)
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
        return BVH::encodeNode(node);
      }
            
      /*! performs split */
      bool split(const PrimInfo& pinfo, PrimInfo& linfo, PrimInfo& rinfo)
      {
        /* variable to track the SAH of the best splitting approach */
        float bestSAH = inf;
        const float leafSAH = intCost*float(pinfo.size())*halfArea(pinfo.geomBounds);
        
        /* perform standard binning in aligned space */
        float alignedObjectSAH = inf;
        HeuristicBinningSAH::Split alignedObjectSplit;
        alignedObjectSplit = alignedHeuristic.find(pinfo,0);
        alignedObjectSAH = travCostAligned*halfArea(pinfo.geomBounds) + intCost*alignedObjectSplit.splitSAH();
        bestSAH = min(alignedObjectSAH,bestSAH);
        
        /* perform standard binning in unaligned space */
        UnalignedHeuristicArrayBinningSAH<BezierPrim>::Split unalignedObjectSplit;
        LinearSpace3fa uspace;
        float unalignedObjectSAH = inf;
        if (alignedObjectSAH > 0.7f*leafSAH) {
          uspace = unalignedHeuristic.computeAlignedSpace(pinfo); 
          const PrimInfo       sinfo = unalignedHeuristic.computePrimInfo(pinfo,uspace);
          unalignedObjectSplit = unalignedHeuristic.find(sinfo,0,uspace);    	
          unalignedObjectSAH = travCostUnaligned*halfArea(pinfo.geomBounds) + intCost*unalignedObjectSplit.splitSAH();
          bestSAH = min(unalignedObjectSAH,bestSAH);
        }

        /* perform splitting into two strands */
        HeuristicStrandSplit::Split strandSplit;
        float strandSAH = inf;
        if (alignedObjectSAH > 0.6f*leafSAH) {
          strandSplit = strandHeuristic.find(pinfo);
          strandSAH = travCostUnaligned*halfArea(pinfo.geomBounds) + intCost*strandSplit.splitSAH();
          bestSAH = min(strandSAH,bestSAH);
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
        /* perform strand split if this is best */
        else if (bestSAH == strandSAH) {
          strandHeuristic.split(strandSplit,pinfo,linfo,rinfo);
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
      NodeRef recurse(size_t depth, const PrimInfo& pinfo, Allocator alloc, bool toplevel)
      {
        if (alloc == nullptr) 
          alloc = createAlloc();

        /* call memory monitor function to signal progress */
        if (toplevel && pinfo.size() <= SINGLE_THREADED_THRESHOLD)
          progressMonitor(pinfo.size());
	
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
          ssize_t bestChild = -1;
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
          if (pinfo.size() > SINGLE_THREADED_THRESHOLD)
          {
            parallel_for(size_t(0), numChildren, [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  node->child(i) = recurse(depth+1,children[i],nullptr,true); 
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
          }
          /* ... continue sequential */
          else {
            for (size_t i=0; i<numChildren; i++) 
              node->child(i) = recurse(depth+1,children[i],alloc,false);
          }
          return BVH::encodeNode(node);
        }
        
        /* create unaligned node */
        else 
        {
          auto node = createUnalignedNode(children,numChildren,unalignedHeuristic,alloc);
          
          /* spawn tasks or ... */
          if (pinfo.size() > SINGLE_THREADED_THRESHOLD)
          {

            parallel_for(size_t(0), numChildren, [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                    node->child(i) = recurse(depth+1,children[i],nullptr,true);
                    _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
          }
          /* ... continue sequentially */
          else
          {
            for (size_t i=0; i<numChildren; i++) 
              node->child(i) = recurse(depth+1,children[i],alloc,false);
          }
          return BVH::encodeNode(node);
        }
      }
    
    private:      
      const CreateAllocFunc& createAlloc;
      const CreateAlignedNodeFunc& createAlignedNode;
      const CreateUnalignedNodeFunc& createUnalignedNode;
      const CreateLeafFunc& createLeaf;
      const ProgressMonitor& progressMonitor;

    private:
      BezierPrim* prims;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
  
    private:
      HeuristicBinningSAH alignedHeuristic;
      UnalignedHeuristicArrayBinningSAH<BezierPrim> unalignedHeuristic;
      HeuristicStrandSplit strandHeuristic;
    };

    template<int N,
             typename CreateAllocFunc,
             typename CreateAlignedNodeFunc, 
             typename CreateUnalignedNodeFunc, 
             typename CreateLeafFunc, 
             typename ProgressMonitor>

      typename BVHN<N>::NodeRef bvh_obb_builder_binned_sah (const CreateAllocFunc& createAlloc,
                                                const CreateAlignedNodeFunc& createAlignedNode, 
                                                const CreateUnalignedNodeFunc& createUnalignedNode, 
                                                const CreateLeafFunc& createLeaf, 
                                                const ProgressMonitor& progressMonitor,
                                                BezierPrim* prims, 
                                                const PrimInfo& pinfo,
                                                const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, 
                                                const size_t minLeafSize, const size_t maxLeafSize) 
    {
      typedef BVHNBuilderHair<N,CreateAllocFunc,CreateAlignedNodeFunc,CreateUnalignedNodeFunc,CreateLeafFunc,ProgressMonitor> Builder;
      Builder builder(prims,createAlloc,createAlignedNode,createUnalignedNode,createLeaf,progressMonitor,
                      branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);
      return builder(pinfo);
    }
  }
}
