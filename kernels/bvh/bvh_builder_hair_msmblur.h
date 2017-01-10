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
             typename CreateAlignedNode4DFunc, 
             typename CreateLeafFunc, 
             typename ProgressMonitor>

      class BVHNBuilderHairMBlur
    {
      ALIGNED_CLASS;

      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef FastAllocator::ThreadLocal2* Allocator;
 
      typedef HeuristicMBlurTemporalSplit<BezierPrimMB,RecalculatePrimRef,NUM_TEMPORAL_BINS> HeuristicTemporal;
      typedef HeuristicArrayBinningMB<BezierPrimMB,NUM_OBJECT_BINS> HeuristicBinningSAH;
      typedef UnalignedHeuristicArrayBinningMB<BezierPrimMB,NUM_OBJECT_BINS> UnalignedHeuristicBinningSAH;
      //typedef HeuristicStrandSplit<BezierPrimMB> HeuristicStrandSplitSAH;

      static const size_t MAX_BRANCHING_FACTOR =  8;         //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;         //!< create balanced tree if we are that many levels before the maximal tree depth
      static const size_t SINGLE_THREADED_THRESHOLD = 4096;  //!< threshold to switch to single threaded build

      static const size_t travCostAligned = 1;
      static const size_t travCostUnaligned = 5;
      static const size_t intCost = 6;

    public:
      
      BVHNBuilderHairMBlur (BezierPrimMB* prims,
                            const CreateAllocFunc& createAlloc, 
                            const CreateAlignedNodeFunc& createAlignedNode, 
                            const CreateUnalignedNodeFunc& createUnalignedNode, 
                            const CreateAlignedNode4DFunc& createAlignedNode4D, 
                            const CreateLeafFunc& createLeaf,
                            const ProgressMonitor& progressMonitor,
                            const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, 
                            const size_t minLeafSize, const size_t maxLeafSize )
        : createAlloc(createAlloc), 
        createAlignedNode(createAlignedNode), 
        createUnalignedNode(createUnalignedNode), 
        createAlignedNode4D(createAlignedNode4D),
        createLeaf(createLeaf),
        progressMonitor(progressMonitor),
        prims(prims), 
        branchingFactor(branchingFactor), maxDepth(maxDepth), logBlockSize(logBlockSize), 
        minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
        alignedHeuristic(prims), unalignedHeuristic(prims)/*, strandHeuristic(prims)*/ {}
       
      /*! entry point into builder */
      NodeRef operator() (const PrimInfoMB& pinfo) {
        NodeRef root = recurse(1,pinfo,nullptr,true);
        _mm_mfence(); // to allow non-temporal stores during build
        return root;
      }
      
    private:

      void deterministic_order(const SetMB& set) 
      {
        /* required as parallel partition destroys original primitive order */
        BezierPrimMB* prims = set.prims->data();
        std::sort(&prims[set.object_range.begin()],&prims[set.object_range.end()]);
      }

      void splitFallback(const SetMB& set, PrimInfoMB& linfo, SetMB& lset, PrimInfoMB& rinfo, SetMB& rset) // FIXME: also perform time split here?
      {
        mvector<PrimRefMB>& prims = *set.prims;
        
        const size_t begin = set.object_range.begin();
        const size_t end   = set.object_range.end();
        const size_t center = (begin + end)/2;
        
        linfo = empty;
        for (size_t i=begin; i<center; i++)
          linfo.add_primref(prims[i]);
        linfo.begin = begin; linfo.end = center; linfo.time_range = set.time_range;
        
        rinfo = empty;
        for (size_t i=center; i<end; i++)
          rinfo.add_primref(prims[i]);	
        rinfo.begin = center; rinfo.end = end; rinfo.time_range = set.time_range;
        
        new (&lset) SetMB(set.prims,range<size_t>(begin,center),set.time_range);
        new (&rset) SetMB(set.prims,range<size_t>(center,end  ),set.time_range);
      }

      /*! creates a large leaf that could be larger than supported by the BVH */
      NodeRef createLargeLeaf(size_t depth, const PrimInfoMB& pinfo, Allocator alloc)
      {
        /* this should never occur but is a fatal error */
        if (depth > maxDepth) 
          throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
        
        /* create leaf for few primitives */
        if (pinfo.size() <= maxLeafSize)
          return createLeaf(depth,pinfo,alloc);
        
        /* fill all children by always splitting the largest one */
        PrimInfoMB children[MAX_BRANCHING_FACTOR];
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
          __aligned(64) PrimInfoMB left, right;
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
      void split(const PrimInfoMB& pinfo, PrimInfoMB& linfo, PrimInfoMB& rinfo, bool& aligned, bool& timesplit)
      {
        /* variable to track the SAH of the best splitting approach */
        float bestSAH = inf;
        const float leafSAH = intCost*float(pinfo.size())*halfArea(pinfo.geomBounds);
        
        /* perform standard binning in aligned space */
        float alignedObjectSAH = inf;
        HeuristicBinningMB::Split alignedObjectSplit;
        alignedObjectSplit = alignedHeuristic.find(set,pinfo,0);
        alignedObjectSAH = travCostAligned*halfArea(pinfo.geomBounds) + intCost*alignedObjectSplit.splitSAH();
        bestSAH = min(alignedObjectSAH,bestSAH);
        
        /* perform standard binning in unaligned space */
        UnalignedHeuristicBinningMB::Split unalignedObjectSplit;
        LinearSpace3fa uspace;
        float unalignedObjectSAH = inf;
        if (alignedObjectSAH > 0.7f*leafSAH) {
          uspace = unalignedHeuristic.computeAlignedSpaceMB(scene,set); 
          const PrimInfoMB sinfo = unalignedHeuristic.computePrimInfoMB(scene,set,uspace);
          unalignedObjectSplit = unalignedHeuristic.find(set,sinfo,0,uspace);    	
          unalignedObjectSAH = travCostUnaligned*halfArea(pinfo.geomBounds) + intCost*unalignedObjectSplit.splitSAH();
          bestSAH = min(unalignedObjectSAH,bestSAH);
        }

        /* do temporal splits only if the the time range is big enough */
        float temporal_split_sah = inf;
        HeuristicTemporal::Split temporal_split;
        if (set.time_range.size() > 1.01f/float(pinfo.max_num_time_segments)) {
          temporal_split = heuristicTemporalSplit.find(set, pinfo, 0);
          temporal_split_sah = temporal_split.splitSAH();
        }
        
        /* perform splitting into two strands */
        /*HeuristicStrandSplitSAH::Split strandSplit;
        float strandSAH = inf;
        if (alignedObjectSAH > 0.6f*leafSAH) {
          strandSplit = strandHeuristic.find(pinfo);
          strandSAH = travCostUnaligned*halfArea(pinfo.geomBounds) + intCost*strandSplit.splitSAH();
          bestSAH = min(strandSAH,bestSAH);
          }*/
        
        /* perform time split if this is the best */
        if (bestSAH == timeSplitSAH) {
          timeSplitHeuristic.split(timeSplit,pinfo,set,linfo,lset,rinfo,rset);
          timesplit = true;
        }
        /* perform aligned split if this is best */
        else if (bestSAH == alignedObjectSAH) {
          alignedHeuristic.split(alignedObjectSplit,pinfo,set,linfo,lset,rinfo,rset);
        }
        /* perform unaligned split if this is best */
        else if (bestSAH == unalignedObjectSAH) {
          unalignedHeuristic.split(unalignedObjectSplit,uspace,pinfo,set,linfo,lset,rinfo,rset);
          aligned = false;
        }
        /* perform strand split if this is best */
        /*else if (bestSAH == strandSAH) {
          strandHeuristic.split(strandSplit,pinfo,linfo,rinfo);
          aligned = false;
          }*/
        /* otherwise perform fallback split */
        else {
          deterministic_order(set);
          splitFallback(pinfo,set,linfo,lset,rinfo,rset);
        }
      }
      
      /*! recursive build */
      NodeRef recurse(size_t depth, const PrimInfoMB& pinfo, Allocator alloc, bool toplevel)
      {
        if (alloc == nullptr) 
          alloc = createAlloc();

        /* call memory monitor function to signal progress */
        if (toplevel && pinfo.size() <= SINGLE_THREADED_THRESHOLD)
          progressMonitor(pinfo.size());
	
        PrimInfoMB children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || pinfo.size() <= minLeafSize) {
          alignedHeuristic.deterministic_order(pinfo);
          return createLargeLeaf(depth,pinfo,alloc);
        }
        
        /* fill all children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = pinfo;
        bool aligned = true;
        bool timesplit = false;
        
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
          PrimInfoMB left, right;
          split(children[bestChild],left,right,aligned,timesplit);
          
          /* add new children left and right */
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor); 
        assert(numChildren > 1);
	
        /* create time split node */
        if (timesplit)
        {
          auto node = createAlignedNode4D(children,numChildren,alloc);

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

        /* create aligned node */
        else if (aligned) 
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
      const CreateAlignedNode4DFunc& createAlignedNode4D;
      const CreateLeafFunc& createLeaf;
      const ProgressMonitor& progressMonitor;

    private:
      BezierPrimMB* prims;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
  
    private:
      HeuristicBinningSAH alignedHeuristic;
      UnalignedHeuristicBinningSAH unalignedHeuristic;
      //HeuristicStrandSplitSAH strandHeuristic;
      HeuristicTemporal heuristicTemporalSplit;
    };

    template<int N,
             typename CreateAllocFunc,
             typename CreateAlignedNodeFunc, 
             typename CreateUnalignedNodeFunc, 
             typename CreateAlignedNode4DFunc, 
             typename CreateLeafFunc, 
             typename ProgressMonitor>

      typename BVHN<N>::NodeRef bvh_obb_builder_binned_sah_mblur (const CreateAllocFunc& createAlloc,
                                                                  const CreateAlignedNodeFunc& createAlignedNode, 
                                                                  const CreateUnalignedNodeFunc& createUnalignedNode, 
                                                                  const CreateAlignedNode4DFunc& createAlignedNode4D, 
                                                                  const CreateLeafFunc& createLeaf, 
                                                                  const ProgressMonitor& progressMonitor,
                                                                  BezierPrimMB* prims, 
                                                                  const PrimInfoMB& pinfo,
                                                                  const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, 
                                                                  const size_t minLeafSize, const size_t maxLeafSize) 
    {
      typedef BVHNBuilderHairMBlur<N,CreateAllocFunc,CreateAlignedNodeFunc,CreateUnalignedNodeFunc,CreateAlignedNode4DFunc,CreateLeafFunc,ProgressMonitor> Builder;
      Builder builder(prims,createAlloc,createAlignedNode,createUnalignedNode,createAlignedNode4D,createLeaf,progressMonitor,
                      branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);
      return builder(pinfo);
    }
  }
}
