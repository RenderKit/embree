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
#include "../builders/bvh_builder_msmblur.h"
#include "../builders/heuristic_binning_array_aligned.h"
#include "../builders/heuristic_binning_array_unaligned.h"
#include "../builders/heuristic_timesplit_array.h"

namespace embree
{
  namespace isa
  {
      struct BuildRecord2
      {
      public:
	__forceinline BuildRecord2 () {}
        
        __forceinline BuildRecord2 (size_t depth) 
          : depth(depth), pinfo(empty) {}
        
        __forceinline BuildRecord2 (const PrimInfoMB& pinfo, const SetMB& prims, size_t depth = 0) 
          : depth(depth), prims(prims), pinfo(pinfo) {}

        __forceinline size_t size() const { return prims.object_range.size(); }
        
      public:
	size_t depth;       //!< depth of the root of this subtree
	SetMB prims;        //!< the list of primitives
	PrimInfoMB pinfo;   //!< bounding info of primitives.
      };

    template<int N,
             typename RecalculatePrimRef, 
             typename CreateAllocFunc,
             typename CreateAlignedNodeFunc, 
             typename CreateUnalignedNodeFunc, 
             typename CreateAlignedNode4DFunc, 
             typename CreateLeafFunc, 
             typename ProgressMonitor>
      
      class BVHNBuilderHairMBlur
    {
      ALIGNED_CLASS;

      static const size_t MAX_BRANCHING_FACTOR =  8;         //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;         //!< create balanced tree if we are that many levels before the maximal tree depth
      static const size_t SINGLE_THREADED_THRESHOLD = 4096;  //!< threshold to switch to single threaded build

      static const size_t travCostAligned = 1;
      static const size_t travCostUnaligned = 5;
      static const size_t intCost = 6;

      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef FastAllocator::ThreadLocal2* Allocator;
      typedef SharedVector<mvector<PrimRefMB>> SharedPrimRefVector;
      typedef LocalChildListT<BuildRecord2,MAX_BRANCHING_FACTOR> LocalChildList;
 
      typedef HeuristicMBlurTemporalSplit<PrimRefMB,RecalculatePrimRef,NUM_TEMPORAL_BINS> HeuristicTemporal;
      typedef HeuristicArrayBinningMB<PrimRefMB,NUM_OBJECT_BINS> HeuristicBinning;
      typedef UnalignedHeuristicArrayBinningMB<PrimRefMB,NUM_OBJECT_BINS> UnalignedHeuristicBinning;

    public:
      
      BVHNBuilderHairMBlur (Scene* scene,
                            const RecalculatePrimRef& recalculatePrimRef,
                            const CreateAllocFunc& createAlloc, 
                            const CreateAlignedNodeFunc& createAlignedNode, 
                            const CreateUnalignedNodeFunc& createUnalignedNode, 
                            const CreateAlignedNode4DFunc& createAlignedNode4D, 
                            const CreateLeafFunc& createLeaf,
                            const ProgressMonitor& progressMonitor,
                            const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, 
                            const size_t minLeafSize, const size_t maxLeafSize )
        : scene(scene),
        createAlloc(createAlloc), 
        createAlignedNode(createAlignedNode), 
        createUnalignedNode(createUnalignedNode), 
        createAlignedNode4D(createAlignedNode4D),
        createLeaf(createLeaf),
        progressMonitor(progressMonitor),
        branchingFactor(branchingFactor), maxDepth(maxDepth), logBlockSize(logBlockSize), 
        minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
        unalignedHeuristic(scene),
        temporalSplitHeuristic(scene->device,recalculatePrimRef) {}
       
      /*! entry point into builder */
      NodeRef operator() (BuildRecord2& current) {
        NodeRef root = recurse(current,nullptr,true);
        _mm_mfence(); // to allow non-temporal stores during build
        return root;
      }
      
    private:

      void deterministic_order(const SetMB& set) 
      {
        /* required as parallel partition destroys original primitive order */
        PrimRefMB* prims = set.prims->data();
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
      NodeRef createLargeLeaf(BuildRecord2& current, Allocator alloc)
      {
        /* this should never occur but is a fatal error */
        if (current.depth > maxDepth) 
          throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize)
          return createLeaf(current,alloc);
        
        /* fill all children by always splitting the largest one */
        LocalChildList children(current);
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          size_t bestSize = 0;
          for (unsigned i=0; i<children.size(); i++)
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
          BuildRecord2 left(current.depth+1);
          BuildRecord2 right(current.depth+1);
          splitFallback(children[bestChild].prims,left.pinfo,left.prims,right.pinfo,right.prims);
          children.split(bestChild,left,right);
          
        } while (children.size() < branchingFactor);
        
        /* create node */
        auto node = createAlignedNode(&children[0],children.size(),alignedHeuristic,alloc);
        return BVH::encodeNode(node);
      }
            
      /*! performs split */
      void split(const BuildRecord2& current, BuildRecord2& lrecord, BuildRecord2& rrecord, bool& aligned, bool& timesplit)
      {
        /* variable to track the SAH of the best splitting approach */
        float bestSAH = inf;
        const float leafSAH = intCost*float(current.pinfo.num_time_segments)*current.pinfo.halfArea();
        
        /* perform standard binning in aligned space */
        HeuristicBinning::Split alignedObjectSplit;
        alignedObjectSplit = alignedHeuristic.find(current.prims,current.pinfo,0);
        float alignedObjectSAH = travCostAligned*current.pinfo.halfArea() + intCost*alignedObjectSplit.splitSAH();
        bestSAH = min(alignedObjectSAH,bestSAH);

        /* perform standard binning in unaligned space */
        UnalignedHeuristicBinning::Split unalignedObjectSplit;
        LinearSpace3fa uspace;
        float unalignedObjectSAH = inf;
        if (alignedObjectSAH > 0.7f*leafSAH) {
          uspace = unalignedHeuristic.computeAlignedSpaceMB(scene,current.prims); 
          const PrimInfoMB sinfo = unalignedHeuristic.computePrimInfoMB(scene,current.prims,uspace);
          unalignedObjectSplit = unalignedHeuristic.find(current.prims,sinfo,0,uspace);    	
          unalignedObjectSAH = travCostUnaligned*current.pinfo.halfArea() + intCost*unalignedObjectSplit.splitSAH();
          bestSAH = min(unalignedObjectSAH,bestSAH);
        }

        /* do temporal splits only if the the time range is big enough */
        float temporal_split_sah = inf;
        typename HeuristicTemporal::Split temporal_split;
        if (current.prims.time_range.size() > 1.01f/float(current.pinfo.max_num_time_segments)) {
          temporal_split = temporalSplitHeuristic.find(current.prims, current.pinfo, 0);
          temporal_split_sah = travCostAligned*current.pinfo.halfArea() + temporal_split.splitSAH();
          bestSAH = min(temporal_split_sah,bestSAH);
        }

        /* perform fallback split */
        if (!std::isfinite(bestSAH))
        {
          deterministic_order(current.prims);
          splitFallback(current.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
        }
        else if (bestSAH == temporal_split_sah) {
          temporalSplitHeuristic.split(temporal_split,current.pinfo,current.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
          timesplit = true;
        }
        /* perform aligned split if this is best */
        else if (bestSAH == alignedObjectSAH) {
          alignedHeuristic.split(alignedObjectSplit,current.pinfo,current.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
        }
        /* perform unaligned split if this is best */
        else if (bestSAH == unalignedObjectSAH) {
          unalignedHeuristic.split(unalignedObjectSplit,uspace,current.pinfo,current.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
          aligned = false;
        }
        /* otherwise perform fallback split */
        else {
          deterministic_order(current.prims);
          splitFallback(current.prims,lrecord.pinfo,lrecord.prims,rrecord.pinfo,rrecord.prims);
        }
      }
      
      /*! recursive build */
      NodeRef recurse(BuildRecord2& current, Allocator alloc, bool toplevel)
      {
        if (alloc == nullptr) 
          alloc = createAlloc();

        /* call memory monitor function to signal progress */
        if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
          progressMonitor(current.size());
       
        /* create leaf node */
        if (current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize) {
          deterministic_order(current.prims);
          return createLargeLeaf(current,alloc);
        }
        
        /* fill all children by always splitting the one with the largest surface area */
        LocalChildList children(current);
        bool aligned = true;
        bool timesplit = false;
        
        do {
          
          /* find best child with largest bounding box area */
          ssize_t bestChild = -1;
          float bestArea = neg_inf;
          for (size_t i=0; i<children.size(); i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].size() <= minLeafSize)
              continue;
            
            /* remember child with largest area */
            const float A = children[i].pinfo.halfArea();
            if (A > bestArea) { 
              bestArea = children[i].pinfo.halfArea();
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          BuildRecord2 left(current.depth+1);
          BuildRecord2 right(current.depth+1);
          split(children[bestChild],left,right,aligned,timesplit);
          children.split(bestChild,left,right);
          
        } while (children.size() < branchingFactor); 
        assert(children.size() > 1);
	
        /* create time split node */
        if (timesplit)
        {
          auto node = createAlignedNode4D(&children[0],children.size(),alloc);

          /* spawn tasks or ... */
          if (current.size() > SINGLE_THREADED_THRESHOLD)
          {
            parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  node->child(i) = recurse(children[i],nullptr,true); 
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
          }
          /* ... continue sequential */
          else {
            for (size_t i=0; i<children.size(); i++) 
              node->child(i) = recurse(children[i],alloc,false);
          }
          return BVH::encodeNode(node);
        }

        /* create aligned node */
        else if (aligned) 
        {
          auto node = createAlignedNode(&children[0],children.size(),alignedHeuristic,alloc);

          /* spawn tasks or ... */
          if (current.size() > SINGLE_THREADED_THRESHOLD)
          {
            parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                  node->child(i) = recurse(children[i],nullptr,true); 
                  _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
          }
          /* ... continue sequential */
          else {
            for (size_t i=0; i<children.size(); i++) 
              node->child(i) = recurse(children[i],alloc,false);
          }
          return BVH::encodeNode(node);
        }
        
        /* create unaligned node */
        else 
        {
          auto node = createUnalignedNode(&children[0],children.size(),unalignedHeuristic,alloc);
          
          /* spawn tasks or ... */
          if (current.size() > SINGLE_THREADED_THRESHOLD)
          {

            parallel_for(size_t(0), children.size(), [&] (const range<size_t>& r) {
                for (size_t i=r.begin(); i<r.end(); i++) {
                    node->child(i) = recurse(children[i],nullptr,true);
                    _mm_mfence(); // to allow non-temporal stores during build
                }                
              });
          }
          /* ... continue sequentially */
          else
          {
            for (size_t i=0; i<children.size(); i++) 
              node->child(i) = recurse(children[i],alloc,false);
          }
          return BVH::encodeNode(node);
        }
      }
    
    private:      
      Scene* scene;
      const CreateAllocFunc& createAlloc;
      const CreateAlignedNodeFunc& createAlignedNode;
      const CreateUnalignedNodeFunc& createUnalignedNode;
      const CreateAlignedNode4DFunc& createAlignedNode4D;
      const CreateLeafFunc& createLeaf;
      const ProgressMonitor& progressMonitor;

    private:
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t logBlockSize;
      const size_t minLeafSize;
      const size_t maxLeafSize;
  
    private:
      HeuristicBinning alignedHeuristic;
      UnalignedHeuristicBinning unalignedHeuristic;
      HeuristicTemporal temporalSplitHeuristic;
    };

    template<int N,
             typename RecalculatePrimRef,
             typename CreateAllocFunc,
             typename CreateAlignedNodeFunc, 
             typename CreateUnalignedNodeFunc, 
             typename CreateAlignedNode4DFunc, 
             typename CreateLeafFunc, 
             typename ProgressMonitor>

      typename BVHN<N>::NodeRef bvh_obb_builder_binned_sah_mblur (Scene* scene,
                                                                  const RecalculatePrimRef& recalculatePrimRef,
                                                                  const CreateAllocFunc& createAlloc,
                                                                  const CreateAlignedNodeFunc& createAlignedNode, 
                                                                  const CreateUnalignedNodeFunc& createUnalignedNode, 
                                                                  const CreateAlignedNode4DFunc& createAlignedNode4D, 
                                                                  const CreateLeafFunc& createLeaf, 
                                                                  const ProgressMonitor& progressMonitor,
                                                                  BuildRecord2& current,
                                                                  const size_t branchingFactor, const size_t maxDepth, const size_t logBlockSize, 
                                                                  const size_t minLeafSize, const size_t maxLeafSize) 
    {
      typedef BVHNBuilderHairMBlur<N,RecalculatePrimRef,CreateAllocFunc,CreateAlignedNodeFunc,CreateUnalignedNodeFunc,CreateAlignedNode4DFunc,CreateLeafFunc,ProgressMonitor> Builder;
      Builder builder(scene,recalculatePrimRef,createAlloc,createAlignedNode,createUnalignedNode,createAlignedNode4D,createLeaf,
                      progressMonitor,branchingFactor,maxDepth,logBlockSize,minLeafSize,maxLeafSize);
      return builder(current);
    }
  }
}
