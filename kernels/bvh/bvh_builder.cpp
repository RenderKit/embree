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

#include "bvh_builder.h"
#include "bvh_rotate.h"

#define ROTATE_TREE 0

namespace embree
{
  namespace isa
  {
    /* tree rotations */
    template<int N>
    __forceinline size_t rotate(typename BVHN<N>::AlignedNode* node, const size_t* counts, const size_t num) {
      return 0;
    }

    template<int N>
    __forceinline size_t dummy(typename BVHN<N>::AlignedNode* node, const size_t* counts, const size_t num) {
      return 0;
    }

#if ROTATE_TREE
    template<>
    __forceinline size_t rotate<4>(BVH4::AlignedNode* node, const size_t* counts, const size_t num)
    {
      size_t n = 0;
      assert(num <= 4);
      for (size_t i=0; i<num; i++)
        n += counts[i];
      if (n >= 4096) {
        for (size_t i=0; i<num; i++) {
          if (counts[i] < 4096) {
            for (int j=0; j<ROTATE_TREE; j++) 
              BVHNRotate<4>::rotate(node->child(i));
            node->child(i).setBarrier();
          }
        }
      }
      return n;
    }
#endif

    template<int N>
    void BVHNBuilder<N>::BVHNBuilderV::build(BVH* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold)
    {
      //bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
      
      NodeRef root;
      BVHBuilderBinnedSAH::build_reduce<NodeRef>
        (root,typename BVH::CreateAlloc(bvh),size_t(0),typename BVH::CreateAlignedNode(bvh),rotate<N>,createLeafFunc,progressFunc,
         prims,pinfo,N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);

      bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
      
#if ROTATE_TREE
      if (N == 4)
      {
        for (int i=0; i<ROTATE_TREE; i++)
          BVHNRotate<N>::rotate(bvh->root);
        bvh->clearBarrier(bvh->root);
      }
#endif
      
      bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));
    }


    template<int N>
    void BVHNBuilderQuantized<N>::BVHNBuilderV::build(BVH* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold)
    {
      //bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
            
      NodeRef root = 0;
      BVHBuilderBinnedSAH::build_reduce<NodeRef>
        (root,typename BVH::CreateAlloc(bvh),size_t(0),typename BVH::CreateQuantizedNode(bvh),dummy<N>,createLeafFunc,progressFunc,
         prims,pinfo,N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);

      NodeRef new_root = (size_t)root | BVH::tyQuantizedNode;
      // todo: COPY LAYOUT FOR LARGE NODES !!!
      //bvh->layoutLargeNodes(pinfo.size()*0.005f);
      assert(new_root.isQuantizedNode());
      bvh->set(new_root,LBBox3fa(pinfo.geomBounds),pinfo.size());
    }

    template<int N>
      struct CreateAlignedNodeMB
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::AlignedNodeMB AlignedNodeMB;

      __forceinline CreateAlignedNodeMB (BVH* bvh) : bvh(bvh) {}
      
      __forceinline AlignedNodeMB* operator() (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t num, FastAllocator::ThreadLocal2* alloc)
      {
        AlignedNodeMB* node = (AlignedNodeMB*) alloc->alloc0->malloc(sizeof(AlignedNodeMB),BVH::byteNodeAlignment); node->clear();
        for (size_t i=0; i<num; i++) {
          children[i].parent = (size_t*)&node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH* bvh;
    };

    template<int N>
    std::tuple<typename BVHN<N>::NodeRef,LBBox3fa> BVHNBuilderMblur<N>::BVHNBuilderV::build(BVH* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold)
    {
      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> LBBox3fa {
        return createLeaf(current,alloc);
      };

      /* reduction function */
      auto reduce = [] (AlignedNodeMB* node, const LBBox3fa* bounds, const size_t num) -> LBBox3fa
      {
        assert(num <= N);
        LBBox3fa allBounds = empty;
        for (size_t i=0; i<num; i++) {
          node->set(i, bounds[i]);
          allBounds.extend(bounds[i]);
        }
        return allBounds;
      };
      auto identity = LBBox3fa(empty);
      
      NodeRef root;
      LBBox3fa root_bounds = BVHBuilderBinnedSAH::build_reduce<NodeRef>
        (root,typename BVH::CreateAlloc(bvh),identity,CreateAlignedNodeMB<N>(bvh),reduce,createLeafFunc,progressFunc,
         prims,pinfo,N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);

      /* set bounding box to merge bounds of all time steps */
      bvh->set(root,root_bounds,pinfo.size()); // FIXME: remove later

#if ROTATE_TREE
      if (N == 4)
      {
        for (int i=0; i<ROTATE_TREE; i++)
          BVHNRotate<N>::rotate(bvh->root);
        bvh->clearBarrier(bvh->root);
      }
#endif
      
      //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: implement for Mblur nodes and activate
      
      return std::make_tuple(root,root_bounds);
    }

    template struct BVHNBuilder<4>;
    template struct BVHNBuilderQuantized<4>;
    template struct BVHNBuilderMblur<4>;    

#if defined(__AVX__)
    template struct BVHNBuilder<8>;
    template struct BVHNBuilderQuantized<8>;
    template struct BVHNBuilderMblur<8>;
#endif
  }
}
