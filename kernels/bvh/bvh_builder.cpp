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

namespace embree
{
  namespace isa
  {
    template<int N>
    void BVHNBuilder<N>::BVHNBuilderV::build(BVH* bvh, BuildProgressMonitor& progressFunc, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold)
    {
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> NodeRef {
        return createLeaf(current,alloc);
      };
      
      NodeRef root = BVHBuilderBinnedSAH::build_reduce<NodeRef>
        (typename BVH::CreateAlloc(bvh),NodeRef(0),typename BVH::CreateAlignedNode(),typename BVH::UpdateAlignedNode(),createLeafFunc,progressFunc,
         prims,pinfo,N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);

      bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
      
      bvh->layoutLargeNodes(size_t(pinfo.size()*0.005f));
    }


    template<int N>
    void BVHNBuilderQuantized<N>::BVHNBuilderV::build(BVH* bvh, BuildProgressMonitor& progressFunc, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold)
    {
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
            
      NodeRef root = BVHBuilderBinnedSAH::build_reduce<NodeRef>
        (typename BVH::CreateAlloc(bvh),NodeRef(0),typename BVH::CreateQuantizedNode(),typename BVH::UpdateQuantizedNode(),createLeafFunc,progressFunc,
         prims,pinfo,N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);

      //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: COPY LAYOUT FOR LARGE NODES !!!
      bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
    }

    template<int N>
    std::tuple<typename BVHN<N>::NodeRef,LBBox3fa> BVHNBuilderMblur<N>::BVHNBuilderV::build(BVH* bvh, BuildProgressMonitor& progressFunc, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold)
    {
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> std::pair<NodeRef,LBBox3fa> {
        return createLeaf(current,alloc);
      };

      auto identity = std::make_pair(NodeRef(0),LBBox3fa(empty));
      
      auto root = BVHBuilderBinnedSAH::build_reduce<NodeRef>
        (typename BVH::CreateAlloc(bvh),identity,typename BVH::CreateAlignedNodeMB(),typename BVH::UpdateAlignedNodeMB(),createLeafFunc,progressFunc,
         prims,pinfo,N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);

      /* set bounding box to merge bounds of all time steps */
      bvh->set(root.first,root.second,pinfo.size()); // FIXME: remove later

      //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: implement for Mblur nodes and activate
      
      return root;
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
