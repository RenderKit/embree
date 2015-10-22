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

#include "bvh_builder.h"
#include "../bvh4/bvh4_rotate.h"

#define ROTATE_TREE 0

namespace embree
{
  namespace isa
  {
    /* tree rotations */
    template<int N>
    __forceinline size_t rotate(typename BVHN<N>::Node* node, const size_t* counts, const size_t N) {
      return 0;
    }

#if ROTATE_TREE
    template<>
    __forceinline size_t rotate<4>(BVH4::Node* node, const size_t* counts, const size_t N)
    {
      size_t n = 0;
      assert(N <= BVH4::N);
      for (size_t i=0; i<N; i++) 
        n += counts[i];
      if (n >= 4096) {
        for (size_t i=0; i<N; i++) {
          if (counts[i] < 4096) {
            for (int j=0; j<ROTATE_TREE; j++) 
              BVH4Rotate::rotate(node->child(i)); 
            node->child(i).setBarrier();
          }
        }
      }
      return n;
    }
#endif

    template<int N>
    void BVHBuilder<N>::BVHBuilderV::build(BVH* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      //bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
      
      typename BVH::NodeRef root;
      BVHBuilderBinnedSAH::build_reduce<BVH::NodeRef>
        (root,BVH::CreateAlloc(bvh),size_t(0),BVH::CreateNode(bvh),rotate<N>,createLeafFunc,progressFunc,
         prims,pinfo,BVH::N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);

      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
#if ROTATE_TREE
      for (int i=0; i<ROTATE_TREE; i++) 
        BVHRotate::rotate(bvh->root);
      bvh->clearBarrier(bvh->root);
#endif
      
      bvh->layoutLargeNodes(pinfo.size()*0.005f);
    }

    template<int N>
      struct CreateNodeMB
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeMB NodeMB;

      __forceinline CreateNodeMB (BVH* bvh) : bvh(bvh) {}
      
      __forceinline NodeMB* operator() (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, FastAllocator::ThreadLocal2* alloc) 
      {
        NodeMB* node = (NodeMB*) alloc->alloc0.malloc(sizeof(NodeMB)); node->clear();
        for (size_t i=0; i<N; i++) {
          children[i].parent = (size_t*)&node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH* bvh;
    };

    template<int N>
    void BVHBuilderMblur<N>::BVHBuilderV::build(BVH* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      //bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> std::pair<BBox3fa,BBox3fa> {
        return createLeaf(current,alloc);
      };

      /* reduction function */
      auto reduce = [] (typename BVH::NodeMB* node, const std::pair<BBox3fa,BBox3fa>* bounds, const size_t N) -> std::pair<BBox3fa,BBox3fa>
      {
        assert(N <= BVH::N);
        BBox3fa bounds0 = empty;
        BBox3fa bounds1 = empty;
        for (size_t i=0; i<N; i++) {
          const BBox3fa b0 = bounds[i].first;
          const BBox3fa b1 = bounds[i].second;
          node->set(i,b0,b1);
          bounds0 = merge(bounds0,b0);
          bounds1 = merge(bounds1,b1);
        }
        return std::pair<BBox3fa,BBox3fa>(bounds0,bounds1);
      };
      auto identity = std::make_pair(BBox3fa(empty),BBox3fa(empty));
      
      typename BVH::NodeRef root;
      BVHBuilderBinnedSAH::build_reduce<typename BVH::NodeRef>
        (root,BVH::CreateAlloc(bvh),identity,CreateNodeMB<N>(bvh),reduce,createLeafFunc,progressFunc,
         prims,pinfo,BVH::N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);

      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
#if ROTATE_TREE
      for (int i=0; i<ROTATE_TREE; i++) 
        BVHRotate::rotate(bvh->root);
      bvh->clearBarrier(bvh->root);
#endif
      
      //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: enable
    }

    template<int N>
    void BVHBuilderSpatial<N>::BVHBuilderV::build(BVH* bvh, BuildProgressMonitor& progress_in, PrimRefList& prims, const PrimInfo& pinfo, 
                                                  const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      //bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
      
      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };

      auto splitPrimitiveFunc = [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) -> void {
        splitPrimitive(prim,dim,pos,left_o,right_o);
      };

      auto createLeafFunc = [&] (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
      
      typename BVH::NodeRef root;
      BVHBuilderBinnedSpatialSAH::build_reduce<typename BVH::NodeRef>
        (root,BVH::CreateAlloc(bvh),size_t(0),BVH::CreateNode(bvh),rotate<N>,
         createLeafFunc,splitPrimitiveFunc,progressFunc,
         prims,pinfo,BVH::N,BVH::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
      
      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
#if ROTATE_TREE
      for (int i=0; i<ROTATE_TREE; i++) 
        BVHRotate::rotate(bvh->root);
      bvh->clearBarrier(bvh->root);
#endif
      
      bvh->layoutLargeNodes(pinfo.size()*0.005f);
    }

    template class BVHBuilder<4>;
    template class BVHBuilderMblur<4>;    
    template class BVHBuilderSpatial<4>;

#if defined(__AVX__)
    template class BVHBuilder<8>;
    template class BVHBuilderMblur<8>;
    template class BVHBuilderSpatial<8>;
#endif
  }
}
