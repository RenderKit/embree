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

#include "bvh4_builder.h"
#include "bvh4_rotate.h"

#define ROTATE_TREE 0

namespace embree
{
  namespace isa
  {
    /* tree rotations */
    __forceinline size_t rotate(BVH4::Node* node, const size_t* counts, const size_t N)
    {
      size_t n = 0;
#if ROTATE_TREE
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
#endif
      return n;
    }

    void BVH4Builder::BVH4BuilderV::build(BVH4* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
      
      BVH4::NodeRef root;
      BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
        (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),rotate,createLeafFunc,progressFunc,
         prims,pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);

      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
#if ROTATE_TREE
      for (int i=0; i<ROTATE_TREE; i++) 
        BVH4Rotate::rotate(bvh->root);
      bvh->clearBarrier(bvh->root);
#endif
      
      bvh->layoutLargeNodes(pinfo.size()*0.005f);
    }

    struct CreateBVH4NodeMB
    {
      __forceinline CreateBVH4NodeMB (BVH4* bvh) : bvh(bvh) {}
      
      __forceinline BVH4::NodeMB* operator() (const isa::BVHBuilderBinnedSAH::BuildRecord& current, BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, FastAllocator::ThreadLocal2* alloc) 
      {
        BVH4::NodeMB* node = (BVH4::NodeMB*) alloc->alloc0.malloc(sizeof(BVH4::NodeMB)); node->clear();
        for (size_t i=0; i<N; i++) {
          children[i].parent = (size_t*)&node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH4* bvh;
    };

    void BVH4BuilderMblur::BVH4BuilderV::build(BVH4* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> std::pair<BBox3fa,BBox3fa> {
        return createLeaf(current,alloc);
      };

      /* reduction function */
      auto reduce = [] (BVH4::NodeMB* node, const std::pair<BBox3fa,BBox3fa>* bounds, const size_t N) -> std::pair<BBox3fa,BBox3fa>
      {
        assert(N <= BVH4::N);
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
      
      BVH4::NodeRef root;
      BVHBuilderBinnedSAH::build_reduce<BVH4::NodeRef>
        (root,BVH4::CreateAlloc(bvh),identity,CreateBVH4NodeMB(bvh),reduce,createLeafFunc,progressFunc,
         prims,pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);

      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
#if ROTATE_TREE
      for (int i=0; i<ROTATE_TREE; i++) 
        BVH4Rotate::rotate(bvh->root);
      bvh->clearBarrier(bvh->root);
#endif
      
      //bvh->layoutLargeNodes(pinfo.size()*0.005f); // FIXME: enable
    }

    void BVH4BuilderSpatial::BVH4BuilderV::build(BVH4* bvh, BuildProgressMonitor& progress_in, PrimRefList& prims, const PrimInfo& pinfo, 
                                                 const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));
      
      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };

      auto splitPrimitiveFunc = [&] (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) -> void {
        splitPrimitive(prim,dim,pos,left_o,right_o);
      };

      auto createLeafFunc = [&] (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
      
      BVH4::NodeRef root;
      BVHBuilderBinnedSpatialSAH::build_reduce<BVH4::NodeRef>
        (root,BVH4::CreateAlloc(bvh),size_t(0),BVH4::CreateNode(bvh),rotate,
         createLeafFunc,splitPrimitiveFunc,progressFunc,
         prims,pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
      
      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
#if ROTATE_TREE
      for (int i=0; i<ROTATE_TREE; i++) 
        BVH4Rotate::rotate(bvh->root);
      bvh->clearBarrier(bvh->root);
#endif
      
      bvh->layoutLargeNodes(pinfo.size()*0.005f);
    }
  }
}
