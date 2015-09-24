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

#include "bvh8_builder.h"

namespace embree
{
  namespace isa
  {
    struct CreateBVH8Alloc
    {
      __forceinline CreateBVH8Alloc (BVH8* bvh) : bvh(bvh) {}
      __forceinline FastAllocator::ThreadLocal2* operator() () const { return bvh->alloc2.threadLocal2();  }

      BVH8* bvh;
    };

    struct CreateBVH8Node
    {
      __forceinline CreateBVH8Node (BVH8* bvh) : bvh(bvh) {}
      
      template<typename BuildRecord>
      __forceinline BVH8::Node* operator() (const BuildRecord& current, BuildRecord* children, const size_t N, FastAllocator::ThreadLocal2* alloc) 
      {
        BVH8::Node* node = nullptr;
        //if (current.pinfo.size() > 4096) node = (BVH8::Node*)   bvh->alloc2.malloc(sizeof(BVH8::Node),sizeof(BVH8::Node));
        //else
        node = (BVH8::Node*) alloc->alloc0.malloc(sizeof(BVH8::Node), 1 << BVH8::alignment); 
        node->clear();
        for (size_t i=0; i<N; i++) {
          node->set(i,children[i].pinfo.geomBounds);
          children[i].parent = (size_t*) &node->child(i);
        }
        *current.parent = bvh->encodeNode(node);
	return node;
      }

      BVH8* bvh;
    };

    /* tree rotations */
    __forceinline size_t rotate(BVH8::Node* node, const size_t* counts, const size_t N)
    {
      size_t n = 0;
      return n;
    }

    void BVH8Builder::BVH8BuilderV::build(BVH8* bvh, BuildProgressMonitor& progress_in, PrimRef* prims, const PrimInfo& pinfo, 
                                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost)
    {
      //bvh->alloc.init_estimate(pinfo.size()*sizeof(PrimRef));

      auto progressFunc = [&] (size_t dn) { 
        progress_in(dn); 
      };
            
      auto createLeafFunc = [&] (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) -> size_t {
        return createLeaf(current,alloc);
      };
      
      BVH8::NodeRef root;
      BVHBuilderBinnedSAH::build_reduce<BVH8::NodeRef>
        (root,CreateBVH8Alloc(bvh),size_t(0),CreateBVH8Node(bvh),rotate,createLeafFunc,progressFunc,
         prims,pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);

      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
      bvh->layoutLargeNodes(pinfo.size()*0.005f);
    }

    void BVH8BuilderSpatial::BVH8BuilderV::build(BVH8* bvh, BuildProgressMonitor& progress_in, PrimRefList& prims, const PrimInfo& pinfo, 
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
      
      BVH8::NodeRef root;
      BVHBuilderBinnedSpatialSAH::build_reduce<BVH8::NodeRef>
        (root,CreateBVH8Alloc(bvh),size_t(0),CreateBVH8Node(bvh),rotate,
         createLeafFunc,splitPrimitiveFunc,progressFunc,
         prims,pinfo,BVH8::N,BVH8::maxBuildDepthLeaf,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
      
      bvh->set(root,pinfo.geomBounds,pinfo.size());
      
      bvh->layoutLargeNodes(pinfo.size()*0.005f);
    }
  }
}
