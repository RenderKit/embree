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

#include "bvh4.h"
#include "../builders/bvh_builder_sah.h"

namespace embree
{
  namespace isa
  {
    struct BVH4Builder
    {
      typedef FastAllocator::ThreadLocal2 Allocator;
      
      struct BVH4BuilderV {
        void build(BVH4* bvh, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost);
        virtual size_t createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) = 0;
      };

      template<typename CreateLeafFunc>
      struct BVH4BuilderT : public BVH4BuilderV
      {
        BVH4BuilderT (CreateLeafFunc createLeafFunc) 
          : createLeafFunc(createLeafFunc) {}

        size_t createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) {
          return createLeafFunc(current,alloc);
        }

        CreateLeafFunc createLeafFunc;
      };

      template<typename CreateLeafFunc>
      static void build(BVH4* bvh, CreateLeafFunc createLeaf, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost) {
        BVH4BuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
      }
    };
  }
}
