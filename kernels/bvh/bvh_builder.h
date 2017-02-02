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

#include "bvh.h"
#include "../builders/bvh_builder_sah.h"

namespace embree
{
  namespace isa
  {
    template<int N>
      struct BVHNBuilder
      {
        typedef BVHN<N> BVH;
        typedef typename BVH::NodeRef NodeRef;
        typedef FastAllocator::ThreadLocal2 Allocator;
      
        struct BVHNBuilderV {
          void build(BVH* bvh, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold);
          virtual size_t createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) = 0;
        };

        template<typename CreateLeafFunc>
        struct BVHNBuilderT : public BVHNBuilderV
        {
          BVHNBuilderT (CreateLeafFunc createLeafFunc)
            : createLeafFunc(createLeafFunc) {}

          size_t createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) {
            return createLeafFunc(current,alloc);
          }

        private:
          CreateLeafFunc createLeafFunc;
        };

        template<typename CreateLeafFunc>
        static void build(BVH* bvh, CreateLeafFunc createLeaf, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold) {
          BVHNBuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);
        }
      };


    template<int N>
      struct BVHNBuilderQuantized
      {
        typedef BVHN<N> BVH;
        typedef typename BVH::NodeRef NodeRef;
        typedef FastAllocator::ThreadLocal2 Allocator;
      
        struct BVHNBuilderV {
          void build(BVH* bvh, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold);
          virtual size_t createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) = 0;
        };

        template<typename CreateLeafFunc>
        struct BVHNBuilderT : public BVHNBuilderV
        {
          BVHNBuilderT (CreateLeafFunc createLeafFunc)
            : createLeafFunc(createLeafFunc) {}

          size_t createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) {
            return createLeafFunc(current,alloc);
          }

        private:
          CreateLeafFunc createLeafFunc;
        };

        template<typename CreateLeafFunc>
        static void build(BVH* bvh, CreateLeafFunc createLeaf, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold) {
          BVHNBuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);
        }
      };

    template<int N>
      struct BVHNBuilderMblur
      {
        typedef BVHN<N> BVH;
        typedef typename BVH::AlignedNodeMB AlignedNodeMB;
        typedef typename BVH::NodeRef NodeRef;
        typedef FastAllocator::ThreadLocal2 Allocator;
      
        struct BVHNBuilderV {
          std::tuple<NodeRef,LBBox3fa> build(BVH* bvh, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold);
          virtual LBBox3fa createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) = 0;
        };

        template<typename CreateLeafFunc>
        struct BVHNBuilderT : public BVHNBuilderV
        {
          BVHNBuilderT (CreateLeafFunc createLeafFunc)
            : createLeafFunc(createLeafFunc) {}

          LBBox3fa createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) {
            return createLeafFunc(current,alloc);
          }

        private:
          CreateLeafFunc createLeafFunc;
        };

        template<typename CreateLeafFunc>
        static std::tuple<NodeRef,LBBox3fa>  build(BVH* bvh, CreateLeafFunc createLeaf, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost, const size_t singleThreadThreshold) {
          return BVHNBuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost,singleThreadThreshold);
        }
      };

  }
}
