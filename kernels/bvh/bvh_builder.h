// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost);
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
                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost) {
          BVHNBuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
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
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost);
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
                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost) {
          BVHNBuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
        }
      };

    template<int N>
      struct BVHNBuilderMblur
      {
        typedef BVHN<N> BVH;
        typedef typename BVH::NodeMB NodeMB;
        typedef typename BVH::NodeRef NodeRef;
        typedef FastAllocator::ThreadLocal2 Allocator;
      
        struct BVHNBuilderV {
          void build(BVH* bvh, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost);
          virtual std::pair<BBox3fa,BBox3fa> createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) = 0;
        };

        template<typename CreateLeafFunc>
        struct BVHNBuilderT : public BVHNBuilderV
        {
          BVHNBuilderT (CreateLeafFunc createLeafFunc)
            : createLeafFunc(createLeafFunc) {}

          std::pair<BBox3fa,BBox3fa> createLeaf (const BVHBuilderBinnedSAH::BuildRecord& current, Allocator* alloc) {
            return createLeafFunc(current,alloc);
          }

        private:
          CreateLeafFunc createLeafFunc;
        };

        template<typename CreateLeafFunc>
        static void build(BVH* bvh, CreateLeafFunc createLeaf, BuildProgressMonitor& progress, PrimRef* prims, const PrimInfo& pinfo, 
                          const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost) {
          BVHNBuilderT<CreateLeafFunc>(createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
        }
      };

    template<int N>
      struct BVHNBuilderSpatial
      {
        typedef BVHN<N> BVH;
        typedef typename BVH::NodeRef NodeRef;
        typedef FastAllocator::ThreadLocal2 Allocator;
      
        struct BVHNBuilderV {
          void build(BVH* bvh, BuildProgressMonitor& progress, PrimRefList& prims, const PrimInfo& pinfo, 
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost);
          virtual void splitPrimitive (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) = 0;
          virtual size_t createLeaf (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc) = 0;
        };

        template<typename SplitPrimitiveFunc, typename CreateLeafFunc>
          struct BVHNBuilderT : public BVHNBuilderV
        {
          BVHNBuilderT (SplitPrimitiveFunc splitPrimitiveFunc, CreateLeafFunc createLeafFunc)
            : splitPrimitiveFunc(splitPrimitiveFunc), createLeafFunc(createLeafFunc) {}

          void splitPrimitive (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) {
            splitPrimitiveFunc(prim,dim,pos,left_o,right_o);
          }

          size_t createLeaf (BVHBuilderBinnedSpatialSAH::BuildRecord& current, Allocator* alloc) {
            return createLeafFunc(current,alloc);
          }

        private:
          SplitPrimitiveFunc splitPrimitiveFunc;
          CreateLeafFunc createLeafFunc;
        };

        template<typename SplitPrimitiveFunc, typename CreateLeafFunc>
          static void build(BVH* bvh, SplitPrimitiveFunc splitPrimitive, CreateLeafFunc createLeaf, BuildProgressMonitor& progress, PrimRefList& prims, const PrimInfo& pinfo, 
                            const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost) {
          BVHNBuilderT<SplitPrimitiveFunc,CreateLeafFunc>(splitPrimitive,createLeaf).build(bvh,progress,prims,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
        }
      };

    // =======================================================================================
    // =======================================================================================
    // =======================================================================================

    template<int N, int NUM_SPATIAL_SPLITS>
      struct BVHNBuilderFastSpatial
      {
        typedef BVHN<N> BVH;
        typedef typename BVH::NodeRef NodeRef;
        typedef FastAllocator::ThreadLocal2 Allocator;
      
        struct BVHNBuilderV {
          void build(BVH* bvh, BuildProgressMonitor& progress, PrimRef* prims0, const size_t extSize, const PrimInfo& pinfo, 
                     const size_t blockSize, const size_t minLeafSize, const size_t maxLeafSize, const float travCost, const float intCost);
          virtual void splitPrimitive (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) = 0;

          virtual void binnerSplit (SpatialBinInfo<NUM_SPATIAL_SPLITS,PrimRef> &spatialBinner, const PrimRef* const source, const size_t begin, const size_t end, const SpatialBinMapping<NUM_SPATIAL_SPLITS> &mapping) = 0;

          virtual size_t createLeaf (const BVHBuilderBinnedFastSpatialSAH::BuildRecord& current, Allocator* alloc) = 0;
        };

        template<typename SplitPrimitiveFunc, typename BinnerSplitPrimitiveFunc, typename CreateLeafFunc>
        struct BVHNBuilderT : public BVHNBuilderV
        {
          BVHNBuilderT (SplitPrimitiveFunc splitPrimitiveFunc, BinnerSplitPrimitiveFunc binnerSplitPrimitive, CreateLeafFunc createLeafFunc)
            : splitPrimitiveFunc(splitPrimitiveFunc), binnerSplitPrimitive(binnerSplitPrimitive), createLeafFunc(createLeafFunc) {}

          __forceinline void splitPrimitive (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) {
            splitPrimitiveFunc(prim,dim,pos,left_o,right_o);
          }

          __forceinline void binnerSplit (SpatialBinInfo<NUM_SPATIAL_SPLITS,PrimRef> &spatialBinner, const PrimRef* const source, const size_t begin, const size_t end, const SpatialBinMapping<NUM_SPATIAL_SPLITS> &mapping) {
            binnerSplitPrimitive(spatialBinner,source,begin,end,mapping);
          }
          
          size_t createLeaf(const BVHBuilderBinnedFastSpatialSAH::BuildRecord& current, Allocator* alloc) {
            return createLeafFunc(current,alloc);
          }

        private:
          SplitPrimitiveFunc splitPrimitiveFunc;
          BinnerSplitPrimitiveFunc binnerSplitPrimitive;
          CreateLeafFunc createLeafFunc;
        };

        template<typename SplitPrimitiveFunc, typename BinnerSplitPrimitiveFunc, typename CreateLeafFunc>
        static void build(BVH* bvh, SplitPrimitiveFunc splitPrimitive, BinnerSplitPrimitiveFunc binnerSplit, 
                          CreateLeafFunc createLeaf, 
                          BuildProgressMonitor& progress, PrimRef* prims0,const size_t extSize,
                          const PrimInfo& pinfo, const size_t blockSize, const size_t minLeafSize, 
                          const size_t maxLeafSize, const float travCost, const float intCost) {
          BVHNBuilderT<SplitPrimitiveFunc,BinnerSplitPrimitiveFunc, CreateLeafFunc>(splitPrimitive,binnerSplit,createLeaf).build(bvh,progress,prims0,extSize,pinfo,blockSize,minLeafSize,maxLeafSize,travCost,intCost);
        }
      };


  }
}
