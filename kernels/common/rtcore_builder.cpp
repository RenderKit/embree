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

#ifdef _WIN32
#  define RTCORE_API extern "C" __declspec(dllexport)
#else
#  define RTCORE_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "default.h"
#include "device.h"
#include "scene.h"
#include "context.h"
#include "alloc.h"

#include "../builders/bvh_builder_sah.h"
#include "../builders/bvh_builder_morton.h"

namespace embree
{ 
  namespace isa // FIXME: support more ISAs for builders
  {
    RTCORE_API RTCAllocator rtcNewAllocator(RTCDevice device, size_t estimatedBytes)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcNewAllocator);
      RTCORE_VERIFY_HANDLE(device);
      FastAllocator* allocator = new FastAllocator((Device*)device,true);
      if (estimatedBytes) allocator->init_estimate(estimatedBytes);
      return (RTCAllocator) allocator;
      RTCORE_CATCH_END((Device*)device);
      return nullptr;
    }

    RTCORE_API RTCThreadLocalAllocator rtcGetThreadLocalAllocator(RTCAllocator allocator)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcNewAllocator);
      return (RTCThreadLocalAllocator) ((FastAllocator*) allocator)->threadLocal();
      RTCORE_CATCH_END(((FastAllocator*) allocator)->getDevice());
      return nullptr;
    }

    RTCORE_API void* rtcThreadLocalMalloc(RTCThreadLocalAllocator localAllocator, size_t bytes, size_t align)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcThreadLocalMalloc);
      FastAllocator::ThreadLocal* alloc = (FastAllocator::ThreadLocal*) localAllocator;
      return alloc->malloc(bytes,align);
      RTCORE_CATCH_END(((FastAllocator::ThreadLocal*) localAllocator)->alloc->getDevice());
      return nullptr;
    }

    RTCORE_API void rtcDeleteThreadLocalAllocators(RTCAllocator allocator)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcDeleteThreadLocalAllocators);
      ((FastAllocator*) allocator)->cleanup();
      RTCORE_CATCH_END(((FastAllocator*) allocator)->getDevice());
    }

    RTCORE_API void rtcClearAllocator(RTCAllocator allocator)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcClearAllocator);
      ((FastAllocator*) allocator)->clear();
      RTCORE_CATCH_END(((FastAllocator*) allocator)->getDevice());
    }

    RTCORE_API void rtcResetAllocator(RTCAllocator allocator)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcResetAllocator);
      ((FastAllocator*) allocator)->reset();
      RTCORE_CATCH_END(((FastAllocator*) allocator)->getDevice());
    }

    RTCORE_API void rtcDeleteAllocator(RTCAllocator allocator)
    {
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcDeleteAllocator);
      delete (FastAllocator*) allocator;
      RTCORE_CATCH_END(((FastAllocator*) allocator)->getDevice());
    }

    RTCORE_API void* rtcBVHBuildSAH(RTCDevice hdevice,
                                      const RTCBuildSettings& settings,
                                      RTCBuildPrimitive* prims,
                                      size_t numPrims,
                                      void* userPtr,
                                      RTCCreateThreadLocalFunc createThreadLocal,
                                      RTCCreateNodeFunc createNode,
                                      RTCSetNodeChildFunc setNodeChild,
                                      RTCSetNodeBoundsFunc setNodeBounds,
                                      RTCCreateLeafFunc createLeaf,
                                      RTCBuildProgressFunc buildProgress)
    {
      Device* device = (Device*) hdevice;
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcBVHBuildSAH);
      RTCORE_VERIFY_HANDLE(device);

      /* calculate priminfo */
      auto computeBounds = [&](const range<size_t>& r) -> CentGeomBBox3fa
        {
          CentGeomBBox3fa bounds(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
            bounds.extend((BBox3fa&)prims[j]);
          return bounds;
        };
      const CentGeomBBox3fa bounds = 
        parallel_reduce(size_t(0),numPrims,size_t(1024),size_t(1024),CentGeomBBox3fa(empty), computeBounds, CentGeomBBox3fa::merge2);

      const PrimInfo pinfo(0,numPrims,bounds.geomBounds,bounds.centBounds);
      
      /* build BVH */
      return BVHBuilderBinnedSAH::build<void*>(
        
        /* thread local allocator for fast allocations */
        [&] () -> void* { 
          return createThreadLocal(userPtr);
        },

        /* lambda function that creates BVH nodes */
        [&](BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, void* threadLocal) -> void*
        {
          void* node = createNode(threadLocal,N);
          for (size_t i=0; i<N; i++)
            setNodeBounds(node,i,(RTCBounds&)children[i].prims.geomBounds);
          return node;
        },

        /* lambda function that updates BVH nodes */
        [&](void* node, void** children, const size_t N) -> void*
        {
          for (size_t i=0; i<N; i++)
            setNodeChild(node,i,children[i]);
          return node;
        },
        
        /* lambda function that creates BVH leaves */
        [&](const BVHBuilderBinnedSAH::BuildRecord& current, void* threadLocal) -> void* {
          return createLeaf(threadLocal,prims+current.prims.begin(),current.prims.size());
        },
        
        /* progress monitor function */
        [&] (size_t dn) { 
          buildProgress(userPtr,dn);
        },
        
        (PrimRef*)prims,pinfo,settings);
         

      RTCORE_CATCH_END(device);
      return nullptr;
    }

    RTCORE_API void* rtcBVHBuildMorton(RTCDevice hdevice,
                                         const RTCBuildSettings& settings,
                                         RTCBuildPrimitive* prims_i,
                                         size_t numPrims,
                                         void* userPtr,
                                         RTCCreateThreadLocalFunc createThreadLocal,
                                         RTCCreateNodeFunc createNode,
                                         RTCSetNodeChildFunc setNodeChild,
                                         RTCSetNodeBoundsFunc setNodeBounds,
                                         RTCCreateLeafFunc createLeaf,
                                         RTCBuildProgressFunc buildProgress)
    {
      Device* device = (Device*) hdevice;
      RTCORE_CATCH_BEGIN;
      RTCORE_TRACE(rtcBVHBuildMorton);
      RTCORE_VERIFY_HANDLE(hdevice);

      /* initialize temporary arrays for morton builder */
      PrimRef* prims = (PrimRef*) prims_i;
      mvector<BVHBuilderMorton::MortonID32Bit> morton_src(device,numPrims);
      mvector<BVHBuilderMorton::MortonID32Bit> morton_tmp(device,numPrims);
      parallel_for(size_t(0), numPrims, size_t(1024), [&] ( range<size_t> r ) {
          for (size_t i=r.begin(); i<r.end(); i++)
            morton_src[i].index = i;
        });
      
      /* start morton build */
      std::pair<void*,BBox3fa> root = BVHBuilderMorton::build<std::pair<void*,BBox3fa>>(
        
        /* thread local allocator for fast allocations */
        [&] () -> void* { 
          return createThreadLocal(userPtr);
        },
        
        /* lambda function that allocates BVH nodes */
        [&] ( void* threadLocal, size_t N ) -> void* {
          return createNode(threadLocal,N);
        },
        
        /* lambda function that sets bounds */
        [&] (void* node, const std::pair<void*,BBox3fa>* children, size_t N) -> std::pair<void*,BBox3fa>
        {
          BBox3fa res = empty;
          for (size_t i=0; i<N; i++) {
            res.extend(children[i].second);
            setNodeChild(node,i,children[i].first);
            setNodeBounds(node,i,(RTCBounds&)children[i].second);
          }
          return std::make_pair(node,res);
        },
        
        /* lambda function that creates BVH leaves */
        [&]( const range<unsigned>& current, void* threadLocal) -> std::pair<void*,BBox3fa>
        {
          const size_t id = morton_src[current.begin()].index;
          const BBox3fa bounds = prims[id].bounds(); 
          void* node = createLeaf(threadLocal,prims_i+current.begin(),current.size());
          return std::make_pair(node,bounds);
        },
        
        /* lambda that calculates the bounds for some primitive */
        [&] (const BVHBuilderMorton::MortonID32Bit& morton) -> BBox3fa {
          return prims[morton.index].bounds();
        },
        
        /* progress monitor function */
        [&] (size_t dn) { 
          buildProgress(userPtr,dn);
        },
        
        morton_src.data(),morton_tmp.data(),numPrims,
        settings);
      
      return root.first;

      RTCORE_CATCH_END(device);
      return nullptr;
    }
  }
}

