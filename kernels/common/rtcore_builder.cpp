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
#  define RTC_API extern "C" __declspec(dllexport)
#else
#  define RTC_API extern "C" __attribute__ ((visibility ("default")))
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
    struct BVH : public RefCount
    {
      BVH (Device* device)
        : device(device), allocator(device,true), morton_src(device,0), morton_tmp(device,0)
      {
        device->refInc();
      }

      ~BVH() {
        device->refDec();
      }

    public:
      Device* device;
      FastAllocator allocator;
      mvector<BVHBuilderMorton::BuildPrim> morton_src;
      mvector<BVHBuilderMorton::BuildPrim> morton_tmp;
    };

    RTC_API RTCBVH rtcNewBVH(RTCDevice device)
    {
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcNewAllocator);
      RTC_VERIFY_HANDLE(device);
      BVH* bvh = new BVH((Device*)device);
      return (RTCBVH) bvh->refInc();
      RTC_CATCH_END((Device*)device);
      return nullptr;
    }

    void* rtcBuildBVHMorton(const RTCBuildSettings* settings)
    {
      BVH* bvh = (BVH*) settings->bvh;
      RTCBuildPrimitive* prims_i =  settings->primitives;
      size_t numPrimitives = settings->numPrimitives;
      RTCCreateNodeFunction createNode = settings->createNode;
      RTCSetNodeChildrenFunction setNodeChildren = settings->setNodeChildren;
      RTCSetNodeBoundsFunction setNodeBounds = settings->setNodeBounds;
      RTCCreateLeafFunction createLeaf = settings->createLeaf;
      RTCProgressMonitorFunction buildProgress = settings->buildProgress;
      void* userPtr = settings->userPtr;
        
      std::atomic<size_t> progress(0);
      
      /* initialize temporary arrays for morton builder */
      PrimRef* prims = (PrimRef*) prims_i;
      mvector<BVHBuilderMorton::BuildPrim>& morton_src = bvh->morton_src;
      mvector<BVHBuilderMorton::BuildPrim>& morton_tmp = bvh->morton_tmp;
      morton_src.resize(numPrimitives);
      morton_tmp.resize(numPrimitives);

      /* compute centroid bounds */
      const BBox3fa centBounds = parallel_reduce ( size_t(0), numPrimitives, BBox3fa(empty), [&](const range<size_t>& r) -> BBox3fa {

          BBox3fa bounds(empty);
          for (size_t i=r.begin(); i<r.end(); i++) 
            bounds.extend(prims[i].bounds().center2());
          return bounds;
        }, BBox3fa::merge);
      
      /* compute morton codes */
      BVHBuilderMorton::MortonCodeMapping mapping(centBounds);
      parallel_for ( size_t(0), numPrimitives, [&](const range<size_t>& r) {
          BVHBuilderMorton::MortonCodeGenerator generator(mapping,&morton_src[r.begin()]);
          for (size_t i=r.begin(); i<r.end(); i++) {
            generator(prims[i].bounds(),(unsigned) i);
          }
        });

      /* start morton build */
      std::pair<void*,BBox3fa> root = BVHBuilderMorton::build<std::pair<void*,BBox3fa>>(
        
        /* thread local allocator for fast allocations */
        [&] () -> FastAllocator::CachedAllocator { 
          return bvh->allocator.getCachedAllocator();
        },
        
        /* lambda function that allocates BVH nodes */
        [&] ( const FastAllocator::CachedAllocator& alloc, size_t N ) -> void* {
          return createNode((RTCThreadLocalAllocator)&alloc, (unsigned int)N,userPtr);
        },
        
        /* lambda function that sets bounds */
        [&] (void* node, const std::pair<void*,BBox3fa>* children, size_t N) -> std::pair<void*,BBox3fa>
        {
          BBox3fa bounds = empty;
          void* childptrs[BVHBuilderMorton::MAX_BRANCHING_FACTOR];
          const RTCBounds* cbounds[BVHBuilderMorton::MAX_BRANCHING_FACTOR];
          for (size_t i=0; i<N; i++) {
            bounds.extend(children[i].second);
            childptrs[i] = children[i].first;
            cbounds[i] = (const RTCBounds*)&children[i].second;
          }
          setNodeBounds(node,cbounds,(unsigned int)N,userPtr);
          setNodeChildren(node,childptrs, (unsigned int)N,userPtr);
          return std::make_pair(node,bounds);
        },
        
        /* lambda function that creates BVH leaves */
        [&]( const range<unsigned>& current, const FastAllocator::CachedAllocator& alloc) -> std::pair<void*,BBox3fa>
        {
          const size_t id = morton_src[current.begin()].index;
          const BBox3fa bounds = prims[id].bounds(); 
          void* node = createLeaf((RTCThreadLocalAllocator)&alloc,prims_i+current.begin(),current.size(),userPtr);
          return std::make_pair(node,bounds);
        },
        
        /* lambda that calculates the bounds for some primitive */
        [&] (const BVHBuilderMorton::BuildPrim& morton) -> BBox3fa {
          return prims[morton.index].bounds();
        },
        
        /* progress monitor function */
        [&] (size_t dn) {
          if (!buildProgress) return true;
          const size_t n = progress.fetch_add(dn)+dn;
          const double f = std::min(1.0,double(n)/double(numPrimitives));
          return buildProgress(userPtr,f);
        },
        
        morton_src.data(),morton_tmp.data(),numPrimitives,
        *settings);

      bvh->allocator.cleanup();
      return root.first;
    }

    void* rtcBuildBVHBinnedSAH(const RTCBuildSettings* settings)
    {
      BVH* bvh = (BVH*) settings->bvh;
      RTCBuildPrimitive* prims =  settings->primitives;
      size_t numPrimitives = settings->numPrimitives;
      RTCCreateNodeFunction createNode = settings->createNode;
      RTCSetNodeChildrenFunction setNodeChildren = settings->setNodeChildren;
      RTCSetNodeBoundsFunction setNodeBounds = settings->setNodeBounds;
      RTCCreateLeafFunction createLeaf = settings->createLeaf;
      RTCProgressMonitorFunction buildProgress = settings->buildProgress;
      void* userPtr = settings->userPtr;
      
      std::atomic<size_t> progress(0);
  
      /* calculate priminfo */
      auto computeBounds = [&](const range<size_t>& r) -> CentGeomBBox3fa
        {
          CentGeomBBox3fa bounds(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
            bounds.extend((BBox3fa&)prims[j]);
          return bounds;
        };
      const CentGeomBBox3fa bounds = 
        parallel_reduce(size_t(0),numPrimitives,size_t(1024),size_t(1024),CentGeomBBox3fa(empty), computeBounds, CentGeomBBox3fa::merge2);

      const PrimInfo pinfo(0,numPrimitives,bounds);
      
      /* build BVH */
      void* root = BVHBuilderBinnedSAH::build<void*>(
        
        /* thread local allocator for fast allocations */
        [&] () -> FastAllocator::CachedAllocator { 
          return bvh->allocator.getCachedAllocator();
        },

        /* lambda function that creates BVH nodes */
        [&](BVHBuilderBinnedSAH::BuildRecord* children, const size_t N, const FastAllocator::CachedAllocator& alloc) -> void*
        {
          void* node = createNode((RTCThreadLocalAllocator)&alloc, (unsigned int)N,userPtr);
          const RTCBounds* cbounds[GeneralBVHBuilder::MAX_BRANCHING_FACTOR];
          for (size_t i=0; i<N; i++) cbounds[i] = (const RTCBounds*) &children[i].prims.geomBounds;
          setNodeBounds(node,cbounds, (unsigned int)N,userPtr);
          return node;
        },

        /* lambda function that updates BVH nodes */
        [&](const BVHBuilderBinnedSAH::BuildRecord& precord, const BVHBuilderBinnedSAH::BuildRecord* crecords, void* node, void** children, const size_t N) -> void* {
          setNodeChildren(node,children, (unsigned int)N,userPtr);
          return node;
        },
        
        /* lambda function that creates BVH leaves */
        [&](const PrimRef* prims, const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) -> void* {
          return createLeaf((RTCThreadLocalAllocator)&alloc,(RTCBuildPrimitive*)(prims+range.begin()),range.size(),userPtr);
        },
        
        /* progress monitor function */
        [&] (size_t dn) {
          if (!buildProgress) return true;
          const size_t n = progress.fetch_add(dn)+dn;
          const double f = std::min(1.0,double(n)/double(numPrimitives));
          return buildProgress(userPtr,f);
        },
        
        (PrimRef*)prims,pinfo,*settings);
        
      bvh->allocator.cleanup();
      return root;
    }

    void* rtcBuildBVHSpatialSAH(const RTCBuildSettings* settings)
    {
      BVH* bvh = (BVH*) settings->bvh;
      RTCBuildPrimitive* prims =  settings->primitives;
      size_t numPrimitives = settings->numPrimitives;
      RTCCreateNodeFunction createNode = settings->createNode;
      RTCSetNodeChildrenFunction setNodeChildren = settings->setNodeChildren;
      RTCSetNodeBoundsFunction setNodeBounds = settings->setNodeBounds;
      RTCCreateLeafFunction createLeaf = settings->createLeaf;
      RTCSplitPrimitiveFunction splitPrimitive = settings->splitPrimitive;
      RTCProgressMonitorFunction buildProgress = settings->buildProgress;
      void* userPtr = settings->userPtr;
      
      std::atomic<size_t> progress(0);
  
      /* calculate priminfo */
      auto computeBounds = [&](const range<size_t>& r) -> CentGeomBBox3fa
        {
          CentGeomBBox3fa bounds(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
            bounds.extend((BBox3fa&)prims[j]);
          return bounds;
        };
      const CentGeomBBox3fa bounds = 
        parallel_reduce(size_t(0),numPrimitives,size_t(1024),size_t(1024),CentGeomBBox3fa(empty), computeBounds, CentGeomBBox3fa::merge2);

      const PrimInfo pinfo(0,numPrimitives,bounds);

      /* function that splits a build primitive */
      struct Splitter
      {
        Splitter (RTCSplitPrimitiveFunction splitPrimitive, unsigned geomID, unsigned primID, void* userPtr)
          : splitPrimitive(splitPrimitive), geomID(geomID), primID(primID), userPtr(userPtr) {}
        
        __forceinline void operator() (PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) const 
        {
          prim.geomIDref() &= BVHBuilderBinnedFastSpatialSAH::GEOMID_MASK;
          splitPrimitive((RTCBuildPrimitive*)&prim,(unsigned)dim,pos,(RTCBounds*)&left_o,(RTCBounds*)&right_o,userPtr);
          left_o.geomIDref()  = geomID; left_o.primIDref()  = primID;
          right_o.geomIDref() = geomID; right_o.primIDref() = primID;
        }

        __forceinline void operator() (const BBox3fa& box, const size_t dim, const float pos, BBox3fa& left_o, BBox3fa& right_o) const 
        {
          PrimRef prim(box,geomID & BVHBuilderBinnedFastSpatialSAH::GEOMID_MASK,primID);
          splitPrimitive((RTCBuildPrimitive*)&prim,(unsigned)dim,pos,(RTCBounds*)&left_o,(RTCBounds*)&right_o,userPtr);
        }
   
        RTCSplitPrimitiveFunction splitPrimitive;
        unsigned geomID;
        unsigned primID;
        void* userPtr;
      };

      /* build BVH */
      void* root = BVHBuilderBinnedFastSpatialSAH::build<void*>(
        
        /* thread local allocator for fast allocations */
        [&] () -> FastAllocator::CachedAllocator { 
          return bvh->allocator.getCachedAllocator();
        },

        /* lambda function that creates BVH nodes */
        [&] (BVHBuilderBinnedFastSpatialSAH::BuildRecord* children, const size_t N, const FastAllocator::CachedAllocator& alloc) -> void*
        {
          void* node = createNode((RTCThreadLocalAllocator)&alloc, (unsigned int)N,userPtr);
          const RTCBounds* cbounds[GeneralBVHBuilder::MAX_BRANCHING_FACTOR];
          for (size_t i=0; i<N; i++) cbounds[i] = (const RTCBounds*) &children[i].prims.geomBounds;
          setNodeBounds(node,cbounds, (unsigned int)N,userPtr);
          return node;
        },

        /* lambda function that updates BVH nodes */
        [&] (const BVHBuilderBinnedFastSpatialSAH::BuildRecord& precord, const BVHBuilderBinnedFastSpatialSAH::BuildRecord* crecords, void* node, void** children, const size_t N) -> void* {
          setNodeChildren(node,children, (unsigned int)N,userPtr);
          return node;
        },
        
        /* lambda function that creates BVH leaves */
        [&] (const PrimRef* prims, const range<size_t>& range, const FastAllocator::CachedAllocator& alloc) -> void* {
          return createLeaf((RTCThreadLocalAllocator)&alloc,(RTCBuildPrimitive*)(prims+range.begin()),range.size(),userPtr);
        },
        
        /* returns the splitter */
        [&] ( const PrimRef& prim ) -> Splitter {
          return Splitter(splitPrimitive,prim.geomID(),prim.primID(),userPtr);
        },

        /* progress monitor function */
        [&] (size_t dn) {
          if (!buildProgress) return true;
          const size_t n = progress.fetch_add(dn)+dn;
          const double f = std::min(1.0,double(n)/double(numPrimitives));
          return buildProgress(userPtr,f);
        },
        
        (PrimRef*)prims,
        pinfo.size()+settings->extraSpace,
        pinfo,*settings);
        
      bvh->allocator.cleanup();
      return root;
    }

    RTC_API void* rtcBuildBVH(const RTCBuildSettings* settings)
    {
      BVH* bvh = (BVH*) settings->bvh;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcBuildBVH);
      RTC_VERIFY_HANDLE(bvh);
      RTC_VERIFY_HANDLE(settings);
      RTC_VERIFY_HANDLE(settings->createNode);
      RTC_VERIFY_HANDLE(settings->setNodeChildren);
      RTC_VERIFY_HANDLE(settings->setNodeBounds);
      RTC_VERIFY_HANDLE(settings->createLeaf);

      /* initialize the allocator */
      bvh->allocator.init_estimate(settings->numPrimitives*sizeof(BBox3fa));
      bvh->allocator.reset();

      /* switch between differnet builders based on quality level */
      if (settings->quality == RTC_BUILD_QUALITY_LOW)
        return rtcBuildBVHMorton(settings);
      else if (settings->quality == RTC_BUILD_QUALITY_MEDIUM)
        return rtcBuildBVHBinnedSAH(settings);
      else if (settings->quality == RTC_BUILD_QUALITY_HIGH) {
        if (settings->splitPrimitive == nullptr || settings->extraSpace == 0)
          return rtcBuildBVHBinnedSAH(settings);
        else
          return rtcBuildBVHSpatialSAH(settings);
      }
      else
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid build quality");

      /* if we are in dynamic mode, then do not clear temporary data */
      if (!(settings->flags & RTC_BUILD_FLAG_DYNAMIC))
      {
        bvh->morton_src.clear();
        bvh->morton_tmp.clear();
      }

      RTC_CATCH_END(bvh->device);
      return nullptr;
    }

    RTC_API void* rtcThreadLocalAlloc(RTCThreadLocalAllocator localAllocator, size_t bytes, size_t align)
    {
      FastAllocator::CachedAllocator* alloc = (FastAllocator::CachedAllocator*) localAllocator;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcThreadLocalAlloc);
      return alloc->malloc0(bytes,align);
      RTC_CATCH_END(alloc->alloc->getDevice());
      return nullptr;
    }

    RTC_API void rtcMakeStaticBVH(RTCBVH hbvh)
    {
      BVH* bvh = (BVH*) hbvh;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcStaticBVH);
      RTC_VERIFY_HANDLE(hbvh);
      bvh->morton_src.clear();
      bvh->morton_tmp.clear();
      RTC_CATCH_END(bvh->device);
    }

    RTC_API void rtcRetainBVH(RTCBVH hbvh)
    {
      BVH* bvh = (BVH*) hbvh;
      Device* device = bvh ? bvh->device : nullptr;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcRetainBVH);
      RTC_VERIFY_HANDLE(hbvh);
      bvh->refInc();
      RTC_CATCH_END(device);
    }
    
    RTC_API void rtcReleaseBVH(RTCBVH hbvh)
    {
      BVH* bvh = (BVH*) hbvh;
      Device* device = bvh ? bvh->device : nullptr;
      RTC_CATCH_BEGIN;
      RTC_TRACE(rtcReleaseBVH);
      RTC_VERIFY_HANDLE(hbvh);
      bvh->refDec();
      RTC_CATCH_END(device);
    }
  }
}

