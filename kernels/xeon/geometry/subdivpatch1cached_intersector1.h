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

#pragma once

#include "subdivpatch1cached.h"
#include "grid_soa_intersector1.h"
#include "grid_aos_intersector1.h"
#include "../../common/ray.h"

#define GRID_SOA 1

namespace embree
{
  namespace isa
  {
    class SubdivPatch1CachedIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;

#if GRID_SOA
      typedef GridSOAIntersector1::Precalculations Precalculations;
#else
      struct Precalculations : public GridAOSIntersector1::Precalculations
      {
        __forceinline Precalculations (const Ray& ray, const void *ptr) 
          :  patch(nullptr), GridAOSIntersector1::Precalculations(ray,ptr) {}

         __forceinline ~Precalculations() 
         {
           if (patch)
             SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
         }
         
      public:
         SubdivPatch1Cached* patch;
      };
#endif
      
      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        STAT3(normal.trav_prims,1,1,1);
        
        if (likely(ty == 0)) {
#if GRID_SOA
          GridSOAIntersector1::intersect(pre,ray,prim,ty,scene,lazy_node);
#else
          GridAOSIntersector1::intersect(pre,ray,(GridAOS::EagerLeaf*)prim,ty,scene,lazy_node);
#endif
        }
        else {
          if (pre.grid) SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
          GridSOA* grid = (GridSOA*) SharedLazyTessellationCache::lookup(prim->entry(),scene->commitCounter,[&] () {
              auto alloc = [] (const size_t bytes) { return SharedLazyTessellationCache::sharedLazyTessellationCache.malloc(bytes); };
#if GRID_SOA
              return GridSOA::create(prim,scene,alloc);
#else
              return GridAOS::create(prim,scene,alloc);
#endif
            });
          lazy_node = grid->root;
          pre.grid = grid;
        }
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        STAT3(shadow.trav_prims,1,1,1);
        
        if (likely(ty == 0)) {
#if GRID_SOA
          return GridSOAIntersector1::occluded(pre,ray,prim,ty,scene,lazy_node);
#else
          return GridAOSIntersector1::occluded(pre,ray,(GridAOS::EagerLeaf*)prim,ty,scene,lazy_node);
#endif
        }
        else {
          if (pre.grid) SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
          GridSOA* grid = (GridSOA*) SharedLazyTessellationCache::lookup(prim->entry(),scene->commitCounter,[&] () {
              auto alloc = [] (const size_t bytes) { return SharedLazyTessellationCache::sharedLazyTessellationCache.malloc(bytes); };
#if GRID_SOA
              return GridSOA::create(prim,scene,alloc);
#else
              return GridAOS::create(prim,scene,alloc);
#endif
            });
          lazy_node = grid->root;
          pre.grid = grid;
        }
        return false;
      }      
    };
  }
}
