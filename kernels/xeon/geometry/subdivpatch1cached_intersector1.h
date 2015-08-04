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

namespace embree
{
  namespace isa
  {
    class SubdivPatch1CachedIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      typedef GridSOAIntersector1::Precalculations Precalculations;
      
      static __forceinline bool processLazyNode(Precalculations& pre, Primitive* prim, Scene* scene, size_t& lazy_node)
      {
        if (pre.grid) SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
        GridSOA* grid = (GridSOA*) SharedLazyTessellationCache::lookup(prim->entry(),scene->commitCounter,[&] () {
            auto alloc = [] (const size_t bytes) { return SharedLazyTessellationCache::sharedLazyTessellationCache.malloc(bytes); };
            return GridSOA::create(prim,scene,alloc);
          });
        //GridSOA* grid = (GridSOA*) prim->root_ref.data;
        //GridSOA* grid = (GridSOA*) prim;
        lazy_node = grid->root;
        pre.grid = grid;
        return false;
      }

      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        if (likely(ty == 0)) GridSOAIntersector1::intersect(pre,ray,prim,ty,scene,lazy_node);
        else                 processLazyNode(pre,prim,scene,lazy_node);
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        if (likely(ty == 0)) return GridSOAIntersector1::occluded(pre,ray,prim,ty,scene,lazy_node);
        else                 return processLazyNode(pre,prim,scene,lazy_node);
      }      
    };

#if defined(__SSE__) && 0
    struct SubdivPatch1CachedIntersector4
    {
      typedef SubdivPatch1Cached Primitive;
      typedef typename GridSOAIntersectorN::Precalculations Precalculations;
      
      static __forceinline bool processLazyNode(Precalculations& pre, Primitive* prim, Scene* scene, size_t& lazy_node)
      {
        if (pre.grid) SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
        GridSOA* grid = (GridSOA*) SharedLazyTessellationCache::lookup(prim->entry(),scene->commitCounter,[&] () {
            auto alloc = [] (const size_t bytes) { return SharedLazyTessellationCache::sharedLazyTessellationCache.malloc(bytes); };
            return GridSOA::create(prim,scene,alloc);
          });
        lazy_node = grid->root;
        pre.grid = grid;
        return false;
      }
      
      static __forceinline void intersect(const bool4& valid, Precalculations& pre, Ray4& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
      {
        if (likely(ty == 0)) GridSOAIntersectorN::intersect(valid,pre,ray,prim,ty,scene,lazy_node);
        else                 processLazyNode(pre,prim,scene,lazy_node);
      }
      
      static __forceinline bool4 occluded(const bool4& valid, Precalculations& pre, Ray4& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        if (likely(ty == 0)) return GridSOAIntersectorN::occluded(valid,pre,ray,prim,ty,scene,lazy_node);
        else                 return processLazyNode(pre,prim,scene,lazy_node);
      }
      
      static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
      {
        if (likely(ty == 0)) GridSOAIntersectorN::intersect(pre,ray,k,prim,ty,scene,lazy_node);
        else                 processLazyNode(pre,prim,scene,lazy_node);
      }
      
      static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        if (likely(ty == 0)) return GridSOAIntersectorN::occluded(pre,ray,k,prim,ty,scene,lazy_node);
        else                 return processLazyNode(pre,prim,scene,lazy_node);
      }
    };
#endif

#if defined(__AVX__) && 0
    struct SubdivPatch1CachedIntersector8
    {
      typedef SubdivPatch1Cached Primitive;
      typedef typename GridSOAIntersectorN::Precalculations Precalculations;
      
      static __forceinline bool processLazyNode(Precalculations& pre, Primitive* prim, Scene* scene, size_t& lazy_node)
      {
        if (pre.grid) SharedLazyTessellationCache::sharedLazyTessellationCache.unlock();
        GridSOA* grid = (GridSOA*) SharedLazyTessellationCache::lookup(prim->entry(),scene->commitCounter,[&] () {
            auto alloc = [] (const size_t bytes) { return SharedLazyTessellationCache::sharedLazyTessellationCache.malloc(bytes); };
            return GridSOA::create(prim,scene,alloc);
          });
        lazy_node = grid->root;
        pre.grid = grid;
        return false;
      }
      
      static __forceinline void intersect(const bool8& valid, Precalculations& pre, Ray8& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
      {
        if (likely(ty == 0)) GridSOAIntersectorN::intersect(valid,pre,ray,prim,ty,scene,lazy_node);
        else                 processLazyNode(pre,prim,scene,lazy_node);
      }
      
      static __forceinline bool8 occluded(const bool8& valid, Precalculations& pre, Ray8& ray, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        if (likely(ty == 0)) return GridSOAIntersectorN::occluded(valid,pre,ray,prim,ty,scene,lazy_node);
        else                 return processLazyNode(pre,prim,scene,lazy_node);
      }
      
      static __forceinline void intersect(Precalculations& pre, Ray8& ray, size_t k, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node)
      {
        if (likely(ty == 0)) GridSOAIntersectorN::intersect(pre,ray,k,prim,ty,scene,lazy_node);
        else                 processLazyNode(pre,prim,scene,lazy_node);
      }
      
      static __forceinline bool occluded(Precalculations& pre, Ray8& ray, size_t k, Primitive* prim, size_t ty, Scene* scene, size_t& lazy_node) 
      {
        if (likely(ty == 0)) return GridSOAIntersectorN::occluded(pre,ray,k,prim,ty,scene,lazy_node);
        else                 return processLazyNode(pre,prim,scene,lazy_node);
      }
    };
#endif
  }
}
