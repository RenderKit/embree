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

#pragma once

#include "subdivpatch1cached.h"
#include "grid_soa_intersector1.h"
#include "grid_soa_intersector_packet.h"
#include "../common/ray.h"

namespace embree
{
  namespace isa
  {
    class SubdivPatch1CachedIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      typedef GridSOAIntersector1::Precalculations Precalculations;

      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
      {
        GridSOAIntersector1::intersect(pre,ray,context,prim,lazy_node);
      }
      static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, size_t ty0, const Primitive* prim, size_t ty, size_t& lazy_node) {
        intersect(pre,ray,context,prim,lazy_node);
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
      {
        return GridSOAIntersector1::occluded(pre,ray,context,prim,lazy_node);
      }
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, size_t ty0, const Primitive* prim, size_t ty, size_t& lazy_node) {
        return occluded(pre,ray,context,prim,ty,lazy_node);
      }
    };

    class SubdivPatch1CachedMBIntersector1
    {
    public:
      typedef SubdivPatch1Cached Primitive;
      typedef GridSOAMBIntersector1::Precalculations Precalculations;
      
      /*! Intersect a ray with the primitive. */
      static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
      {
        GridSOAMBIntersector1::intersect(pre,ray,context,prim,lazy_node);
      }
      static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, size_t ty0, const Primitive* prim, size_t ty, size_t& lazy_node) {
        intersect(pre,ray,context,prim,ty,lazy_node);
      }
      
      /*! Test if the ray is occluded by the primitive */
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
      {
        return GridSOAMBIntersector1::occluded(pre,ray,context,prim,lazy_node);
      }
      static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, size_t ty0, const Primitive* prim, size_t ty, size_t& lazy_node) {
        return occluded(pre,ray,context,prim,ty,lazy_node);
      }
    };

    template <int K>
      struct SubdivPatch1CachedIntersectorK
    {
      typedef SubdivPatch1Cached Primitive;
      typedef GridSOAIntersectorK<K>::Precalculations Precalculations;
            
      static __forceinline void intersect(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        GridSOAIntersectorK<K>::intersect(valid,pre,ray,context,prim,lazy_node);
      }
      
      static __forceinline vbool<K> occluded(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        return GridSOAIntersectorK<K>::occluded(valid,pre,ray,context,prim,lazy_node);
      }
      
      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        GridSOAIntersectorK<K>::intersect(pre,ray,k,context,prim,lazy_node);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        GridSOAIntersectorK<K>::occluded(pre,ray,k,context,prim,lazy_node);
      }
    };

    typedef SubdivPatch1CachedIntersectorK<4>  SubdivPatch1Intersector4;
    typedef SubdivPatch1CachedIntersectorK<8>  SubdivPatch1Intersector8;
    typedef SubdivPatch1CachedIntersectorK<16> SubdivPatch1Intersector16;


    template <int K>
      struct SubdivPatch1CachedMBIntersectorK
    {
      typedef SubdivPatch1Cached Primitive;
      typedef GridSOAMBIntersectorK<K>::Precalculations Precalculations;
            
      static __forceinline void intersect(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        GridSOAMBIntersectorK<K>::intersect(valid,pre,ray,context,prim,lazy_node);
      }

      static __forceinline vbool<K> occluded(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        return GridSOAMBIntersectorK<K>::occluded(valid,pre,ray,context,prim,lazy_node);
      }

      static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        GridSOAMBIntersectorK<K>::intersect(pre,ray,k,context,prim,lazy_node);
      }
      
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
      {
        return GridSOAMBIntersectorK<K>::occluded(pre,ray,k,context,prim,lazy_node);
      }
    };

    typedef SubdivPatch1CachedMBIntersectorK<4>  SubdivPatch1MBIntersector4;
    typedef SubdivPatch1CachedMBIntersectorK<8>  SubdivPatch1MBIntersector8;
    typedef SubdivPatch1CachedMBIntersectorK<16> SubdivPatch1MBIntersector16;
  }
}

