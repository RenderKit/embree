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

#include "../../common/scene.h"
#include "../../common/ray.h"

namespace embree
{
  namespace isa
  {
    template<typename Intersector>
      struct ArrayIntersector1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(Precalculations& pre, Ray& ray, size_t ty, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++)
            Intersector::intersect(pre,ray,prim[i],scene,geomID_to_instID);
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, size_t ty, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,prim[i],scene,geomID_to_instID))
              return true;
          }
          return false;
        }
      };

    template<typename Intersector1, typename Intersector2>
      struct Select2Intersector1
      {
        typedef void* Primitive;
        typedef typename Intersector1::Primitive* Primitive1;
        typedef typename Intersector2::Primitive* Primitive2;
        typedef typename Intersector1::Precalculations Precalculations1;
        typedef typename Intersector2::Precalculations Precalculations2;

        struct Precalculations
        {
          __forceinline Precalculations (const Ray& ray, const void* ptr)
            : pre1(ray,ptr), pre2(ray,ptr) {}

        public:
          Precalculations1 pre1;
          Precalculations2 pre2;
        };
        
        static __forceinline void intersect(Precalculations& pre, Ray& ray, size_t ty, const Primitive* prim_i, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node)
        {
          if (likely(ty == 0)) {
            Primitive1 prim = (Primitive1) prim_i;
            for (size_t i=0; i<num; i++)
              Intersector1::intersect(pre.pre1,ray,prim[i],scene,geomID_to_instID);
          } else {
            Primitive2 prim = (Primitive2) prim_i;
            for (size_t i=0; i<num; i++)
              Intersector2::intersect(pre.pre2,ray,prim[i],scene,geomID_to_instID);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, size_t ty, const Primitive* prim_i, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node) 
        {
          if (likely(ty == 0)) {
            Primitive1 prim = (Primitive1) prim_i;
            for (size_t i=0; i<num; i++) {
              if (Intersector1::occluded(pre.pre1,ray,prim[i],scene,geomID_to_instID))
                return true;
            }
          } else {
            Primitive2 prim = (Primitive2) prim_i;
            for (size_t i=0; i<num; i++) {
              if (Intersector2::occluded(pre.pre2,ray,prim[i],scene,geomID_to_instID))
                return true;
            }
          }
          return false;
        }
      };
    
    template<int K, typename Intersector>
      struct ArrayIntersectorK
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool<K> occluded(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool<K> valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }

        /* Dummy functions for templates */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) {}
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) { return false; }
      };
    
    template<int K, typename Intersector>
      struct ArrayIntersectorK_1 
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool<K> occluded(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool<K> valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(pre,ray,k,prim[i],scene);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,k,prim[i],scene))
              return true;
          }
          return false;
        }
      };
  }
}
