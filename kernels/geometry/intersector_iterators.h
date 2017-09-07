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

#include "../common/scene.h"
#include "../common/ray.h"
#include "../common/accel.h"
#include "../geometry/primitive.h"

namespace embree
{
  namespace isa
  {
    template<
      typename Intersector0,
      typename Intersector1,
      typename Intersector2,
      typename Intersector3,
      typename Intersector4,
      typename Intersector5,
      typename Intersector6,
      typename Intersector7>

      struct Virtual8LeafIntersector1
      {
        typedef void* Primitive;

        struct Precalculations
        {
          __forceinline Precalculations (const Ray& ray, const AccelData* accel)
            : leaf_intersector(accel->leaf_intersector), pre0(ray,accel), pre1(ray,accel), pre2(ray,accel), pre3(ray,accel), pre4(ray,accel), pre5(ray,accel), pre6(ray,accel), pre7(ray,accel) 
          {
            table[0] = &pre0;
            table[1] = &pre1;
            table[2] = &pre2;
            table[3] = &pre3;
            table[4] = &pre4;
            table[5] = &pre5;
            table[6] = &pre6;
            table[7] = &pre7;
          }
        
        public:
          const LeafIntersector* leaf_intersector;
          typename Intersector0::Precalculations pre0;
          typename Intersector1::Precalculations pre1;
          typename Intersector2::Precalculations pre2;
          typename Intersector3::Precalculations pre3;
          typename Intersector4::Precalculations pre4;
          typename Intersector5::Precalculations pre5;
          typename Intersector6::Precalculations pre6;
          typename Intersector7::Precalculations pre7;
          void* table[8];
        };

        static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
        {
          //const unsigned int ty = (unsigned int) Leaf::decodeTy(*(unsigned int*)prim);
          assert(ty < 8);
          if (likely(ty == 0)) {
            Intersector0::intersect(pre.pre0,ray,context,(typename Intersector0::Primitive*)prim,ty,lazy_node);
            //} else if (ty == 1) {
            //Intersector1::intersect(pre.pre1,ray,context,(typename Intersector1::Primitive*)prim,ty,lazy_node);
          } else if (ty == 2) {
            Intersector2::intersect(pre.pre2,ray,context,(typename Intersector2::Primitive*)prim,ty,lazy_node);
            //} else if (ty == 3) {
            //Intersector3::intersect(pre.pre3,ray,context,(typename Intersector3::Primitive*)prim,ty,lazy_node);
          } else {
            pre.leaf_intersector->vtable1[ty].intersect(pre.table[ty],ray,context,prim,ty,lazy_node);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
        {
          //const unsigned int ty = (unsigned int) Leaf::decodeTy(*(unsigned int*)prim);
          assert(ty < 8);
          if (likely(ty == 0)) {
            return Intersector0::occluded(pre.pre0,ray,context,(typename Intersector0::Primitive*)prim,ty,lazy_node);
            //} else if (ty == 1) {
            //return Intersector1::occluded(pre.pre1,ray,context,(typename Intersector1::Primitive*)prim,ty,lazy_node);
          } else if (ty == 2) {
            return Intersector2::occluded(pre.pre2,ray,context,(typename Intersector2::Primitive*)prim,ty,lazy_node);
            //} else if (ty == 3) {
            //return Intersector3::occluded(pre.pre3,ray,context,(typename Intersector3::Primitive*)prim,ty,lazy_node);
          } else {
            return pre.leaf_intersector->vtable1[ty].occluded(pre.table[ty],ray,context,prim,ty,lazy_node);
          }
        }
      };

    template<typename Intersector>
      struct ArrayIntersector1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;

        static const bool validIntersectorK = false; // FIXME: why do we need this

        static __forceinline void intersect(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
        {
          for (size_t i=0;; i++) {
            Intersector::intersect(pre,ray,context,prim[i]);
            if (prim[i].last()) break;
          }
        }

        static __forceinline void vintersect(void* pre, Ray& ray, IntersectContext* context, const void* prim, size_t ty, size_t& lazy_node) {
          intersect(*(Precalculations*)pre,ray,context,(const Primitive*)prim,ty,lazy_node);
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
        {
          for (size_t i=0;; i++) {
            if (Intersector::occluded(pre,ray,context,prim[i]))
              return true;
            if (prim[i].last()) break;
          }
          return false;
        }

        static __forceinline bool voccluded(void* pre, Ray& ray, IntersectContext* context, const void* prim, size_t ty, size_t& lazy_node) {
          return occluded(*(Precalculations*)pre,ray,context,(const Primitive*)prim,ty,lazy_node);
        }

        template<int K>
        static __forceinline void intersectK(const vbool<K>& valid, /* PrecalculationsK& pre, */ RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
        {
        }

        template<int K>        
        static __forceinline vbool<K> occludedK(const vbool<K>& valid, /* PrecalculationsK& pre, */ RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
        {
          return valid;
        }
      };

    template<int K, typename Intersector>
      struct ArrayIntersectorK_1 
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
        {
          for (size_t i=0;; i++) {
            Intersector::intersect(valid,pre,ray,context,prim[i]);
            if (prim[i].last()) break;
          }
        }
        
        static __forceinline vbool<K> occluded(const vbool<K>& valid, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
        {
          vbool<K> valid0 = valid;
          for (size_t i=0;; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,context,prim[i]);
            if (none(valid0)) break;
            if (prim[i].last()) break;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node)
        {
          for (size_t i=0;; i++) {
            Intersector::intersect(pre,ray,k,context,prim[i]);
            if (prim[i].last()) break;
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t ty, size_t& lazy_node) 
        {
          for (size_t i=0;; i++) {
            if (Intersector::occluded(pre,ray,k,context,prim[i]))
              return true;
            if (prim[i].last()) break;
          }
          return false;
        }
      };

    // =============================================================================================

    template<int K, typename IntersectorK>
      struct ArrayIntersectorKStream 
      {
        typedef typename IntersectorK::Primitive PrimitiveK;
        typedef typename IntersectorK::Precalculations PrecalculationsK;

        static const bool validIntersectorK = true;
        
        static __forceinline void intersectK(const vbool<K>& valid, /* PrecalculationsK& pre, */ RayK<K>& ray, IntersectContext* context, const PrimitiveK* prim, size_t ty, size_t& lazy_node)
        {
          PrecalculationsK pre(valid,ray); // FIXME: might cause trouble

          for (size_t i=0; ; i++) {
            IntersectorK::intersect(valid,pre,ray,context,prim[i]);
            if (prim[i].last()) break;
          }
        }
        
        static __forceinline vbool<K> occludedK(const vbool<K>& valid, /* PrecalculationsK& pre, */ RayK<K>& ray, IntersectContext* context, const PrimitiveK* prim, size_t ty, size_t& lazy_node)
        {
          PrecalculationsK pre(valid,ray); // FIXME: might cause trouble
          vbool<K> valid0 = valid;
          for (size_t i=0; ; i++) {
            valid0 &= !IntersectorK::occluded(valid0,pre,ray,context,prim[i]);
            if (none(valid0)) break;
            if (prim[i].last()) break;
          }
          return !valid0;
        }
      };
  }
}
