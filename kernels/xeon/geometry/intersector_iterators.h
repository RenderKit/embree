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

#include "../../common/default.h"
#include "../../common/scene.h"
#include "../../common/ray.h"

namespace embree
{
  namespace isa
  {
    template<typename Intersector>
      struct ListIntersector1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(pre,ray,*prim,scene,geomID_to_instID);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node) 
        {
          while (true) {
            if (Intersector::occluded(pre,ray,*prim,scene,geomID_to_instID))
              return true;
            if (prim->last()) break;
            prim++;
          }
          return false;
        }
      };
    
    template<typename Intersector>
      struct ArrayIntersector1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++)
            Intersector::intersect(pre,ray,prim[i],scene,geomID_to_instID);
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, const Primitive* prim, size_t num, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,prim[i],scene,geomID_to_instID))
              return true;
          }
          return false;
        }
      };
    
    template<typename Intersector1, typename Intersector2>
      struct Switch2Intersector1
      {
        typedef typename Intersector1::Primitive Primitive1;
        typedef typename Intersector2::Primitive Primitive2;
        typedef void Primitive;
        
        struct Precalculations 
        {
          __forceinline Precalculations (const Ray& ray, const void *ptr) 
            : pre1(ray,ptr), pre2(ray,ptr) {}
          
          typename Intersector1::Precalculations pre1;
          typename Intersector2::Precalculations pre2;
        };
        
        static __forceinline void intersect(Precalculations& pre, Ray& ray, Primitive* prim, size_t ty, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node)
        {
          if (ty == 0) Intersector1::intersect(pre.pre1,ray,*(Primitive1*)prim,scene,geomID_to_instID,lazy_node);
          else         Intersector2::intersect(pre.pre2,ray,*(Primitive2*)prim,scene,geomID_to_instID,lazy_node);
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray& ray, Primitive* prim, size_t ty, Scene* scene, const unsigned* geomID_to_instID, size_t& lazy_node) 
        {
          if (ty == 0) return Intersector1::occluded(pre.pre1,ray,*(Primitive1*)prim,scene,geomID_to_instID,lazy_node);
          else         return Intersector2::occluded(pre.pre2,ray,*(Primitive2*)prim,scene,geomID_to_instID,lazy_node);
        }
      };
    
#if defined __SSE__
    
    template<typename Intersector>
      struct ListIntersector4
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(valid,pre,ray,*prim,scene);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline vbool4 occluded(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool4 valid0 = valid;
          while (true) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,scene);
            if (none(valid0)) break;
            if (prim->last()) break;
            prim++;
          }
          return !valid0;
        }
      };
    
    template<typename Intersector>
      struct ArrayIntersector4
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool4 occluded(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool4 valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
      };
    
    
    template<typename Intersector>
      struct ListIntersector4_1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(valid,pre,ray,*prim,scene);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline vbool4 occluded(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool4 valid0 = valid;
          while (true) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,scene);
            if (none(valid0)) break;
            if (prim->last()) break;
            prim++;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(pre,ray,k,*prim,scene);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          while (true) {
            if (Intersector::occluded(pre,ray,k,*prim,scene))
              return true;
            if (prim->last()) break;
            prim++;
          }
          return false;
        }
      };
    
    template<typename Intersector>
      struct ArrayIntersector4_1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool4 occluded(const vbool4& valid, Precalculations& pre, Ray4& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool4 valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(pre,ray,k,prim[i],scene);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,k,prim[i],scene))
              return true;
          }
          return false;
        }
      };
    
#endif
    
#if defined __AVX__
    
    template<typename Intersector>
      struct ListIntersector8
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(valid,pre,ray,*prim,scene);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline vbool8 occluded(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool8 valid0 = valid;
          while (true) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,scene);
            if (none(valid0)) break;
            if (prim->last()) break;
            prim++;
          }
          return !valid0;
        }
      };
    
    template<typename Intersector>
      struct ArrayIntersector8
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool8 occluded(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool8 valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
      };
    
    
    template<typename Intersector>
      struct ListIntersector8_1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(valid,pre,ray,*prim,scene);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline vbool8 occluded(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool8 valid0 = valid;
          while (true) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,*prim,scene);
            if (none(valid0)) break;
            if (prim->last()) break;
            prim++;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          while (true) {
            Intersector::intersect(pre,ray,k,*prim,scene);
            if (prim->last()) break;
            prim++;
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          while (true) {
            if (Intersector::occluded(pre,ray,k,*prim,scene))
              return true;
            if (prim->last()) break;
            prim++;
          }
          return false;
        }
      };
    
    template<typename Intersector>
      struct ArrayIntersector8_1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool8 occluded(const vbool8& valid, Precalculations& pre, Ray8& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool8 valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(pre,ray,k,prim[i],scene);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray8& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,k,prim[i],scene))
              return true;
          }
          return false;
        }
      };
    
#endif

#if defined(__AVX512F__)
    template<typename Intersector>
      struct ArrayIntersector16
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool16& valid, Precalculations& pre, Ray16& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool16 occluded(const vbool16& valid, Precalculations& pre, Ray16& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool16 valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }

        static __forceinline void intersect(Precalculations& pre, Ray16& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(pre,ray,k,prim[i],scene);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray16& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,k,prim[i],scene))
              return true;
          }
          return false;
        }

      };


    template<typename Intersector>
      struct ArrayIntersector16_1
      {
        typedef typename Intersector::Primitive Primitive;
        typedef typename Intersector::Precalculations Precalculations;
        
        static __forceinline void intersect(const vbool16& valid, Precalculations& pre, Ray16& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(valid,pre,ray,prim[i],scene);
          }
        }
        
        static __forceinline vbool16 occluded(const vbool16& valid, Precalculations& pre, Ray16& ray, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          vbool16 valid0 = valid;
          for (size_t i=0; i<num; i++) {
            valid0 &= !Intersector::occluded(valid0,pre,ray,prim[i],scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        static __forceinline void intersect(Precalculations& pre, Ray16& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node)
        {
          for (size_t i=0; i<num; i++) {
            Intersector::intersect(pre,ray,k,prim[i],scene);
          }
        }
        
        static __forceinline bool occluded(Precalculations& pre, Ray16& ray, size_t k, const Primitive* prim, size_t num, Scene* scene, size_t& lazy_node) 
        {
          for (size_t i=0; i<num; i++) {
            if (Intersector::occluded(pre,ray,k,prim[i],scene))
              return true;
          }
          return false;
        }
      };

#endif
  }
}
