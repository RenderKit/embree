// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "bvh4hair.h"
#include "common/ray.h"
#include "common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH4Hair single ray traversal implementation. */
    class BVH4HairIntersector1 
    {
      /* shortcuts for frequently used types */
      typedef BVH4Hair::simdb simdb;
      typedef BVH4Hair::simdi simdi;
      typedef BVH4Hair::simdf simdf;
      typedef Vec3<BVH4Hair::simdf> simd3f;
      typedef BVH4Hair::NodeRef NodeRef;
      typedef BVH4Hair::Node Node;
      typedef BVH4Hair::AlignedNode AlignedNode;
      typedef BVH4Hair::UnalignedNode UnalignedNode;
      typedef BVH4Hair::Bezier1 Bezier1;
      typedef BVH4Hair::NAABBox3fa NAABBox3fa;
      typedef BVH4Hair::AffineSpaceSOA4 AffineSpaceSOA4;

      static const size_t stackSize = 1+3*BVH4Hair::maxDepth;

      struct __aligned(16) StackItem 
      {
      public:
        __forceinline static void swap2(StackItem& a, StackItem& b) 
        { 
#if defined(__AVX__) && !BVH4HAIR_NAVIGATION
          ssef sse_a = load4f(&a);
          ssef sse_b = load4f(&b);
          store4f(&a,sse_b);
          store4f(&b,sse_a);
#else
          StackItem t = b; b = a; a = t;
#endif
    }

        __forceinline friend bool operator<(const StackItem& s1, const StackItem& s2) {
          return s1.tNear > s2.tNear;
        }

        /*! Sort 2 stack items. */
        __forceinline friend void sort(StackItem& s1, StackItem& s2) {
          if (s2.tNear < s1.tNear) swap2(s2,s1);
        }
        
        /*! Sort 3 stack items. */
        __forceinline friend void sort(StackItem& s1, StackItem& s2, StackItem& s3)
        {
          if (s2.tNear < s1.tNear) swap2(s2,s1);
          if (s3.tNear < s2.tNear) swap2(s3,s2);
          if (s2.tNear < s1.tNear) swap2(s2,s1);
        }
    
        /*! Sort 4 stack items. */
        __forceinline friend void sort(StackItem& s1, StackItem& s2, StackItem& s3, StackItem& s4)
        {
          if (s2.tNear < s1.tNear) swap2(s2,s1);
          if (s4.tNear < s3.tNear) swap2(s4,s3);
          if (s3.tNear < s1.tNear) swap2(s3,s1);
          if (s4.tNear < s2.tNear) swap2(s4,s2);
          if (s3.tNear < s2.tNear) swap2(s3,s2);
        }

      public:
        size_t ref;
        float tNear,tFar;
#if BVH4HAIR_NAVIGATION
        size_t depth;
#endif
      };

    private:
      static size_t intersectBox(const UnalignedNode* node, Ray& ray, const simd3f& org, const simd3f& dir, simdf& tNear, simdf& tFar);
      static void intersectBezier(const LinearSpace3fa& ray_space, Ray& ray, const Bezier1& bezier, const Scene* scene);
      static bool occludedBezier(const LinearSpace3fa& ray_space, Ray& ray, const Bezier1& bezier, const Scene* scene);
      
    public:
      static void intersect(const BVH4Hair* This, Ray& ray);
      static void occluded (const BVH4Hair* This, Ray& ray);
    };
  }
}
