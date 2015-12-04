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

#include "bvh.h"
#include "../../common/ray.h"
#include "../../common/stack_item.h"

namespace embree
{
  namespace isa 
  {

    /*! BVH hybrid packet intersector. Switches between packet and single ray traversal (optional). */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single = true>
    class BVHNIntersectorKHybrid2
    {

      /*! An item on the stack holds the node ID and distance of that node. */
      template<typename T>
      struct __aligned(16) StackItemMaskT
      {
        /*! assert that the xchg function works */
        static_assert(sizeof(T) <= 8, "sizeof(T) <= 8 failed");

        /*! use SSE instructions to swap stack items */
        __forceinline static void xchg(StackItemMaskT& a, StackItemMaskT& b) 
        { 
          const vfloat4 sse_a = vfloat4::load((float*)&a); 
          const vfloat4 sse_b = vfloat4::load((float*)&b);
          vfloat4::store(&a,sse_b);
          vfloat4::store(&b,sse_a);
        }

        /*! Sort 2 stack items. */
        __forceinline friend void sort(StackItemMaskT& s1, StackItemMaskT& s2) {
          if (s2.dist < s1.dist) xchg(s2,s1);
        }
    
        /*! Sort 3 stack items. */
        __forceinline friend void sort(StackItemMaskT& s1, StackItemMaskT& s2, StackItemMaskT& s3)
        {
          if (s2.dist < s1.dist) xchg(s2,s1);
          if (s3.dist < s2.dist) xchg(s3,s2);
          if (s2.dist < s1.dist) xchg(s2,s1);
        }
    
        /*! Sort 4 stack items. */
        __forceinline friend void sort(StackItemMaskT& s1, StackItemMaskT& s2, StackItemMaskT& s3, StackItemMaskT& s4)
        {
          if (s2.dist < s1.dist) xchg(s2,s1);
          if (s4.dist < s3.dist) xchg(s4,s3);
          if (s3.dist < s1.dist) xchg(s3,s1);
          if (s4.dist < s2.dist) xchg(s4,s2);
          if (s3.dist < s2.dist) xchg(s3,s2);
        }

        /*! Sort N stack items. */
        __forceinline friend void sort(StackItemMaskT* begin, StackItemMaskT* end)
        {
          for (StackItemMaskT* i = begin+1; i != end; ++i)
          {
            const vfloat4 item = vfloat4::load((float*)i);
            const unsigned int dist = i->dist;
            StackItemMaskT* j = i;

            while ((j != begin) && ((j-1)->dist < dist))
            {
              vfloat4::store(j, vfloat4::load((float*)(j-1)));
              --j;
            }

            vfloat4::store(j, item);
          }
        }
    
      public:
        unsigned int dist;        
        unsigned int mask;
        T ptr; 
      };

      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersectorK::Precalculations Precalculations;
      typedef typename PrimitiveIntersectorK::Primitive Primitive;
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;
      typedef typename BVH::Node Node;
      typedef typename BVH::NodeMB NodeMB;
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef Vec3<vint<K>> Vec3viK;

      static const size_t stackSizeChunk  = N*BVH::maxDepth+1;
      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

      static const size_t switchThreshold = (K==4)  ? 3 :
                                            (K==8)  ? ((N==4) ? 5 : 7) :
                                            (K==16) ? 7 :
                                                      0;

    public:
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
    };

  }
}
