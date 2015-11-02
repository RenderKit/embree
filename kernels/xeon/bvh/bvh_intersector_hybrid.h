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
    class BVHNIntersectorKHybrid
    {
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

      static const size_t stackSizeChunk = N*BVH::maxDepth+1;

      static const size_t switchThreshold = (K==4)  ? 3 :
                                            (K==8)  ? ((N==4) ? 5 : 7) :
                                            (K==16) ? 7 :
                                                      0;

    public:
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
    };

    /*! BVH packet intersector. */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK>
    class BVHNIntersectorKChunk : public BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,false> {};

#if defined(__AVX512F__)
    template<int types, typename PrimitiveIntersectorK>
      class BVHNIntersectorKHybrid<8,16,types,false,PrimitiveIntersectorK,true>
    {
      static const int N = 8;
      static const int K = 16;


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

      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   const BaseNode* node,
                                                   size_t mask,
                                                   const vfloat16& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        //const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); 
          cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }
        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); 
        c0.prefetch(types); 
        const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); 
        c1.prefetch(types); 
        const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; return; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; return; }
        }

        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;

        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        NodeRef c = node->child(r); 
        c.prefetch(types); 
        unsigned int d = ((unsigned int*)&tNear)[r]; 
        stackPtr->ptr = c; 
        stackPtr->dist = d; 
        stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); 
        c.prefetch(types); 
        d = *(unsigned int*)&tNear[r]; 
        stackPtr->ptr = c; 
        stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! fallback case if more than 4 children are hit */
        StackItemT<NodeRef>* stackFirst = stackPtr-4;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          c = node->child(r); 
          c.prefetch(types); 
          d = *(unsigned int*)&tNear[r]; 
          stackPtr->ptr = c; 
          stackPtr->dist = d; 
          stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        sort(stackFirst,stackPtr);
        cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
      }

    public:
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
    };
#endif

  }


}
