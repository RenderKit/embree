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


    template<int types>
      class BVHNNodeTraverserKHit
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:

      // FIXME: optimize sequence
      static __forceinline unsigned int traverseClosestHit(NodeRef& cur,
                                                           size_t mask,
                                                           const vfloat16& tNear,
                                                           const vint16& tMask,
                                                           StackItemMaskT<NodeRef>*& stackPtr,
                                                           StackItemMaskT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);         
        cur.prefetch(types);
        
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return tMask[r];
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur; // node->child(r); 
        const unsigned int d0 = ((unsigned int*)&tNear)[r];
        const int m0 = tMask[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); 
        c1.prefetch(types); 
        const unsigned int d1 = ((unsigned int*)&tNear)[r];
        const int m1 = tMask[r];

        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr->dist = d1; stackPtr++; cur = c0; return m0; }
          else         { stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr->dist = d0; stackPtr++; cur = c1; return m1; }
        }

        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr->dist = d0; stackPtr++;
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr->dist = d1; stackPtr++;

        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        NodeRef c = node->child(r); c.prefetch(types); 
        unsigned int d = ((unsigned int*)&tNear)[r]; 
        unsigned int m = tMask[r];
        stackPtr->ptr = c; stackPtr->mask = m; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; 
          unsigned int mm = stackPtr[-1].mask;
          stackPtr--;
          return mm;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); 
        c.prefetch(types); 
        m = tMask[r];
        d = *(unsigned int*)&tNear[r]; 
        stackPtr->ptr = c; stackPtr->mask = m; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; 
          unsigned int mm = stackPtr[-1].mask;
          stackPtr--;
          return mm;
        }

        /*! fallback case if more than 4 children are hit */
        StackItemMaskT<NodeRef>* stackFirst = stackPtr-4;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          c = node->child(r); 
          c.prefetch(types); 
          d = *(unsigned int*)&tNear[r]; 
          m = tMask[r];
          stackPtr->ptr  = c; 
          stackPtr->mask = m;
          stackPtr->dist = d; 
          stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        sort(stackFirst,stackPtr);
        cur = (NodeRef) stackPtr[-1].ptr; 
        unsigned int mm = stackPtr[-1].mask;
        stackPtr--;
        return mm;
      }


      static __forceinline int traverseAnyHit(NodeRef& cur,
                                              size_t mask,
                                              const vfloat16& tNear,
                                              const vint16& tMask,
                                              StackItemMaskT<NodeRef>*& stackPtr,
                                              StackItemMaskT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);         
        cur.prefetch(types);
        
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return tMask[r];
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur; // node->child(r); 
        const unsigned int d0 = ((unsigned int*)&tNear)[r];
        const int m0 = tMask[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); 
        c1.prefetch(types); 
        const unsigned int d1 = ((unsigned int*)&tNear)[r];
        const int m1 = tMask[r];

        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr++; cur = c0; return m0; }
          else         { stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr++; cur = c1; return m1; }
        }

        /*! Here starts the slow path for 3+ hit children. */
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr++;
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr++;

        StackItemMaskT<NodeRef>* stackFirst = stackPtr-2;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          NodeRef c = node->child(r); 
          c.prefetch(types); 
          int m = tMask[r];
          stackPtr->ptr  = c; 
          stackPtr->mask = m;
          stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        cur = (NodeRef) stackPtr[-1].ptr; 
        unsigned int mm = stackPtr[-1].mask;
        stackPtr--;
        return mm;
      }

    };

      /*! BVH hybrid packet intersector. Switches between packet and single ray traversal (optional). */
      template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single = true>
        class BVHNIntersectorKHybrid2
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
