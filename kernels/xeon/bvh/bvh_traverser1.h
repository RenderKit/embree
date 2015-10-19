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
#include "../../common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH node traversal for single rays. */
    template<int N, int types>
    class BVHNTraverser1;

    /* Specialization for BVH4. */
    template<int types>
    class BVHNTraverser1<4,types>
    {
      typedef BVH4 BVH;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat4& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
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
        NodeRef c = node->child(r); c.prefetch(types); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
        cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
      }

      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat4& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; return; }
          else         { *stackPtr = c0; stackPtr++; cur = c1; return; }
        }
        assert(stackPtr < stackEnd);
        *stackPtr = c0; stackPtr++;
        assert(stackPtr < stackEnd);
        *stackPtr = c1; stackPtr++;

        /*! three children are hit */
        r = __bscf(mask);
        cur = node->child(r); cur.prefetch(types);
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        /*! four children are hit */
        cur = node->child(3); cur.prefetch(types);
        assert(cur != BVH::emptyNode);
      }
    };

    /* Specialization for BVH8. */
    template<int types>
    class BVHNTraverser1<8,types>
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat8& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
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
        NodeRef c = node->child(r); c.prefetch(types); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
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
          c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        sort(stackFirst,stackPtr);
        cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
      }

      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat8& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; return; }
          else         { *stackPtr = c0; stackPtr++; cur = c1; return; }
        }
        assert(stackPtr < stackEnd);
        *stackPtr = c0; stackPtr++;
        assert(stackPtr < stackEnd);
        *stackPtr = c1; stackPtr++;

        /*! three children are hit */
        r = __bscf(mask);
        cur = node->child(r); cur.prefetch(types);
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        /*! fallback case if more than 3 children are hit */
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          NodeRef c = node->child(r); c.prefetch(types); *stackPtr = c; stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        cur = (NodeRef) stackPtr[-1]; stackPtr--;
      }
    };
  }
}
