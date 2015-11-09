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
#include "bvh_intersector_node.h"
#include "../../common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH regular node traversal for single rays. */
    template<int N, int Nx, int types>
    class BVHNNodeTraverser1Hit;

    /* Specialization for BVH4. */
    template<int types>
      class BVHNNodeTraverser1Hit<4,4,types>
    {
      typedef BVH4 BVH;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::BaseNode BaseNode;

    public:
      /* Traverses a node with at least one hit child. Optimized for finding the closest hit (intersection). */
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat4& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
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

      /* Traverses a node with at least one hit child. Optimized for finding any hit (occlusion). */
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat4& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        assert(mask != 0);
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



#if defined (__AVX512F__)

    // ====================================================================================
    // ====================================================================================
    // ====================================================================================

    /* Specialization for BVH4. */
    template<int types>
      class BVHNNodeTraverser1Hit<4,16,types>
    {
      typedef BVH4 BVH;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::BaseNode BaseNode;

    public:
      // FIXME: optimize sequence
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat16& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
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

      // FIXME: optimize sequence
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat16& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        assert(mask != 0);
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


    template<int types>
      class BVHNNodeTraverser1Hit<8,16,types>
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:

      // FIXME: optimize sequence
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat16& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);
        const vbool16 org_mask = (int)mask;
        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r); 
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        //NodeRef c0 = node->child(r); 
        //c0.prefetch(types); 
        NodeRef c0 = cur; // node->child(r); 
        //c0.prefetch(types); 
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
#if 0
        /* const vint16 tNear_i = (asInt(tNear) & (~7)) | vint16(step); */
        /* const vint16 dist    = select((vbool16)(int)org_mask,tNear_i,vint16(0xffffffff)); */
        const vbool16 vmask = (int)org_mask;
        const vint16 offsets = vint16::load((int*)node->children);
        gather_prefetch64((void*)(0*64),vmask,offsets);
        gather_prefetch64((void*)(1*64),vmask,offsets);
        gather_prefetch64((void*)(2*64),vmask,offsets);
        gather_prefetch64((void*)(3*64),vmask,offsets);

        const size_t hits    = __popcnt(org_mask);
        vint16 tNear_i = vint16(0xffffffff);
        const vint16 dist = vint16::compact((vbool16)(int)org_mask,(asInt(tNear) & (~7)) | vint16(step),tNear_i);

        
        /* if (likely(hits <=4)) */
        /* { */
        /* } */
        /* else */
        {
          const vint8 order = sortNetwork((__m256i)dist) & 7;
          const unsigned int cur_index = toScalar(order);
          cur = node->child(cur_index);
          //cur.prefetch();
          // FIXME: replace with scatter
          for (size_t i=0;i<hits-1;i++) 
          {
            r = order[hits-1-i];
            assert( ((unsigned int)1 << r) & org_mask);
            const NodeRef c = node->child(r); 
            assert(c != BVH8::emptyNode);
            //c.prefetch(); 
            const unsigned int d = *(unsigned int*)&tNear[r]; 
            stackPtr->ptr = c; 
            stackPtr->dist = d; 
            stackPtr++;            
          }
        }

#else
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
#endif
      }

      // FIXME: optimize sequence
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat16& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        assert(mask != 0);
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

    // ====================================================================================
    // ====================================================================================
    // ====================================================================================


#endif

    /* Specialization for BVH8. */
    template<int types>
      class BVHNNodeTraverser1Hit<8,8,types>
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   const BaseNode* node,
                                                   size_t mask,
                                                   const vfloat8& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        //const BaseNode* node = cur.baseNode(types);

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

      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat8& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);
        traverseClosestHit(cur,node,mask,tNear,stackPtr,stackEnd);
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

    /*! BVH transform node traversal for single rays. */
    template<int N, int Nx, int types, bool transform>
    class BVHNNodeTraverser1Transform;

#define ENABLE_TRANSFORM_CACHE 0

    template<int N, int Nx, int types>
      class BVHNNodeTraverser1Transform<N,Nx,types,true>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::TransformNode TransformNode;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRay<N,Nx>& vray)
#if ENABLE_TRANSFORM_CACHE
        : cacheSlot(0), cacheTag(-1)
#endif
      {
        new (&tlray) TravRay<N,Nx>(vray);
      }

      /* If a transform node is passed, traverses the node and returns true. */
      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(normal.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(RTCORE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif          
          leafType = node->type;
          geomID_to_instID = &node->instID;

#if ENABLE_TRANSFORM_CACHE
          const vboolx xfm_hit = cacheTag == vintx(node->xfmID);
          if (likely(any(xfm_hit))) {
            const int slot = __bsf(movemask(xfm_hit));
            vray = cacheEntry[slot];
            ray.org = vray.org_xyz;
            ray.dir = vray.dir_xyz;
          } 
          else 
#endif
            //if (likely(!node->identity)) 
          {
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N,Nx>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N,Nx>&)tlray).dir_xyz);  
            new (&vray) TravRay<N,Nx>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = vray;
            cacheSlot++;
#endif
          }
          stackPtr->ptr = BVH::popRay; stackPtr->dist = neg_inf; stackPtr++;
          stackPtr->ptr = node->child; stackPtr->dist = neg_inf; stackPtr++;
          return true;
        }

        /*! restore toplevel ray */
        if (cur == BVH::popRay)
        {
          leafType = 0;
          geomID_to_instID = nullptr;
          vray = (TravRay<N,Nx>&) tlray;
          ray.org = ((TravRay<N,Nx>&)tlray).org_xyz;
          ray.dir = ((TravRay<N,Nx>&)tlray).dir_xyz;
          return true;
        }

        return false;
      }

      /* If a transform node is passed, traverses the node and returns true. */
      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(shadow.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(RTCORE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif
          leafType = node->type;
          geomID_to_instID = &node->instID;

#if ENABLE_TRANSFORM_CACHE
          const vboolx xfm_hit = cacheTag == vintx(node->xfmID);
          if (likely(any(xfm_hit))) {
            const int slot = __bsf(movemask(xfm_hit));
            vray = cacheEntry[slot];
            ray.org = vray.org_xyz;
            ray.dir = vray.dir_xyz;
          } 
          else 
#endif
            //if (likely(!node->identity)) 
          {
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N,Nx>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N,Nx>&)tlray).dir_xyz);
            new (&vray) TravRay<N,Nx>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = vray;
            cacheSlot++;
#endif
          }
          *stackPtr = BVH::popRay; stackPtr++;
          *stackPtr = node->child; stackPtr++;
          return true;
        }

        /*! restore toplevel ray */
        if (cur == BVH::popRay)
        {
          leafType = 0;
          geomID_to_instID = nullptr;
          vray = (TravRay<N,Nx>&) tlray;
          ray.org = ((TravRay<N,Nx>&)tlray).org_xyz;
          ray.dir = ((TravRay<N,Nx>&)tlray).dir_xyz;
          return true;
        }

        return false;
      }

    private:
      TravRay<N,Nx> tlray;

#if ENABLE_TRANSFORM_CACHE
    private:
      unsigned int cacheSlot;
      vintx cacheTag;
      TravRay<N,Nx> cacheEntry[VSIZEX];
#endif
    };

    template<int N, int Nx, int types>
      class BVHNNodeTraverser1Transform<N,Nx,types,false>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRay<N,Nx>& vray) {}

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        return false;
      }

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        return false;
      }
    };

    /*! BVH node traversal for single rays. */
    template<int N, int Nx, int types>
      class BVHNNodeTraverser1 : public BVHNNodeTraverser1Hit<N, Nx, types>, public BVHNNodeTraverser1Transform<N, Nx, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>
    {
    public:
      __forceinline explicit BVHNNodeTraverser1(const TravRay<N,Nx>& vray) : BVHNNodeTraverser1Transform<N, Nx, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>(vray) {}
    };
  }
}
