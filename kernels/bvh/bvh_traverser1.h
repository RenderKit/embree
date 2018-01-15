// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
#include "node_intersector1.h"
#include "../common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH regular node traversal for single rays. */
    template<int N, int Nx, int types>
    class BVHNNodeTraverser1Hit;

    /*! Helper functions for fast sorting using AVX512 instructions. */
#if defined(__AVX512F__)   

    /* KNL code path */
    __forceinline static void isort_update(vfloat16 &dist, vllong8 &ptr, const vfloat16 &d, const vllong8 &p)
    {
      const vfloat16 dist_shift = align_shift_right<15>(dist,dist);
      const vllong8  ptr_shift  = align_shift_right<7>(ptr,ptr);
      const vbool16 m_geq = d >= dist;
      const vbool16 m_geq_shift = m_geq << 1;
      dist = select(m_geq,d,dist);
      ptr  = select(vboold8(m_geq),p,ptr);
      dist = select(m_geq_shift,dist_shift,dist);
      ptr  = select(vboold8(m_geq_shift),ptr_shift,ptr);
    }

    __forceinline static void isort_quick_update(vfloat16 &dist, vllong8 &ptr, const vfloat16 &d, const vllong8 &p)
    {
      //dist = align_shift_right<15>(dist,d);
      //ptr  = align_shift_right<7>(ptr,p);
      dist = align_shift_right<15>(dist,permute(d,vint16(zero)));
      ptr  = align_shift_right<7>(ptr,permute(p,vllong8(zero)));
    }

    template<int N, int Nx, int types, class NodeRef, class BaseNode>
    static __forceinline void traverseClosestHitAVX512(NodeRef& cur,
                                                       size_t mask,
                                                       const vfloat<Nx>& tNear,
                                                       StackItemT<NodeRef>*& stackPtr,
                                                       StackItemT<NodeRef>* stackEnd)
    {
      assert(mask != 0);
      const BaseNode* node = cur.baseNode(types);

      vllong8 children( vllong<N>::loadu((void*)node->children) );
      children = vllong8::compact((int)mask,children);
      vfloat16 distance = tNear;
      distance = vfloat16::compact((int)mask,distance,tNear);

      cur = toScalar(children);
      cur.prefetch(types);


      mask &= mask-1;
      if (likely(mask == 0)) return;

      /* 2 hits: order A0 B0 */
      const vllong8 c0(children);
      const vfloat16 d0(distance);
      children = align_shift_right<1>(children,children);
      distance = align_shift_right<1>(distance,distance);
      const vllong8 c1(children);
      const vfloat16 d1(distance);

      cur = toScalar(children);
      cur.prefetch(types);

      /* a '<' keeps the order for equal distances, scenes like powerplant largely benefit from it */
      const vboolf16 m_dist  = d0 < d1;
      const vfloat16 dist_A0 = select(m_dist, d0, d1);
      const vfloat16 dist_B0 = select(m_dist, d1, d0);
      const vllong8 ptr_A0   = select(vboold8(m_dist), c0, c1);
      const vllong8 ptr_B0   = select(vboold8(m_dist), c1, c0);

      mask &= mask-1;
      if (likely(mask == 0)) {
        cur = toScalar(ptr_A0);
        stackPtr[0].ptr            = toScalar(ptr_B0);
        *(float*)&stackPtr[0].dist = toScalar(dist_B0);
        stackPtr++;
        return;
      }

      /* 3 hits: order A1 B1 C1 */

      children = align_shift_right<1>(children,children);
      distance = align_shift_right<1>(distance,distance);

      const vllong8 c2(children);
      const vfloat16 d2(distance);

      cur = toScalar(children);
      cur.prefetch(types);

      const vboolf16 m_dist1     = dist_A0 <= d2;
      const vfloat16 dist_tmp_B1 = select(m_dist1, d2, dist_A0);
      const vllong8  ptr_A1      = select(vboold8(m_dist1), ptr_A0, c2);
      const vllong8  ptr_tmp_B1  = select(vboold8(m_dist1), c2, ptr_A0);

      const vboolf16 m_dist2     = dist_B0 <= dist_tmp_B1;
      const vfloat16 dist_B1     = select(m_dist2, dist_B0 , dist_tmp_B1);
      const vfloat16 dist_C1     = select(m_dist2, dist_tmp_B1, dist_B0);
      const vllong8  ptr_B1      = select(vboold8(m_dist2), ptr_B0, ptr_tmp_B1);
      const vllong8  ptr_C1      = select(vboold8(m_dist2), ptr_tmp_B1, ptr_B0);

      mask &= mask-1;
      if (likely(mask == 0)) {
        cur = toScalar(ptr_A1);
        stackPtr[0].ptr  = toScalar(ptr_C1);
        *(float*)&stackPtr[0].dist = toScalar(dist_C1);
        stackPtr[1].ptr  = toScalar(ptr_B1);
        *(float*)&stackPtr[1].dist = toScalar(dist_B1);
        stackPtr+=2;
        return;
      }

      /* 4 hits: order A2 B2 C2 D2 */

      const vfloat16 dist_A1  = select(m_dist1, dist_A0, d2);

      children = align_shift_right<1>(children,children);
      distance = align_shift_right<1>(distance,distance);

      const vllong8 c3(children);
      const vfloat16 d3(distance);

      cur = toScalar(children);
      cur.prefetch(types);

      const vboolf16 m_dist3     = dist_A1 <= d3;
      const vfloat16 dist_tmp_B2 = select(m_dist3, d3, dist_A1);
      const vllong8  ptr_A2      = select(vboold8(m_dist3), ptr_A1, c3);
      const vllong8  ptr_tmp_B2  = select(vboold8(m_dist3), c3, ptr_A1);

      const vboolf16 m_dist4     = dist_B1 <= dist_tmp_B2;
      const vfloat16 dist_B2     = select(m_dist4, dist_B1 , dist_tmp_B2);
      const vfloat16 dist_tmp_C2 = select(m_dist4, dist_tmp_B2, dist_B1);
      const vllong8  ptr_B2      = select(vboold8(m_dist4), ptr_B1, ptr_tmp_B2);
      const vllong8  ptr_tmp_C2  = select(vboold8(m_dist4), ptr_tmp_B2, ptr_B1);

      const vboolf16 m_dist5     = dist_C1 <= dist_tmp_C2;
      const vfloat16 dist_C2     = select(m_dist5, dist_C1 , dist_tmp_C2);
      const vfloat16 dist_D2     = select(m_dist5, dist_tmp_C2, dist_C1);
      const vllong8  ptr_C2      = select(vboold8(m_dist5), ptr_C1, ptr_tmp_C2);
      const vllong8  ptr_D2      = select(vboold8(m_dist5), ptr_tmp_C2, ptr_C1);

      mask &= mask-1;
      if (likely(mask == 0)) {
        cur = toScalar(ptr_A2);
        stackPtr[0].ptr  = toScalar(ptr_D2);
        *(float*)&stackPtr[0].dist = toScalar(dist_D2);
        stackPtr[1].ptr  = toScalar(ptr_C2);
        *(float*)&stackPtr[1].dist = toScalar(dist_C2);
        stackPtr[2].ptr  = toScalar(ptr_B2);
        *(float*)&stackPtr[2].dist = toScalar(dist_B2);
        stackPtr+=3;
        return;
      }

      /* >=5 hits: reverse to descending order for writing to stack */

      const size_t hits = 4 + __popcnt(mask);
      const vfloat16 dist_A2  = select(m_dist3, dist_A1, d3);
      vfloat16 dist(neg_inf);
      vllong8 ptr(zero);


      isort_quick_update(dist,ptr,dist_A2,ptr_A2);
      isort_quick_update(dist,ptr,dist_B2,ptr_B2);
      isort_quick_update(dist,ptr,dist_C2,ptr_C2);
      isort_quick_update(dist,ptr,dist_D2,ptr_D2);

      do {

        children = align_shift_right<1>(children,children);
        distance = align_shift_right<1>(distance,distance);

        cur = toScalar(children);
        cur.prefetch(types);

        const vfloat16 new_dist(permute(distance,vint16(zero)));
        const vllong8 new_ptr(permute(children,vllong8(zero)));

        mask &= mask-1;
        isort_update(dist,ptr,new_dist,new_ptr);

      } while(mask);

      const vboold8 m_stack_ptr(0x55);  // 10101010 (lsb -> msb)
      const vboolf16 m_stack_dist(0x4444); // 0010001000100010 (lsb -> msb)

      /* extract current noderef */
      cur = toScalar(permute(ptr,vllong8(hits-1)));
      /* rearrange pointers to beginning of 16 bytes block */
      vllong8 stackElementA0;
      stackElementA0 = vllong8::expand(m_stack_ptr,ptr,stackElementA0);
      /* put distances in between */
      vuint16 stackElementA1((__m512i)stackElementA0);
      stackElementA1 = vuint16::expand(m_stack_dist,asUInt(dist),stackElementA1);
      /* write out first 4 x 16 bytes block to stack */
      vuint16::storeu(stackPtr,stackElementA1);
      /* get upper half of dist and ptr */
      dist = align_shift_right<4>(dist,dist);
      ptr  = align_shift_right<4>(ptr,ptr);
      /* assemble and write out second block */
      vllong8 stackElementB0;
      stackElementB0 = vllong8::expand(m_stack_ptr,ptr,stackElementB0);
      vuint16 stackElementB1((__m512i)stackElementB0);
      stackElementB1 = vuint16::expand(m_stack_dist,asUInt(dist),stackElementB1);
      vuint16::storeu(stackPtr + 4,stackElementB1);
      /* increase stack pointer */
      stackPtr += hits-1;
    }


    /* SKX code path */
#if defined(__AVX512VL__)

    template<int N>
    __forceinline static void isort_update(vfloat<N> &dist, vint<N> &ptr, const vfloat<N> &d, const vint<N> &p)
    {
      const vfloat<N> dist_shift = align_shift_right<N-1>(dist,dist);
      const vint<N>  ptr_shift  = align_shift_right<N-1>(ptr,ptr);
      const vboolf<N> m_geq = d >= dist;
      const vboolf<N> m_geq_shift = m_geq << 1;
      dist = select(m_geq,d,dist);
      ptr  = select(m_geq,p,ptr);
      dist = select(m_geq_shift,dist_shift,dist);
      ptr  = select(m_geq_shift,ptr_shift,ptr);
    }

    template<int N>
    __forceinline static void isort_quick_update(vfloat<N> &dist, vint<N> &ptr, const vfloat<N> &d, const vint<N> &p)
    {
      dist = align_shift_right<N-1>(dist,permute(d,vint<N>(zero)));
      ptr  = align_shift_right<N-1>(ptr,permute(p,vint<N>(zero)));
    }

    template<int N, int Nx, int types, class NodeRef, class BaseNode>
    static __forceinline void traverseClosestHitAVX512VL(NodeRef& cur,
                                                         size_t mask,
                                                         const vfloat<Nx>& tNear,
                                                         StackItemT<NodeRef>*& stackPtr,
                                                         StackItemT<NodeRef>* stackEnd)
    {
      assert(mask != 0);
      const BaseNode* node = cur.baseNode(types);
      vint<N> children( step );
      children = vint<N>::compact((int)mask,children);
      vfloat<N> distance = tNear;
      distance = vfloat<N>::compact((int)mask,distance,tNear);

      cur = node->child((unsigned int)toScalar(children));
      cur.prefetch(types);


      mask &= mask-1;
      if (likely(mask == 0)) return;

      /* 2 hits: order A0 B0 */
      const vint<N> c0(children);
      const vfloat<N> d0(distance);
      children = align_shift_right<1>(children,children);
      distance = align_shift_right<1>(distance,distance);
      const vint<N> c1(children);
      const vfloat<N> d1(distance);

      cur = node->child((unsigned int)toScalar(children));
      cur.prefetch(types);

      /* a '<' keeps the order for equal distances, scenes like powerplant largely benefit from it */
      const vboolf<N> m_dist  = d0 < d1;
      const vfloat<N> dist_A0 = select(m_dist, d0, d1);
      const vfloat<N> dist_B0 = select(m_dist, d1, d0);
      const vint<N> ptr_A0   = select(vboolf<N>(m_dist), c0, c1);
      const vint<N> ptr_B0   = select(vboolf<N>(m_dist), c1, c0);

      mask &= mask-1;
      if (likely(mask == 0)) {
        cur = node->child((unsigned int)toScalar(ptr_A0));
        stackPtr[0].ptr            = node->child((unsigned int)toScalar(ptr_B0));
        *(float*)&stackPtr[0].dist = toScalar(dist_B0);
        stackPtr++;
        return;
      }

      /* 3 hits: order A1 B1 C1 */

      children = align_shift_right<1>(children,children);
      distance = align_shift_right<1>(distance,distance);

      const vint<N> c2(children);
      const vfloat<N> d2(distance);

      cur = node->child((unsigned int)toScalar(children));
      cur.prefetch(types);

      const vboolf<N> m_dist1     = dist_A0 <= d2;
      const vfloat<N> dist_tmp_B1 = select(m_dist1, d2, dist_A0);
      const vint<N>  ptr_A1      = select(vboolf<N>(m_dist1), ptr_A0, c2);
      const vint<N>  ptr_tmp_B1  = select(vboolf<N>(m_dist1), c2, ptr_A0);

      const vboolf<N> m_dist2     = dist_B0 <= dist_tmp_B1;
      const vfloat<N> dist_B1     = select(m_dist2, dist_B0 , dist_tmp_B1);
      const vfloat<N> dist_C1     = select(m_dist2, dist_tmp_B1, dist_B0);
      const vint<N>  ptr_B1      = select(vboolf<N>(m_dist2), ptr_B0, ptr_tmp_B1);
      const vint<N>  ptr_C1      = select(vboolf<N>(m_dist2), ptr_tmp_B1, ptr_B0);

      mask &= mask-1;
      if (likely(mask == 0)) {
        cur = node->child((unsigned int)toScalar(ptr_A1));
        stackPtr[0].ptr  = node->child((unsigned int)toScalar(ptr_C1));
        *(float*)&stackPtr[0].dist = toScalar(dist_C1);
        stackPtr[1].ptr  = node->child((unsigned int)toScalar(ptr_B1));
        *(float*)&stackPtr[1].dist = toScalar(dist_B1);
        stackPtr+=2;
        return;
      }

      /* 4 hits: order A2 B2 C2 D2 */

      const vfloat<N> dist_A1  = select(m_dist1, dist_A0, d2);

      children = align_shift_right<1>(children,children);
      distance = align_shift_right<1>(distance,distance);

      const vint<N> c3(children);
      const vfloat<N> d3(distance);

      cur = node->child((unsigned int)toScalar(children));
      cur.prefetch(types);

      const vboolf<N> m_dist3     = dist_A1 <= d3;
      const vfloat<N> dist_tmp_B2 = select(m_dist3, d3, dist_A1);
      const vint<N>  ptr_A2      = select(vboolf<N>(m_dist3), ptr_A1, c3);
      const vint<N>  ptr_tmp_B2  = select(vboolf<N>(m_dist3), c3, ptr_A1);

      const vboolf<N> m_dist4     = dist_B1 <= dist_tmp_B2;
      const vfloat<N> dist_B2     = select(m_dist4, dist_B1 , dist_tmp_B2);
      const vfloat<N> dist_tmp_C2 = select(m_dist4, dist_tmp_B2, dist_B1);
      const vint<N>  ptr_B2      = select(vboolf<N>(m_dist4), ptr_B1, ptr_tmp_B2);
      const vint<N>  ptr_tmp_C2  = select(vboolf<N>(m_dist4), ptr_tmp_B2, ptr_B1);

      const vboolf<N> m_dist5     = dist_C1 <= dist_tmp_C2;
      const vfloat<N> dist_C2     = select(m_dist5, dist_C1 , dist_tmp_C2);
      const vfloat<N> dist_D2     = select(m_dist5, dist_tmp_C2, dist_C1);
      const vint<N>  ptr_C2      = select(vboolf<N>(m_dist5), ptr_C1, ptr_tmp_C2);
      const vint<N>  ptr_D2      = select(vboolf<N>(m_dist5), ptr_tmp_C2, ptr_C1);

      mask &= mask-1;
      if (likely(mask == 0)) {
        cur = node->child((unsigned int)toScalar(ptr_A2));
        stackPtr[0].ptr  = node->child((unsigned int)toScalar(ptr_D2));
        *(float*)&stackPtr[0].dist = toScalar(dist_D2);
        stackPtr[1].ptr  = node->child((unsigned int)toScalar(ptr_C2));
        *(float*)&stackPtr[1].dist = toScalar(dist_C2);
        stackPtr[2].ptr  = node->child((unsigned int)toScalar(ptr_B2));
        *(float*)&stackPtr[2].dist = toScalar(dist_B2);
        stackPtr+=3;
        return;
      }

      /* >=5 hits: reverse to descending order for writing to stack */

      const size_t hits = 4 + __popcnt(mask);
      const vfloat<N> dist_A2  = select(m_dist3, dist_A1, d3);
      vfloat<N> dist(neg_inf);
      vint<N> ptr(zero);


      isort_quick_update(dist,ptr,dist_A2,ptr_A2);
      isort_quick_update(dist,ptr,dist_B2,ptr_B2);
      isort_quick_update(dist,ptr,dist_C2,ptr_C2);
      isort_quick_update(dist,ptr,dist_D2,ptr_D2);

      do {

        children = align_shift_right<1>(children,children);
        distance = align_shift_right<1>(distance,distance);

        cur = node->child((unsigned int)toScalar(children));
        cur.prefetch(types);

        const vfloat<N> new_dist(permute(distance,vint<N>(zero)));
        const vint<N> new_ptr(permute(children,vint<N>(zero)));

        mask &= mask-1;
        isort_update(dist,ptr,new_dist,new_ptr);

      } while(mask);

      for (size_t i=0;i<hits-1;i++)
      {
        stackPtr->ptr  = node->child((unsigned int)toScalar(ptr));
        *(float*)&stackPtr->dist = toScalar(dist);
        dist = align_shift_right<1>(dist,dist);
        ptr  = align_shift_right<1>(ptr,ptr);
        stackPtr++;
      }
      cur = node->child((unsigned int)toScalar(ptr));
    }
#endif

#endif

    /* Specialization for BVH4. */
    template<int Nx, int types>
    class BVHNNodeTraverser1Hit<4, Nx, types>
    {
      typedef BVH4 BVH;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::BaseNode BaseNode;

    public:
      /* Traverses a node with at least one hit child. Optimized for finding the closest hit (intersection). */
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat<Nx>& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
#if defined(__AVX512F__) && 1

#if defined(__AVX512ER__)
        traverseClosestHitAVX512<4,Nx,types,NodeRef,BaseNode>(cur,mask,tNear,stackPtr,stackEnd);
#elif defined(__AVX512VL__)
        traverseClosestHitAVX512VL<4,Nx,types,NodeRef,BaseNode>(cur,mask,tNear,stackPtr,stackEnd);
#else
        static_assert(false);
#endif

#else
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur;
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
#endif
      }

      /* Traverses a node with at least one hit child. Optimized for finding any hit (occlusion). */
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat<Nx>& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r); 
        cur.prefetch(types);

        /* simpler in sequence traversal order */
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        for (; ;)
        {
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return;
          assert(stackPtr < stackEnd);
          *stackPtr = cur; stackPtr++;
        }
      }
    };

    /* Specialization for BVH8. */
    template<int Nx, int types>
    class BVHNNodeTraverser1Hit<8, Nx, types>
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat<Nx>& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
#if defined(__AVX512F__) && 1

#if defined(__AVX512ER__)
        traverseClosestHitAVX512<8,Nx,types,NodeRef,BaseNode>(cur,mask,tNear,stackPtr,stackEnd);
#elif defined(__AVX512VL__)
        traverseClosestHitAVX512VL<8,Nx,types,NodeRef,BaseNode>(cur,mask,tNear,stackPtr,stackEnd);
#else
        static_assert(false);
#endif

#else
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur;
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

      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat<Nx>& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);

        /* simpler in sequence traversal order */
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        for (; ;)
        {
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return;
          assert(stackPtr < stackEnd);
          *stackPtr = cur; stackPtr++;
        }
      }
    };


    /*! BVH transform node traversal for single rays. */
    template<int N, int Nx, bool robust, int types, bool transform>
    class BVHNNodeTraverser1Transform;

#define ENABLE_TRANSFORM_CACHE 0

    template<int N, int Nx, bool robust, int types>
      class BVHNNodeTraverser1Transform<N, Nx, robust, types, true>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::TransformNode TransformNode;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRayBase<N,Nx,robust>& tray)
#if ENABLE_TRANSFORM_CACHE
        : cacheSlot(0), cacheTag(-1)
#endif
      {
        new (&topRay) TravRayBase<N,Nx,robust>(tray);
      }

      /* If a transform node is passed, traverses the node and returns true. */
      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRayBase<N,Nx,robust>& tray,
                                           IntersectContext* context,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(normal.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(EMBREE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif          
          //context->geomID_to_instID = &node->instID;
          context->instID = ray.instID;
          context->geomID = ray.geomID;
          ray.instID = node->instID;
          ray.geomID = -1;

#if ENABLE_TRANSFORM_CACHE
          const vboolx xfm_hit = cacheTag == vintx(node->xfmID);
          if (likely(any(xfm_hit))) {
            const int slot = __bsf(movemask(xfm_hit));
            tray = cacheEntry[slot];
            ray.org = tray.org_xyz;
            ray.dir = tray.dir_xyz;
          } 
          else 
#endif
            //if (likely(!node->identity)) 
          {
            const Vec3fa ray_org = xfmPoint (node->world2local, ((TravRayBase<N,Nx,robust>&)topRay).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local, ((TravRayBase<N,Nx,robust>&)topRay).dir_xyz);
            new (&tray) TravRayBase<N,Nx,robust>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = tray;
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
          //context->geomID_to_instID = nullptr;
          tray = (TravRayBase<N,Nx,robust>&)topRay;
          ray.org = ((TravRayBase<N,Nx,robust>&)topRay).org_xyz;
          ray.dir = ((TravRayBase<N,Nx,robust>&)topRay).dir_xyz;
          if (ray.geomID == -1) {
            ray.instID = context->instID;
            ray.geomID = context->geomID;
          }
          return true;
        }

        return false;
      }

      /* If a transform node is passed, traverses the node and returns true. */
      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRayBase<N,Nx,robust>& tray,
                                           IntersectContext* context,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(shadow.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(EMBREE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif
          //context->geomID_to_instID = &node->instID;
          context->instID = ray.instID;
          context->geomID = ray.geomID;
          ray.instID = node->instID;
          ray.geomID = -1;

#if ENABLE_TRANSFORM_CACHE
          const vboolx xfm_hit = cacheTag == vintx(node->xfmID);
          if (likely(any(xfm_hit))) {
            const int slot = __bsf(movemask(xfm_hit));
            tray = cacheEntry[slot];
            ray.org = tray.org_xyz;
            ray.dir = tray.dir_xyz;
          } 
          else 
#endif
            //if (likely(!node->identity)) 
          {
            const Vec3fa ray_org = xfmPoint (node->world2local, ((TravRayBase<N,Nx,robust>&)topRay).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local, ((TravRayBase<N,Nx,robust>&)topRay).dir_xyz);
            new (&tray) TravRayBase<N,Nx,robust>(ray_org, ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = tray;
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
          //context->geomID_to_instID = nullptr;
          tray = (TravRayBase<N,Nx,robust>&)topRay;
          ray.org = ((TravRayBase<N,Nx,robust>&)topRay).org_xyz;
          ray.dir = ((TravRayBase<N,Nx,robust>&)topRay).dir_xyz;
          if (ray.geomID == -1) {
            ray.instID = context->instID;
            ray.geomID = context->geomID;
          }
          return true;
        }

        return false;
      }

    private:
      TravRayBase<N,Nx,robust> topRay;

#if ENABLE_TRANSFORM_CACHE
    private:
      unsigned int cacheSlot;
      vintx cacheTag;
      TravRayBase<N,Nx,robust> cacheEntry[VSIZEX];
#endif
    };

    template<int N, int Nx, bool robust, int types>
      class BVHNNodeTraverser1Transform<N, Nx, robust, types, false>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRayBase<N,Nx,robust>& tray) {}

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRayBase<N,Nx,robust>& tray,
                                           IntersectContext* context,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        return false;
      }

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRayBase<N,Nx,robust>& tray,
                                           IntersectContext* context,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        return false;
      }
    };

    /*! BVH node traversal for single rays. */
    template<int N, int Nx, bool robust, int types>
      class BVHNNodeTraverser1 : public BVHNNodeTraverser1Hit<N, Nx, types>, public BVHNNodeTraverser1Transform<N, Nx, robust, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>
    {
    public:
      __forceinline explicit BVHNNodeTraverser1(const TravRayBase<N,Nx,robust>& tray) : BVHNNodeTraverser1Transform<N, Nx, robust, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>(tray) {}
    };
  }
}
