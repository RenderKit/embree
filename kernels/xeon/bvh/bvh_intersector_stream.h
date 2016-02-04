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

#if defined(__AVX__)
    template<int types, int K>
      class BVHNNodeTraverserKHit
    {
      typedef BVHN<K> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;


    public:


      // FIXME: optimize sequence
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   unsigned int &m_trav_active,
                                                   size_t mask,
                                                   const vfloat<K>& tNear,
                                                   const vint<K>& tMask,
                                                   StackItemMaskT<NodeRef>*& stackPtr,
                                                   StackItemMaskT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
#if 0
        vint<K> _tMask = tMask;
        const vbool<K> vmask = vbool<K>((unsigned int)mask);
        vint<K> cmask = vint<K>::compact(vmask,_tMask);
        m_trav_active = toScalar(cmask);
#endif
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);         
        cur.prefetch(types);
        m_trav_active = tMask[r]; //firstMask;
        assert(cur != BVH::emptyNode);
        if (unlikely(mask == 0)) return;

        unsigned int *active = (unsigned int*)&tMask;

        vint<K> dist = (asInt(tNear) & 0xfffffff8) | vint<K>( step );

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur; 
        const unsigned int d0 = dist[r];
        const int m0 = active[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); 
        c1.prefetch(types); 
        const unsigned int d1 = dist[r];
        const int m1 = active[r];

        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr->dist = d1; stackPtr++; cur = c0; m_trav_active = m0; return; }
          else         { stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr->dist = d0; stackPtr++; cur = c1; m_trav_active = m1; return; }
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
        unsigned int d = dist[r]; 
        unsigned int m = active[r];
        stackPtr->ptr = c; stackPtr->mask = m; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; 
          unsigned int mm = stackPtr[-1].mask;
          stackPtr--;
          m_trav_active = mm;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); 
        c.prefetch(types); 
        m = active[r];
        d = dist[r]; 
        stackPtr->ptr = c; stackPtr->mask = m; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; 
          unsigned int mm = stackPtr[-1].mask;
          stackPtr--;
          m_trav_active = mm;
          return;
        }

        /*! fallback case if more than 4 children are hit */
        StackItemMaskT<NodeRef>* stackFirst = stackPtr-4;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          c = node->child(r); 
          c.prefetch(types); 
          d = dist[r]; 
          m = active[r];
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
        m_trav_active = mm;
      }




      static __forceinline int traverseAnyHit(NodeRef& cur,
                                              size_t mask,
                                              const vfloat<K>& tNear,
                                              const vint<K>& tMask,
                                              StackItemMaskT<NodeRef>*& stackPtr,
                                              StackItemMaskT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);         
        cur.prefetch(types);

        
        /* simple in order sequence */
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return tMask[r];
        assert(stackPtr < stackEnd);
        stackPtr->ptr  = cur;
        stackPtr->mask = tMask[r];
        stackPtr++;

        for (; ;)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          cur = node->child(r);
          cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return tMask[r];
          assert(stackPtr < stackEnd);
          stackPtr->ptr  = cur;
          stackPtr->mask = tMask[r];
          stackPtr++;
        }
      }

    };
#endif


    /*! BVH ray stream intersector. */
    template<int N, int types, bool robust, typename PrimitiveIntersector>
      class BVHNStreamIntersector
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector::Precalculations Precalculations;
      typedef typename PrimitiveIntersector::Primitive Primitive;
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;
      typedef typename BVH::Node Node;
      typedef typename BVH::NodeMB NodeMB;

      static const size_t stackSizeChunk  = N*BVH::maxDepth+1;
      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;
      static const size_t MAX_RAYS_PER_OCTANT = 32;
      static const size_t K = N;

      struct RayContext {
        Vec3fa rdir;      //     rdir.w = tnear;
        Vec3fa org_rdir;  // org_rdir.w = tfar;
      };

    public:
      static void intersect(BVH* bvh, Ray **ray, size_t numRays, size_t flags);
      static void occluded (BVH* bvh, Ray **ray, size_t numRays, size_t flags);
    };



  }
}
