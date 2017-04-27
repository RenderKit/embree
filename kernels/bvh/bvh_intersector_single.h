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

#include "bvh.h"
#include "bvh_intersector_node.h"
#include "bvh_traverser1.h"
#include "../common/ray.h"

namespace embree
{
  namespace isa 
  {
    /*! BVH single ray intersector for packets. */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK>
    class BVHNIntersectorKSingle
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersectorK::Precalculations Precalculations;
      typedef typename PrimitiveIntersectorK::Primitive Primitive;
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;
      typedef typename BVH::AlignedNode AlignedNode;
      typedef Vec3<vfloat<N>> Vec3vfN;
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef Vec3<vint<K>> Vec3viK;

      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

      /* right now AVX512KNL SIMD extension only for standard node types */
      static const size_t Nx = types == BVH_AN1 ? vextend<N>::size : N;

    public:

#if defined(__AVX512F__)
      __forceinline static void isort_update(vfloat16 &dist, vllong8 &ptr, const vfloat16 &d, const vllong8 &p)
      {
        const vfloat16 dist_shift = align_shift_right<15>(dist,dist);
        const vllong8  ptr_shift  = align_shift_right<7>(ptr,ptr);
        const vbool16 m_leq = d <= dist;
        const vbool16 m_leq_shift = m_leq << 1;
        dist = select(m_leq,d,dist);
        ptr  = select(vboold8(m_leq),p,ptr);
        dist = select(m_leq_shift,dist_shift,dist);
        ptr  = select(vboold8(m_leq_shift),ptr_shift,ptr);
      }
#endif

      static __forceinline void traverseClosestHitNew(const BaseNode* node,
                                                      NodeRef& cur,
                                                      size_t mask,
                                                      const vfloat<Nx>& tNear,
                                                      StackItemT<NodeRef>*& stackPtr,
                                                      StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        //const BaseNode* node = cur.baseNode(types);
#if defined(__AVX512F__)
        /*! one child is hit, continue with that child */
        const size_t hits = __popcnt(mask);

        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        const vllong8 c0(cur);
        const vfloat16 d0(tNear[r]);
        r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        const vllong8 c1(cur);
        const vfloat16 d1(tNear[r]);

        const vboolf16 m_dist = d0 <= d1;
        const vfloat16 d_near = select(m_dist, d0, d1);
        const vfloat16 d_far  = select(m_dist, d1, d0);
        const vllong8 c_near  = select(vboold8(m_dist), c0, c1);
        const vllong8 c_far   = select(vboold8(m_dist), c1, c0);

        if (likely(mask == 0)) {
          cur = toScalar(c_near);
          *(float*)&stackPtr[0].dist = toScalar(d_far);
          stackPtr[0].ptr  = toScalar(c_far);
          stackPtr++;
          return;
        }


        r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        const vllong8 c2(cur);
        const vfloat16 d2(tNear[r]);

        const vboolf16 m_dist1 = d_near <= d2;
        const vfloat16 d_near1  = select(m_dist1, d_near, d2);
        const vfloat16 d_far1   = select(m_dist1, d2, d_near);
        const vllong8  c_near1  = select(vboold8(m_dist1), c_near, c2);
        const vllong8  c_far1   = select(vboold8(m_dist1), c2, c_near);

        const vboolf16 m_dist2 = d_far <= d_far1;
        const vfloat16 d_near2  = select(m_dist2, d_far , d_far1);
        const vfloat16 d_far2   = select(m_dist2, d_far1, d_far);
        const vllong8  c_near2  = select(vboold8(m_dist2), c_far, c_far1);
        const vllong8  c_far2   = select(vboold8(m_dist2), c_far1, c_far);

        if (likely(mask == 0)) {

          cur = toScalar(c_near1);
          *(float*)&stackPtr[0].dist = toScalar(d_far2);
          stackPtr[0].ptr  = toScalar(c_far2);
          *(float*)&stackPtr[1].dist = toScalar(d_near2);
          stackPtr[1].ptr  = toScalar(c_near2);
          stackPtr+=2;
          return;
        }

        vfloat16 dist(pos_inf);
        vllong8 ptr(zero);

        isort_update(dist,ptr,d0,c0);
        isort_update(dist,ptr,d1,c1);
        isort_update(dist,ptr,d2,c2);

        do {
          const size_t r = __bscf(mask);
          cur = node->child(r);
          const vfloat16 new_dist(tNear[r]);
          const vllong8 new_ptr(cur);
          cur.prefetch(types);
          isort_update(dist,ptr,new_dist,new_ptr);
        } while(mask);

        cur = toScalar(ptr);
        stackPtr += hits - 1;
        for (size_t i=0;i<hits-1;i++)
        {
          dist = align_shift_right<1>(dist,dist);
          ptr  = align_shift_right<1>(ptr,ptr);          
          stackPtr[-1-i].ptr  = toScalar(ptr);
          *(float*)&stackPtr[-1-i].dist = toScalar(dist);
        }
#else
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
        stackPtr->dist = d; 
        stackPtr++;
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
          stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        sort(stackFirst,stackPtr);
        cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
#endif
      }

      static void intersect1(const BVH* bvh, 
                             NodeRef root, 
                             const size_t k, 
                             Precalculations& pre,
                             RayK<K>& ray, 
                             const Vec3vfK &ray_org, 
                             const Vec3vfK &ray_dir, 
                             const Vec3vfK &ray_rdir, 
                             const vfloat<K> &ray_tnear, 
                             const vfloat<K> &ray_tfar,
                             const Vec3viK& nearXYZ, 
                             IntersectContext* context)
      {
	/*! stack state */
	StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
	StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
	StackItemT<NodeRef>* stackEnd = stack + stackSizeSingle;
	stack[0].ptr = root;
	stack[0].dist = neg_inf;
	
	/*! load the ray into SIMD registers */
        TravRay<N,Nx> vray(k,ray_org,ray_dir,ray_rdir,nearXYZ);
        vfloat<Nx> ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
	
	/* pop loop */
	while (true) pop:
	{
	  /*! pop next node */
	  if (unlikely(stackPtr == stack)) break;
	  stackPtr--;
	  NodeRef cur = NodeRef(stackPtr->ptr);
	  
	  /*! if popped node is too far, pop next one */
	  //if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k])) continue;
          if (unlikely((int)stackPtr->dist >= *(const int*)&ray.tfar[k])) continue;

          /* downtraversal loop */
          while (true)
          {
            //cur.prefetch(types);
            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
            STAT3(normal.trav_nodes,1,1,1);

            /* intersect node */
            size_t mask = 0;
            vfloat<Nx> tNear;
            BVHNNodeIntersector1<N,Nx,types,robust>::intersect(cur,vray,ray_near,ray_far,pre.ftime(k),tNear,mask);

            AlignedNode *node = cur.alignedNode(); 

#if 0 // defined(__AVX512F__)
            vllong8 c = vllong8::loadu(node->children);            
            c = vllong8::compact((__mmask16)mask,c);
            cur = vllong8::extract64bit(c);
            STAT3(normal.trav_hit_boxes[__popcnt(mask)],1,1,1);
#endif
            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;
            
            /* select next child and push other children */
#if defined(__AVX512F__)
            traverseClosestHitNew(node,cur,mask,tNear,stackPtr,stackEnd);
#else
            BVHNNodeTraverser1<N,Nx,types>::traverseClosestHit(cur,mask,tNear,stackPtr,stackEnd);
#endif
          }

	  /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
	  STAT3(normal.trav_leaves, 1, 1, 1);
	  size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          //const unsigned int old_primID = ray.primID[k];
          PrimitiveIntersectorK::intersect(pre, ray, k, context, prim, num, lazy_node);

          ray_far = ray.tfar[k];

#if 0
          if (unlikely(old_primID != ray.primID[k]))
          {
            const unsigned int ui_tfar = *(unsigned int*)&ray.tfar[k];
            /* stack compaction */
            StackItemT<NodeRef>* const end_stackPtr = stackPtr;
            stackPtr = stack;
            for (StackItemT<NodeRef>* s = stack; s != end_stackPtr; s++)
              if (unlikely((unsigned int)s->dist <= ui_tfar))
                *stackPtr++=*s;
          }

#endif

          /* if (unlikely(lazy_node)) { */
          /*   stackPtr->ptr = lazy_node; */
          /*   stackPtr->dist = neg_inf; */
          /*   stackPtr++; */
          /* } */
	}
      }
      
      static bool occluded1(const BVH* bvh, NodeRef root, const size_t k, Precalculations& pre,
                            RayK<K>& ray, const Vec3vfK &ray_org, const Vec3vfK &ray_dir, const Vec3vfK &ray_rdir, const vfloat<K> &ray_tnear, const vfloat<K> &ray_tfar,
                            const Vec3viK& nearXYZ, IntersectContext* context)
      {
	/*! stack state */
	NodeRef stack[stackSizeSingle];  //!< stack of nodes that still need to get traversed
        NodeRef* stackPtr = stack+1;     //!< current stack pointer
	NodeRef* stackEnd = stack+stackSizeSingle;
	stack[0]  = root;
      
	/*! load the ray into SIMD registers */
        TravRay<N,Nx> vray(k,ray_org,ray_dir,ray_rdir,nearXYZ);
        const vfloat<Nx> ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
	
	/* pop loop */
	while (true) pop:
	{
	  /*! pop next node */
	  if (unlikely(stackPtr == stack)) break;
	  stackPtr--;
	  NodeRef cur = (NodeRef) *stackPtr;
	  
          /* downtraversal loop */
          while (true)
          {
            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
            STAT3(shadow.trav_nodes,1,1,1);

            /* intersect node */
            size_t mask = 0;
            vfloat<Nx> tNear;
            BVHNNodeIntersector1<N,Nx,types,robust>::intersect(cur,vray,ray_near,ray_far,pre.ftime(k),tNear,mask);

            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;

            /* select next child and push other children */
            BVHNNodeTraverser1<N,Nx,types>::traverseAnyHit(cur,mask,tNear,stackPtr,stackEnd);
          }

	  /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
	  STAT3(shadow.trav_leaves,1,1,1);
	  size_t num; Primitive* prim = (Primitive*) cur.leaf(num);

          size_t lazy_node = 0;
          if (PrimitiveIntersectorK::occluded(pre,ray,k,context,prim,num,lazy_node)) {
	    ray.geomID[k] = 0;
	    return true;
	  }

          if (unlikely(lazy_node)) {
            *stackPtr = lazy_node;
            stackPtr++;
          }
	}
	return false;
      }
      
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray, IntersectContext* context);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray, IntersectContext* context);
    };
  }
}
