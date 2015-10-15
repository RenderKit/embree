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

#include "../bvh/bvh.h"
#include "../bvh/bvh_intersector_node.h"

#include "../../common/ray.h"
#include "../../common/stack_item.h"

namespace embree
{
  namespace isa 
  {
    /*! Converts single ray traversal into packet traversal. */
    template<int N, int K, typename Intersector1>
    class BVHNIntersectorKFromIntersector1
    {
      typedef BVHN<N> BVH;

    public:
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
    };

    /*! Single ray traversal for packets. */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK>
    class BVHNIntersectorKSingle
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersectorK::Precalculations Precalculations;
      typedef typename PrimitiveIntersectorK::Primitive Primitive;
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::Node Node;
      typedef typename BVH::BaseNode BaseNode;
      typedef Vec3<vfloat<N>> Vec3vfN;
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef Vec3<vint<K>> Vec3viK;

      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

    public:

      static __forceinline void intersect1(const BVH* bvh, NodeRef root, const size_t k, Precalculations& pre,
                                           RayK<K>& ray, const Vec3vfK &ray_org, const Vec3vfK &ray_dir, const Vec3vfK &ray_rdir, const vfloat<K> &ray_tnear, const vfloat<K> &ray_tfar,
                                           const Vec3viK& nearXYZ)
      {
	/*! stack state */
	StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
	StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
	StackItemT<NodeRef>* stackEnd = stack + stackSizeSingle;
	stack[0].ptr = root;
	stack[0].dist = neg_inf;
	
	/*! load the ray into SIMD registers */
        TravRay<N> vray(k,ray_org,ray_dir,ray_rdir,nearXYZ);
        vfloat<N> ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
	
	/* pop loop */
	while (true) pop:
	{
	  /*! pop next node */
	  if (unlikely(stackPtr == stack)) break;
	  stackPtr--;
	  NodeRef cur = NodeRef(stackPtr->ptr);
	  
	  /*! if popped node is too far, pop next one */
	  if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
	    continue;
	  
	  /* downtraversal loop */
	  while (true)
	  {
            vbool<N> vmask;
            vfloat<N> tNear;
	    
	    /*! stop if we found a leaf node */
	    if (unlikely(cur.isLeaf(types))) break;
	    STAT3(normal.trav_nodes,1,1,1);
	    
	    /* process standard nodes */
	    if (likely(cur.isNode(types)))
              vmask = intersect_node<N,robust>(cur.node(),vray,ray_near,ray_far,tNear);
	    
	    /* process motion blur nodes */
	    else if (likely(cur.isNodeMB(types)))
              vmask = intersect_node<N>(cur.nodeMB(),vray,ray_near,ray_far,ray.time[k],tNear);
	    
	    /*! process nodes with unaligned bounds */
	    else if (unlikely(cur.isUnalignedNode(types)))
              vmask = intersect_node<N>(cur.unalignedNode(),vray,ray_near,ray_far,tNear);
	    
	    /*! process nodes with unaligned bounds and motion blur */
	    else if (unlikely(cur.isUnalignedNodeMB(types)))
              vmask = intersect_node<N>(cur.unalignedNodeMB(),vray,ray_near,ray_far,ray.time[k],tNear);
	    
            size_t mask = movemask(vmask);

	    /*! if no child is hit, pop next node */
            const BaseNode* node = cur.baseNode(types);
	    if (unlikely(mask == 0))
	      goto pop;
	    
	    /*! one child is hit, continue with that child */
	    size_t r = __bscf(mask);
	    if (likely(mask == 0)) {
	      cur = node->child(r); cur.prefetch(types);
              assert(cur != BVH::emptyNode);
	      continue;
	    }
	    
	    /*! two children are hit, push far child, and continue with closer child */
	    NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
	    r = __bscf(mask);
	    NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
            assert(c0 != BVH::emptyNode);
            assert(c1 != BVH::emptyNode);
	    if (likely(mask == 0)) {
	      assert(stackPtr < stackEnd); 
	      if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
	      else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
	    }

#if defined(__AVX2__)
            if (N == 8)
            {
              /*! use 8-wide sorting network */
              const size_t hits = __popcnt(movemask(vmask));
              const vint<N> tNear_i = asInt(tNear);
              const int orderMask = N-1;
              const vint<N> dist = select(vmask, (tNear_i & (~orderMask)) | vint<N>(step), vint<N>(True));
              const vint<N> order = sortNetwork(dist) & orderMask;
              const unsigned int cur_index = toScalar(order);
              cur = node->child(cur_index);
              cur.prefetch();

              for (size_t i=0; i<hits-1; i++)
              {
                r = order[hits-1-i];
                assert(((unsigned int)1 << r) & movemask(vmask));
                const NodeRef c = node->child(r);
                assert(c != BVH::emptyNode);
                c.prefetch();
                const unsigned int d = *(unsigned int*)&tNear[r];
                stackPtr->ptr = c;
                stackPtr->dist = d;
                stackPtr++;
              }
            }
            else
#endif
            {
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
                continue;
              }

              /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
              assert(stackPtr < stackEnd);
              r = __bscf(mask);
              c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
              assert(c != BVH::emptyNode);
              if (likely(N == 4 || mask == 0)) {
                sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
                cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
                continue;
              }

              /*! fallback case if more than 4 children are hit */
              while (1)
              {
                assert(stackPtr < stackEnd);
                r = __bscf(mask);
                c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
                assert(c != BVH::emptyNode);
                if (unlikely(mask == 0)) break;
              }
              cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            }
	  }
	  
	  /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
	  STAT3(normal.trav_leaves, 1, 1, 1);
	  size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          PrimitiveIntersectorK::intersect(pre, ray, k, prim, num, bvh->scene, lazy_node);
	  ray_far = ray.tfar[k];

          if (unlikely(lazy_node)) {
            stackPtr->ptr = lazy_node;
            stackPtr->dist = neg_inf;
            stackPtr++;
          }
	}
      }
      
      static __forceinline bool occluded1(const BVH* bvh, NodeRef root, const size_t k, Precalculations& pre,
                                          RayK<K>& ray, const Vec3vfK &ray_org, const Vec3vfK &ray_dir, const Vec3vfK &ray_rdir, const vfloat<K> &ray_tnear, const vfloat<K> &ray_tfar,
                                          const Vec3viK& nearXYZ)
      {
	/*! stack state */
	NodeRef stack[stackSizeSingle];  //!< stack of nodes that still need to get traversed
	NodeRef* stackPtr = stack+1;        //!< current stack pointer
	NodeRef* stackEnd = stack+stackSizeSingle;
	stack[0]  = root;
      
	/*! load the ray into SIMD registers */
        TravRay<N> vray(k,ray_org,ray_dir,ray_rdir,nearXYZ);
        const vfloat<N> ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
	
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
            vbool<N> vmask;
            vfloat<N> tNear;
	    
	    /*! stop if we found a leaf node */
	    if (unlikely(cur.isLeaf(types))) break;
	    STAT3(shadow.trav_nodes,1,1,1);
	    
	    /* process standard nodes */
	    if (likely(cur.isNode(types)))
              vmask = intersect_node<N,robust>(cur.node(),vray,ray_near,ray_far,tNear);
	    
	    /* process motion blur nodes */
	    else if (likely(cur.isNodeMB(types)))
              vmask = intersect_node<N>(cur.nodeMB(),vray,ray_near,ray_far,ray.time[k],tNear);

	    /*! process nodes with unaligned bounds */
	    else if (unlikely(cur.isUnalignedNode(types)))
              vmask = intersect_node<N>(cur.unalignedNode(),vray,ray_near,ray_far,tNear);
	    
	    /*! process nodes with unaligned bounds and motion blur */
	    else if (unlikely(cur.isUnalignedNodeMB(types)))
              vmask = intersect_node<N>(cur.unalignedNodeMB(),vray,ray_near,ray_far,ray.time[k],tNear);
	    
            size_t mask = movemask(vmask);

	    /*! if no child is hit, pop next node */
            const BaseNode* node = cur.baseNode(types);
	    if (unlikely(mask == 0))
	      goto pop;
	  
	    /*! one child is hit, continue with that child */
	    size_t r = __bscf(mask);
	    if (likely(mask == 0)) {
	      cur = node->child(r); cur.prefetch(types); 
              assert(cur != BVH::emptyNode);
	      continue;
	    }
	    
	    /*! two children are hit, push far child, and continue with closer child */
	    NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
	    r = __bscf(mask);
	    NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
            assert(c0 != BVH::emptyNode);
            assert(c1 != BVH::emptyNode);
	    if (likely(mask == 0)) {
	      assert(stackPtr < stackEnd);
	      if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; continue; }
	      else         { *stackPtr = c0; stackPtr++; cur = c1; continue; }
	    }
	    assert(stackPtr < stackEnd);
	    *stackPtr = c0; stackPtr++;
	    assert(stackPtr < stackEnd);
	    *stackPtr = c1; stackPtr++;
	    
	    /*! three children are hit */
	    r = __bscf(mask);
	    cur = node->child(r); cur.prefetch(types);
            assert(cur != BVH::emptyNode);
	    if (likely(mask == 0)) continue;
	    assert(stackPtr < stackEnd);
	    *stackPtr = cur; stackPtr++;
	    
            /*! four or more children are hit */
            if (N == 4)
            {
              /*! four children are hit */
              cur = node->child(3); cur.prefetch(types);
              assert(cur != BVH::emptyNode);
            }
            else
            {
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
	  }
	
	  /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
	  STAT3(shadow.trav_leaves,1,1,1);
	  size_t num; Primitive* prim = (Primitive*) cur.leaf(num);

          size_t lazy_node = 0;
          if (PrimitiveIntersectorK::occluded(pre,ray,k,prim,num,bvh->scene,lazy_node)) {
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
      
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
    };
  }
}
