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
#include "bvh_traverser1.h"
#include "../../common/ray.h"

namespace embree
{
  namespace isa 
  {
    /*! Converts single ray intersector into packet intersector. */
    template<int N, int K, typename Intersector1>
    class BVHNIntersectorKFromIntersector1
    {
      typedef BVHN<N> BVH;

    public:
      static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
      static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
    };

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
      typedef typename BVH::Node Node;
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

            /* intersect node */
            BVHNNodeIntersector1<N,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,vmask);
            size_t mask = movemask(vmask);

            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;

            /* select next child and push other children */
            BVHNTraverser1<N,types>::traverseClosestHit(cur,mask,vmask,tNear,stackPtr,stackEnd);
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
        NodeRef* stackPtr = stack+1;     //!< current stack pointer
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

            /* intersect node */
            BVHNNodeIntersector1<N,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,vmask);
            size_t mask = movemask(vmask);

            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;

            /* select next child and push other children */
            BVHNTraverser1<N,types>::traverseAnyHit(cur,mask,vmask,tNear,stackPtr,stackEnd);
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
