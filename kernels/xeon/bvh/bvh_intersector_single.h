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

      static __forceinline void intersect1(const BVH* bvh, 
                                           NodeRef root, 
                                           const size_t k, 
                                           Precalculations& pre,
                                           RayK<K>& ray, 
                                           const Vec3vfK &ray_org, 
                                           const Vec3vfK &ray_dir, 
                                           const Vec3vfK &ray_rdir, 
                                           const vfloat<K> &ray_tnear, 
                                           const vfloat<K> &ray_tfar,
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
            size_t mask;
            vfloat<N> tNear;

            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
            STAT3(normal.trav_nodes,1,1,1);

            /* intersect node */
            BVHNNodeIntersector1<N,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,mask);

            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;

            /* select next child and push other children */
            BVHNNodeTraverser1<N,types>::traverseClosestHit(cur,mask,tNear,stackPtr,stackEnd);
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
            size_t mask;
            vfloat<N> tNear;

            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
            STAT3(shadow.trav_nodes,1,1,1);

            /* intersect node */
            BVHNNodeIntersector1<N,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,mask);

            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;

            /* select next child and push other children */
            BVHNNodeTraverser1<N,types>::traverseAnyHit(cur,mask,tNear,stackPtr,stackEnd);
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



#if 1 // defined(__AVX512F__) //defined(__TARGET_AVX512KNL__)


    /*! BVH single ray intersector for packets. */
    template<int types, typename PrimitiveIntersectorK>
      class BVHNIntersectorKSingle<8,16,types,false,PrimitiveIntersectorK>
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
      typedef Vec3<vfloat<N>> Vec3vfN;
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef Vec3<vint<K>> Vec3viK;

      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

    public:

      static __forceinline void intersect1(const BVH* bvh, 
                                           NodeRef root, 
                                           const size_t k, 
                                           Precalculations& pre,
                                           RayK<K>& ray, 
                                           const Vec3vfK &ray_org, 
                                           const Vec3vfK &ray_dir, 
                                           const Vec3vfK &ray_rdir, 
                                           const vfloat<K> &ray_tnear, 
                                           const vfloat<K> &ray_tfar,
                                           const Vec3viK& nearXYZ)
      {
        asm nop;
	/*! stack state */
	StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
	StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
	StackItemT<NodeRef>* stackEnd = stack + stackSizeSingle;
	stack[0].ptr = root;
	stack[0].dist = neg_inf;
	
	/*! load the ray into SIMD registers */
        const vbool16 mask8(0xff);

        TravRay<K> vray(k,ray_org,ray_dir,ray_rdir,nearXYZ, sizeof(vfloat<N>));
        const vfloat<K> ray_near(select(mask8,vfloat<K>(ray_tnear[k]),vfloat<K>(pos_inf)));
              vfloat<K> ray_far (select(mask8,vfloat<K>(ray_tfar[k] ),vfloat<K>(neg_inf)));
        //vfloat<K> ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);

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
            size_t mask;

            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
            STAT3(normal.trav_nodes,1,1,1);

            /* intersect node */
            //BVHNNodeIntersector1<N,types,false>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,mask);
            const typename BVH8::Node* node = cur.node();
#if 1
            const vfloat16 nodeX = vfloat16::load((float*)((const char*)&node->lower_x));
            const vfloat16 nodeY = vfloat16::load((float*)((const char*)&node->lower_y));
            const vfloat16 nodeZ = vfloat16::load((float*)((const char*)&node->lower_z));
            const vbool16 m_active = nodeX != vfloat<K>(pos_inf);
            const vfloat16 tNearFarX = msub(nodeX, vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tNearFarY = msub(nodeY, vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tNearFarZ = msub(nodeZ, vray.rdir.z, vray.org_rdir.z);
            const vfloat16 tFarNearX = align_shift_right<8>(tNearFarX,tNearFarX);
            const vfloat16 tFarNearY = align_shift_right<8>(tNearFarY,tNearFarY);
            const vfloat16 tFarNearZ = align_shift_right<8>(tNearFarZ,tNearFarZ);
            const vfloat16 tNearX = min(tNearFarX,tFarNearX);
            const vfloat16 tFarX  = max(tNearFarX,tFarNearX);
            const vfloat16 tNearY = min(tNearFarY,tFarNearY);
            const vfloat16 tFarY  = max(tNearFarY,tFarNearY);
            const vfloat16 tNearZ = min(tNearFarZ,tFarNearZ);
            const vfloat16 tFarZ  = max(tNearFarZ,tFarNearZ);

            const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near);
            const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);

            const vbool16 vmask = le(m_active,tNear,tFar);

#else
            const vfloat16 tNearX = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearX)), vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tNearY = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearY)), vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tNearZ = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearZ)), vray.rdir.z, vray.org_rdir.z);
            const vfloat16 tFarX  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farX )), vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tFarY  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farY )), vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tFarZ  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farZ )), vray.rdir.z, vray.org_rdir.z);
            const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near);
            const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);

            const vbool16 vmask = tNear <= tFar;

#endif
            mask = movemask(vmask);

            /*! if no child is hit, pop next node */
            if (unlikely(none(vmask))) goto pop;

            /* select next child and push other children */
            vfloat8 tNear8((__m256)tNear);
            BVHNNodeTraverser1<N,types>::traverseClosestHit(cur,mask,tNear8,stackPtr,stackEnd);
          }

	  /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
	  STAT3(normal.trav_leaves, 1, 1, 1);
	  size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          PrimitiveIntersectorK::intersect(pre, ray, k, prim, num, bvh->scene, lazy_node);
	  //ray_far = ray.tfar[k];
          ray_far = select(mask8,vfloat<K>(ray_tfar[k] ),vfloat<K>(neg_inf));

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
        const vbool16 mask8(0xff);
        TravRay<K> vray(k,ray_org,ray_dir,ray_rdir,nearXYZ, sizeof(vfloat<N>));
        const vfloat<K> ray_near(select(mask8,ray_tnear[k],vfloat<K>(pos_inf))), ray_far(select(mask8,ray_tfar[k],vfloat<K>(neg_inf)));
	
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
            size_t mask;

            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
            STAT3(shadow.trav_nodes,1,1,1);

            /* intersect node */
            //BVHNNodeIntersector1<N,types,false>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,mask);
            const typename BVH8::Node* node = cur.node();

            const vfloat16 tNearX = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearX)), vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tNearY = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearY)), vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tNearZ = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearZ)), vray.rdir.z, vray.org_rdir.z);
            const vfloat16 tFarX  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farX )), vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tFarY  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farY )), vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tFarZ  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farZ )), vray.rdir.z, vray.org_rdir.z);
      
            const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near);
            const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);

            const vbool16 vmask = tNear <= tFar;
            mask = movemask(vmask);

            /*! if no child is hit, pop next node */
            if (unlikely(none(vmask))) goto pop;

            vfloat8 tNear8((__m256)tNear);
            /* select next child and push other children */
            BVHNNodeTraverser1<N,types>::traverseAnyHit(cur,mask,tNear8,stackPtr,stackEnd);
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

#endif

  }
}
