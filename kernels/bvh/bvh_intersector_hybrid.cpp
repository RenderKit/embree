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

#include "bvh_intersector_hybrid.h"
#include "bvh_traverser1.h"
#include "bvh_intersector_node.h"

#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector.h"
#include "../geometry/trianglev_intersector.h"
#include "../geometry/trianglev_mb_intersector.h"
#include "../geometry/trianglei_intersector.h"
#include "../geometry/quadv_intersector.h"
#include "../geometry/quadi_intersector.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/linei_intersector.h"
#include "../geometry/subdivpatch1eager_intersector.h"
#include "../geometry/subdivpatch1cached_intersector.h"
#include "../geometry/object_intersector.h"

#define SWITCH_DURING_DOWN_TRAVERSAL 1
#define FORCE_SINGLE_MODE 0
#define ENABLE_FAST_COHERENT_CODEPATHS 1


#define STREAM_IN_HYBRID 0

namespace embree
{
  namespace isa
  {

    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::intersect1(const BVH* bvh,
                                                                                           NodeRef root,
                                                                                           const size_t k,
                                                                                           Precalculations& pre,
                                                                                           RayK<K>& ray,
                                                                                           const Vec3vf<K> &ray_org,
                                                                                           const Vec3vf<K> &ray_dir,
                                                                                           const Vec3vf<K> &ray_rdir,
                                                                                           const vfloat<K> &ray_tnear,
                                                                                           const vfloat<K> &ray_tfar,
                                                                                           const Vec3vi<K>& nearXYZ,
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
#if defined(__AVX512ER__)
        /* much faster on KNL */
        if (unlikely(any(vfloat<Nx>(*(float*)&stackPtr->dist) > ray_far)))
          continue;
#else
        if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
          continue;
#endif

        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf node */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);

          /* intersect node */
          size_t mask = 0;
          vfloat<Nx> tNear;
          BVHNNodeIntersector1<N,Nx,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,mask);

          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;

          /* select next child and push other children */
          BVHNNodeTraverser1<N,Nx,types>::traverseClosestHit(cur,mask,tNear,stackPtr,stackEnd);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        size_t lazy_node = 0;
        PrimitiveIntersectorK::intersect(pre, ray, k, context, prim, num, lazy_node);

        ray_far = ray.tfar[k];

        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = neg_inf;
          stackPtr++;
        }
      }
    }

    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::intersect(vint<K>* __restrict__ valid_i, Accel::Intersectors* __restrict__ This, RayK<K>& __restrict__ ray, IntersectContext* __restrict__ context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      
#if ENABLE_FAST_COHERENT_CODEPATHS == 1
      assert(context);
      if (unlikely(types == BVH_AN1 && context->user && isCoherent(context->user->flags)))
      {
        intersect_coherent(valid_i,This,ray,context);
        return;
      }
#endif

      /* filter out invalid rays */
      vbool<K> valid = *valid_i == -1;
#if defined(EMBREE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* return if there are no valid rays */
      size_t valid_bits = movemask(valid);
      if (unlikely(valid_bits == 0)) return;

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,(ray.time >= 0.0f) & (ray.time <= 1.0f)));
      Precalculations pre(valid,ray);

      /* load ray */
      Vec3vf<K> ray_org = ray.org;
      Vec3vf<K> ray_dir = ray.dir;
      vfloat<K> ray_tnear = max(ray.tnear,0.0f);
      vfloat<K> ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vf<K> rdir = rcp_safe(ray_dir);
      const Vec3vf<K> org(ray_org);
      const Vec3vf<K> org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));

      /* compute near/far per ray */
      Vec3vi<K> nearXYZ;
#if FORCE_SINGLE_MODE == 0
      if (single)
#endif
      {
        nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
        nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
        nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));
      }

      /* determine switch threshold based on flags */
      const size_t switchThreshold = (context->user && isCoherent(context->user->flags)) ? 2 : switchThresholdIncoherent;

      vint<K> octant = ray.octant();
      octant = select(valid, octant, vint<K>(0xffffffff));

      do
      {
        const size_t valid_index = __bsf(valid_bits);
        const vbool<K> octant_valid = octant[valid_index] == octant;

#if defined(__AVX__)
        STAT3(normal.trav_hit_boxes[__popcnt(movemask(octant_valid))],1,1,1);
#endif

        valid_bits &= ~(size_t)movemask(octant_valid);

        const vfloat<K> inf = vfloat<K>(pos_inf);

        /* allocate stack and push root node */
        vfloat<K> stack_near[stackSizeChunk];
        NodeRef stack_node[stackSizeChunk];
        stack_node[0] = BVH::invalidNode;
        stack_near[0] = inf;
        stack_node[1] = bvh->root;
        stack_near[1] = select(octant_valid,ray_tnear,inf);
        NodeRef* stackEnd MAYBE_UNUSED = stack_node+stackSizeChunk;
        NodeRef* __restrict__ sptr_node = stack_node + 2;
        vfloat<K>* __restrict__ sptr_near = stack_near + 2;

        while (1) pop:
        {
          /* pop next node from stack */
          assert(sptr_node > stack_node);
          sptr_node--;
          sptr_near--;
          NodeRef cur = *sptr_node;
          if (unlikely(cur == BVH::invalidNode)) {
            assert(sptr_node == stack_node);
            break;
          }

          /* cull node if behind closest hit point */
          vfloat<K> curDist = *sptr_near;
          const vbool<K> active = curDist < ray_tfar;
          if (unlikely(none(active)))
            continue;

          /* switch to single ray traversal */
#if (!defined(__WIN32__) || defined(__X86_64__)) && defined(__SSE4_2__)
#if FORCE_SINGLE_MODE == 0
          if (single)
#endif
          {
            size_t bits = movemask(active);
#if FORCE_SINGLE_MODE == 0
            if (unlikely(__popcnt(bits) <= switchThreshold))
#endif
            {
              for (; bits!=0; ) {
                const size_t i = __bscf(bits);
                intersect1(bvh, cur, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ, context);
              }
              ray_tfar = min(ray_tfar,ray.tfar);
              continue;
            }
          }
#endif
          while (likely(!cur.isLeaf()))
          {
            /* process nodes */
            const vbool<K> valid_node = ray_tfar > curDist;
            STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
            const NodeRef nodeRef = cur;
            const BaseNode* __restrict__ const node = nodeRef.baseNode(types);

            /* set cur to invalid */
            cur = BVH::emptyNode;
            curDist = pos_inf;

            size_t num_child_hits = 0;

            for (unsigned i=0; i<N; i++)
            {
              const NodeRef child = node->children[i];
              if (unlikely(child == BVH::emptyNode)) break;
              vfloat<K> lnearP;
              vbool<K> lhit = valid_node;
              BVHNNodeIntersectorK<N,K,types,robust>::intersect(nodeRef,i,org,ray_dir,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP,lhit);

              /* if we hit the child we choose to continue with that child if it
                 is closer than the current next child, or we push it onto the stack */
              if (likely(any(lhit)))
              {                                
                assert(sptr_node < stackEnd);
                assert(child != BVH::emptyNode);
                const vfloat<K> childDist = select(lhit,lnearP,inf);
                /* push cur node onto stack and continue with hit child */
                if (any(childDist < curDist))
                {
                  if (likely(cur != BVH::emptyNode)) {
                    num_child_hits++;
                    *sptr_node = cur; sptr_node++;
                    *sptr_near = curDist; sptr_near++;
                  }
                  curDist = childDist;
                  cur = child;
                }

                /* push hit child onto stack */
                else {
                  num_child_hits++;                  
                  *sptr_node = child; sptr_node++;
                  *sptr_near = childDist; sptr_near++;
                }
              }
            }

#if defined(__AVX__)
            //STAT3(normal.trav_hit_boxes[num_child_hits],1,1,1);
#endif

            if (unlikely(cur == BVH::emptyNode))
              goto pop;
            
            /* improved distance sorting for 3 or more hits */
            if (unlikely(num_child_hits >= 2))
            {
              if (any(sptr_near[-2] < sptr_near[-1]))
              {
                std::swap(sptr_near[-2],sptr_near[-1]);
                std::swap(sptr_node[-2],sptr_node[-1]);
              }
              if (unlikely(num_child_hits >= 3))
              {
                if (any(sptr_near[-3] < sptr_near[-1]))
                {
                  std::swap(sptr_near[-3],sptr_near[-1]);
                  std::swap(sptr_node[-3],sptr_node[-1]);
                }
                if (any(sptr_near[-3] < sptr_near[-2]))
                {
                  std::swap(sptr_near[-3],sptr_near[-2]);
                  std::swap(sptr_node[-3],sptr_node[-2]);
                }
              }
            }

#if SWITCH_DURING_DOWN_TRAVERSAL == 1
            if (single)
            {
              // seems to be the best place for testing utilization
              if (unlikely(popcnt(ray_tfar > curDist) <= switchThreshold))
              {
                *sptr_node++ = cur;
                *sptr_near++ = curDist;
                goto pop;
              }
            }
#endif
          }

          /* return if stack is empty */
          if (unlikely(cur == BVH::invalidNode)) {
            assert(sptr_node == stack_node);
            break;
          }

          /* intersect leaf */
          assert(cur != BVH::emptyNode);
          const vbool<K> valid_leaf = ray_tfar > curDist;
          STAT3(normal.trav_leaves,1,popcnt(valid_leaf),K);
          if (unlikely(none(valid_leaf))) continue;
          size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

          size_t lazy_node = 0;
          PrimitiveIntersectorK::intersect(valid_leaf,pre,ray,context,prim,items,lazy_node);
          ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

          if (unlikely(lazy_node)) {
            *sptr_node = lazy_node; sptr_node++;
            *sptr_near = neg_inf;   sptr_near++;
          }
        }
      } while(valid_bits);

      AVX_ZERO_UPPER();
    }


    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::intersect_coherent(vint<K>* __restrict__ valid_i,
                                                                                                   Accel::Intersectors* __restrict__ This,
                                                                                                   RayK<K>& __restrict__ ray,
                                                                                                   IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      
      /* filter out invalid rays */
      vbool<K> valid = *valid_i == -1;
#if defined(EMBREE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* return if there are no valid rays */
      size_t valid_bits = movemask(valid);
      if (unlikely(valid_bits == 0)) return;

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,(ray.time >= 0.0f) & (ray.time <= 1.0f)));
      Precalculations pre(valid,ray);

      /* load ray */
      Vec3vf<K> ray_org = ray.org;
      Vec3vf<K> ray_dir = ray.dir;
      const vfloat<K> org_ray_tnear = max(ray.tnear,0.0f);
      const vfloat<K> org_ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vf<K> rdir = rcp_safe(ray_dir);
      const Vec3vf<K> org(ray_org);
      const Vec3vf<K> org_rdir = org * rdir;

      vint<K> octant = ray.octant();
      octant = select(valid,octant,vint<K>(0xffffffff));

      do
      {
        const size_t valid_index = __bsf(valid_bits);
        const vbool<K> octant_valid = octant[valid_index] == octant;
        valid_bits &= ~(size_t)movemask(octant_valid);

        const vfloat<K> ray_tnear = select(octant_valid,org_ray_tnear,vfloat<K>(pos_inf));
              vfloat<K> ray_tfar  = select(octant_valid,org_ray_tfar ,vfloat<K>(neg_inf));

        Frustum<N,Nx,K,robust> frustum(octant_valid,org,rdir,ray_tnear,ray_tfar);

        StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes
        StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
        stack[0].ptr  = bvh->root;
        stack[0].dist = neg_inf;

        while (1) pop:
        {
          /* pop next node from stack */
          if (unlikely(stackPtr == stack)) break;

          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);

          /* cull node if behind closest hit point */
          vfloat<K> curDist = *(float*)&stackPtr->dist;
          const vbool<K> active = curDist < ray_tfar;
          if (unlikely(none(active))) continue;

          while (likely(!cur.isLeaf()))
          {
            /* process nodes */
            //STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
            const NodeRef nodeRef = cur;
            const AlignedNode* __restrict__ const node = nodeRef.alignedNode();

            vfloat<Nx> fmin;
            size_t m_frusta_node = frustum.intersect(node,fmin);

            if (unlikely(!m_frusta_node)) goto pop;
            cur = BVH::emptyNode;
            curDist = pos_inf;
            
#if defined(__AVX__)
            //STAT3(normal.trav_hit_boxes[__popcnt(m_frusta_node)],1,1,1);
#endif
            size_t num_child_hits = 0;
            do {
              const size_t i = __bscf(m_frusta_node);
              vfloat<K> lnearP;
              vbool<K> lhit = false; // motion blur is not supported, so the initial value will be ignored
              STAT3(normal.trav_nodes,1,1,1);
              BVHNNodeIntersectorK<N,K,types,robust>::intersect(nodeRef,i,org,ray_dir,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP,lhit);

              if (likely(any(lhit)))
              {                                
                const vfloat<K> childDist = fmin[i];
                const NodeRef child = node->child(i);
                child.prefetch();
                if (any(childDist < curDist))
                {
                  if (likely(cur != BVH::emptyNode)) {
                    num_child_hits++;
                    stackPtr->ptr = cur;
                    *(float*)&stackPtr->dist = toScalar(curDist);
                    stackPtr++;
                  }
                  curDist = childDist;
                  cur = child;
                }
                /* push hit child onto stack */
                else {
                  num_child_hits++;
                  stackPtr->ptr = child;
                  *(float*)&stackPtr->dist = toScalar(childDist);
                  stackPtr++;
                }                
              }
            } while(m_frusta_node);

            if (unlikely(cur == BVH::emptyNode)) goto pop;

            /* improved distance sorting for 3 or more hits */
            if (unlikely(num_child_hits >= 2))
            {
              if (stackPtr[-2].dist < stackPtr[-1].dist)
                std::swap(stackPtr[-2],stackPtr[-1]);
              if (unlikely(num_child_hits >= 3))
              {
                if (stackPtr[-3].dist < stackPtr[-1].dist)
                  std::swap(stackPtr[-3],stackPtr[-1]);
                if (stackPtr[-3].dist < stackPtr[-2].dist)
                  std::swap(stackPtr[-3],stackPtr[-2]);
              }
            }
          }

          /* intersect leaf */
          assert(cur != BVH::invalidNode);
          assert(cur != BVH::emptyNode);
          const vbool<K> valid_leaf = ray_tfar > curDist;
          STAT3(normal.trav_leaves,1,popcnt(valid_leaf),K);
          if (unlikely(none(valid_leaf))) continue;
          size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

          size_t lazy_node = 0;
          PrimitiveIntersectorK::intersect(valid_leaf,pre,ray,context,prim,items,lazy_node);

          /* reduce max distance interval on successful intersection */
          if (likely(any((ray.tfar < ray_tfar) & valid_leaf)))
          {
            ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
            frustum.updateMaxDist(ray_tfar);
          }

          if (unlikely(lazy_node)) {
            stackPtr->ptr = lazy_node;
            stackPtr->dist = neg_inf;
            stackPtr++;
          }
        }
        
      } while(valid_bits);

      AVX_ZERO_UPPER();
    }

    // ===================================================================================================================================================================
    // ===================================================================================================================================================================
    // ===================================================================================================================================================================

    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    bool BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::occluded1(const BVH* bvh, NodeRef root, const size_t k, Precalculations& pre,
                                                                                          RayK<K>& ray, const Vec3vf<K> &ray_org, const Vec3vf<K> &ray_dir, const Vec3vf<K> &ray_rdir, const vfloat<K> &ray_tnear, const vfloat<K> &ray_tfar,
                                                                                          const Vec3vi<K>& nearXYZ, IntersectContext* context)
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
            BVHNNodeIntersector1<N,Nx,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time[k],tNear,mask);

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

    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::occluded(vint<K>* __restrict__ valid_i, Accel::Intersectors* __restrict__ This, RayK<K>& __restrict__ ray, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      
#if ENABLE_FAST_COHERENT_CODEPATHS == 1
      assert(context);
      if (unlikely(types == BVH_AN1 && context->user && isCoherent(context->user->flags)))
      {
        occluded_coherent(valid_i,This,ray,context);
        return;
      }
#endif

      /* filter out already occluded and invalid rays */
      vbool<K> valid = (*valid_i == -1) & (ray.geomID != 0);
#if defined(EMBREE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* return if there are no valid rays */
      size_t valid_bits = movemask(valid);
      if (unlikely(valid_bits == 0)) return;

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,(ray.time >= 0.0f) & (ray.time <= 1.0f)));
      Precalculations pre(valid,ray);

      /* load ray */
      vbool<K> terminated = !valid;
      Vec3vf<K> ray_org = ray.org, ray_dir = ray.dir;
      vfloat<K> ray_tnear = max(ray.tnear,0.0f);
      vfloat<K> ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vf<K> rdir = rcp_safe(ray_dir);
      const Vec3vf<K> org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));

#if STREAM_IN_HYBRID == 1
      Vec3vi<K> nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
      nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
      nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

      const int shiftTable[8] = { (int)1 << 0, (int)1 << 1, (int)1 << 2, (int)1 << 3, (int)1 << 4, (int)1 << 5, (int)1 << 6, (int)1 << 7 };

      {

        /*! stack state */
        StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes
        StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
        //StackItemT<NodeRef>* stackEnd = stack + stackSizeSingle;
        stack[0].ptr = bvh->root;
        stack[0].dist = movemask(valid);



        while (1) pop:
        {
          if (unlikely(stackPtr == stack)) break;
          STAT3(shadow.trav_stack_pop,1,1,1);
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
          size_t cur_mask = stackPtr->dist & movemask(!terminated);
          if (unlikely(cur_mask == 0)) continue;

          while (true)
          {

            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf())) break;
          
            const AlignedNode* __restrict__ const node = cur.alignedNode();

            const vfloat<Nx> bminX = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x));
            const vfloat<Nx> bmaxX = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->upper_x));
            const vfloat<Nx> bminY = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_y));
            const vfloat<Nx> bmaxY = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->upper_y));
            const vfloat<Nx> bminZ = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_z));
            const vfloat<Nx> bmaxZ = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->upper_z));
          
            size_t bits = cur_mask;

#if defined(__AVX__)
            STAT3(shadow.trav_hit_boxes[__popcnt(cur_mask)],1,1,1);
#endif

            assert(bits);
            const vbool<Nx> valid_children = bminX <= bmaxX;

            vint<Nx> vmask(zero);
            do
            {   
              STAT3(shadow.trav_nodes,1,1,1);

              const size_t i = __bscf(bits);
              assert(i < K);
              //const vint<Nx> bitmask = vint<Nx>((int)1 << i);

              const vint<Nx> bitmask(shiftTable[i]);

              const vfloat<Nx> tNearX = msub(bminX, rdir.x[i], org_rdir.x[i]);
              const vfloat<Nx> tNearY = msub(bminY, rdir.y[i], org_rdir.y[i]);
              const vfloat<Nx> tNearZ = msub(bminZ, rdir.z[i], org_rdir.z[i]);
              const vfloat<Nx> tFarX  = msub(bmaxX, rdir.x[i], org_rdir.x[i]);
              const vfloat<Nx> tFarY  = msub(bmaxY, rdir.y[i], org_rdir.y[i]);
              const vfloat<Nx> tFarZ  = msub(bmaxZ, rdir.z[i], org_rdir.z[i]);
 
              const vfloat<Nx> tNear  = maxi(mini(tNearX,tFarX), mini(tNearY,tFarY), mini(tNearZ,tFarZ), vfloat<Nx>(ray_tnear[i]));
              const vfloat<Nx> tFar   = mini(maxi(tNearX,tFarX), maxi(tNearY,tFarY), maxi(tNearZ,tFarZ), vfloat<Nx>(ray_tfar[i]));
              const vbool<Nx> hit_mask   = tNear <= tFar;
#if defined(__AVX2__)
              vmask = vmask | (bitmask & vint<Nx>(hit_mask));
#else
              vmask = select(hit_mask, vmask | bitmask, vmask);
#endif
            } while(bits);     
            size_t mask = movemask( (vmask != vint<Nx>(zero)) & valid_children);
            if (unlikely(mask == 0)) goto pop;

            /* select next child and push other children */
            //const BaseNode* node = cur.baseNode(types);

            /*! one child is hit, continue with that child */
            size_t r = __bscf(mask);
            assert(r < N);
            cur = node->child(r);         
            cur.prefetch(types);
            cur_mask = vmask[r];

            /* simple in order sequence */
            assert(cur != BVH::emptyNode);
            if (likely(mask == 0)) continue;
            stackPtr->ptr  = cur;
            stackPtr->dist = cur_mask;
            stackPtr++;

            for (; ;)
            {
              r = __bscf(mask);
              assert(r < N);

              cur = node->child(r);          
              cur.prefetch(types);
              cur_mask = vmask[r];
              assert(cur != BVH::emptyNode);
              if (likely(mask == 0)) break;
              stackPtr->ptr  = cur;
              stackPtr->dist = cur_mask;
              stackPtr++;
            }
          }

        
          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(shadow.trav_leaves,1,1,1);
          size_t num; Primitive* prim = (Primitive*) cur.leaf(num);        


          size_t bits = cur_mask;
          size_t lazy_node = 0;

          for (; bits!=0; ) 
          {
            const size_t k = __bscf(bits);
            if (PrimitiveIntersectorK::occluded(pre,ray,k,context,prim,num,lazy_node)) {
              ray.geomID[k] = 0;
              set(terminated, k);
            }
            /* lazy node */
            if (unlikely(lazy_node)) {
              stackPtr->ptr = lazy_node;
              stackPtr->dist = cur_mask;
              stackPtr++;
            }
          }

          if (all(terminated)) break;
        }

      } 
      // while(valid_bits);

#else
      const vfloat<K> inf = vfloat<K>(pos_inf);

      /* compute near/far per ray */
      Vec3vi<K> nearXYZ;
      if (single)
      {
        nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
        nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
        nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));
      }

      /* determine switch threshold based on flags */
      const size_t switchThreshold = (context->user && isCoherent(context->user->flags)) ? 2 : switchThresholdIncoherent;

      /* allocate stack and push root node */
      vfloat<K> stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear;
      NodeRef* stackEnd MAYBE_UNUSED = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      vfloat<K>* __restrict__ sptr_near = stack_near + 2;

      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        /* cull node if behind closest hit point */
        vfloat<K> curDist = *sptr_near;
        const vbool<K> active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;

        /* switch to single ray traversal */
#if (!defined(__WIN32__) || defined(__X86_64__)) && defined(__SSE4_2__)
        if (single)
        {
          size_t bits = movemask(active);
          if (unlikely(__popcnt(bits) <= switchThreshold)) {
            for (; bits!=0; ) {
              const size_t i = __bscf(bits);
              if (occluded1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ,context))
                set(terminated, i);
            }
            if (all(terminated)) break;
            ray_tfar = select(terminated,vfloat<K>(neg_inf),ray_tfar);
            continue;
          }
        }
#endif

        while (likely(!cur.isLeaf()))
        {
          /* process nodes */
          const vbool<K> valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),K);
          const NodeRef nodeRef = cur;
          const BaseNode* __restrict__ const node = nodeRef.baseNode(types);

          /* set cur to invalid */
          cur = BVH::emptyNode;
          curDist = pos_inf;

          for (unsigned i=0; i<N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH::emptyNode)) break;
            vfloat<K> lnearP;
            vbool<K> lhit = valid_node;
            BVHNNodeIntersectorK<N,K,types,robust>::intersect(nodeRef,i,org,ray_dir,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP,lhit);

            /* if we hit the child we push the previously hit node onto the stack, and continue with the currently hit child */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              assert(child != BVH::emptyNode);
              const vfloat<K> childDist = select(lhit,lnearP,inf);

              /* push 'cur' node onto stack and continue with hit child */
              if (likely(cur != BVH::emptyNode)) {
                *sptr_node = cur; sptr_node++;
                *sptr_near = curDist; sptr_near++;
              }
              curDist = childDist;
              cur = child;
            }
          }
          if (unlikely(cur == BVH::emptyNode))
            goto pop;

#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          if (single)
          {
            // seems to be the best place for testing utilization
            if (unlikely(popcnt(ray_tfar > curDist) <= switchThreshold))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
          }
#endif
	}

        /* return if stack is empty */
        if (unlikely(cur == BVH::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }


        /* intersect leaf */
        assert(cur != BVH::emptyNode);
        const vbool<K> valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),K);
        if (unlikely(none(valid_leaf))) continue;
        size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
#if defined(__AVX__)
        STAT3(shadow.trav_hit_boxes[popcnt(vbool<K>(!terminated))],1,1,1);
#endif

        terminated |= PrimitiveIntersectorK::occluded(!terminated,pre,ray,context,prim,items,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated, vfloat<K>(neg_inf), ray_tfar); // ignore node intersections for terminated rays

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
#endif
      vint<K>::store(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }


    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::occluded_coherent(vint<K>* __restrict__ valid_i, Accel::Intersectors* __restrict__ This,
                                                                                                  RayK<K>& __restrict__ ray, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      
      /* filter out invalid rays */
      vbool<K> valid = *valid_i == -1;
#if defined(EMBREE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* return if there are no valid rays */
      size_t valid_bits = movemask(valid);
      if (unlikely(valid_bits == 0)) return;

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid, (ray.time >= 0.0f) & (ray.time <= 1.0f)));
      Precalculations pre(valid,ray);

      /* load ray */
      vbool<K> terminated = !valid;
      Vec3vf<K> ray_org = ray.org;
      Vec3vf<K> ray_dir = ray.dir;
      const vfloat<K> org_ray_tnear = max(ray.tnear,0.0f);
      const vfloat<K> org_ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vf<K> rdir = rcp_safe(ray_dir);
      const Vec3vf<K> org(ray_org);
      const Vec3vf<K> org_rdir = org * rdir;

      vint<K> octant = ray.octant();
      octant = select(valid, octant, vint<K>(0xffffffff));

      do
      {
        const size_t valid_index = __bsf(valid_bits);
        vbool<K> octant_valid = octant[valid_index] == octant;
        valid_bits &= ~(size_t)movemask(octant_valid);

        const vfloat<K> ray_tnear = select(octant_valid, org_ray_tnear, vfloat<K>(pos_inf));
        vfloat<K> ray_tfar = select(octant_valid, org_ray_tfar, vfloat<K>(neg_inf));

        const Frustum<N,Nx,K,robust> frustum(octant_valid, org, rdir, ray_tnear, ray_tfar);

        StackItemMaskT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes
        StackItemMaskT<NodeRef>* stackPtr = stack + 1;   //!< current stack pointer
        stack[0].ptr  = bvh->root;
        stack[0].mask = (unsigned int)movemask(octant_valid);

        while (1) pop:
        {
          /* pop next node from stack */
          if (unlikely(stackPtr == stack)) break;

          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);

          /* cull node of active rays have already been terminated */
          unsigned int m_active = stackPtr->mask & (~movemask(terminated));

          if (unlikely(m_active == 0)) continue;

          while (likely(!cur.isLeaf()))
          {
            /* process nodes */
            //STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
            const NodeRef nodeRef = cur;
            const AlignedNode* __restrict__ const node = nodeRef.alignedNode();

            vfloat<Nx> fmin;
            size_t m_frusta_node = frustum.intersect(node,fmin);

            if (unlikely(!m_frusta_node)) goto pop;
            cur = BVH::emptyNode;
            m_active = 0;

#if defined(__AVX__)
            //STAT3(normal.trav_hit_boxes[__popcnt(m_frusta_node)],1,1,1);
#endif
            size_t num_child_hits = 0;
            do {
              const size_t i = __bscf(m_frusta_node);
              vfloat<K> lnearP;
              vbool<K> lhit = false; // motion blur is not supported, so the initial value will be ignored
              STAT3(normal.trav_nodes,1,1,1);
              BVHNNodeIntersectorK<N,K,types,robust>::intersect(nodeRef,i,org,ray_dir,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP,lhit);

              if (likely(any(lhit)))
              {                                
                const NodeRef child = node->child(i);
                assert(child != BVH::emptyNode);
                child.prefetch();
                if (likely(cur != BVH::emptyNode)) {
                  num_child_hits++;
                  stackPtr->ptr  = cur;
                  stackPtr->mask = m_active;
                  stackPtr++;
                }
                cur = child;
                m_active = (unsigned int)movemask(lhit);
              }
            } while(m_frusta_node);

            if (unlikely(cur == BVH::emptyNode)) goto pop;
          }

          /* intersect leaf */
          assert(cur != BVH::invalidNode);
          assert(cur != BVH::emptyNode);
#if defined(__AVX__)
          STAT3(normal.trav_leaves,1,__popcnt(m_active),K);
#endif
          if (unlikely(!m_active)) continue;
          size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

          size_t lazy_node = 0;
          terminated |= PrimitiveIntersectorK::occluded(!terminated,pre,ray,context,prim,items,lazy_node);
          octant_valid &= !terminated;
          if (unlikely(none(octant_valid))) break;
          ray_tfar = select(terminated, vfloat<K>(neg_inf), ray_tfar); // ignore node intersections for terminated rays

          if (unlikely(lazy_node)) {
            stackPtr->ptr  = lazy_node;
            stackPtr->mask = (unsigned int)movemask(octant_valid);
            stackPtr++;
          }
        }
      } while(valid_bits);
      vint<K>::store(valid & terminated, &ray.geomID, 0);

      AVX_ZERO_UPPER();
    }
  }
}
