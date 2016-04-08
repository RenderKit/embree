// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "bvh_intersector_stream.h"
#include "bvh_intersector_single.h"
#include "bvh_intersector_node.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/linei_intersector.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle4i_intersector_pluecker.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/grid_aos_intersector1.h"
#include "../geometry/object_intersector1.h"
#include "../geometry/quadv_intersector_moeller.h"
#include "../geometry/quadi_intersector_moeller.h"
#include "../geometry/quadi_intersector_pluecker.h"
#include "../../common/scene.h"

#define DBG(x) 
//PRINT(x)
// todo: make offset constant in AVX512 mode

namespace embree
{
  namespace isa
  {
/* experimental fiber mode */
#define EXPERIMENTAL_FIBER_MODE 0
#define FIBERING 1

/* enable traversal of either two small streams or one large stream */
#define TWO_STREAMS_FIBER_MODE 0 // 0 = no fiber, 1 = switch at pop, 2 = switch at each node, 3 = switch at leaf

#if TWO_STREAMS_FIBER_MODE == 0 && !defined(__AVX512F__) && EXPERIMENTAL_FIBER_MODE == 0
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(unsigned int);
#else
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(size_t);
#endif
    

#if EXPERIMENTAL_FIBER_MODE
    /* pure fiber mode, no streams */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {
      StackItemT<NodeRef> stack[2][stackSize];           //!< stack of nodes 
      TraversalContext contexts[2];
      size_t ctxID = 0;
      //TraversalContext* ctx = &contexts[ctxID];
      TraversalContext ctx;
      //TraversalContext context; TraversalContext* ctx = &context;
      
      size_t r = 0;
      {
        if (r >= numTotalRays) return;
        contexts[ctxID].init((Ray&)*input_rays[r++],bvh,stack[ctxID]);
#if FIBERING
        if (r < numTotalRays) {
          ctxID =(ctxID+1)%2;
          contexts[ctxID].init((Ray&)*input_rays[r++],bvh,stack[ctxID]);
        }
#endif
        ctx = contexts[ctxID];
       
        /* pop loop */
        while (true) pop:
        {
          /*! pop next node */
          if (unlikely(ctx.stackPtr == ctx.stackBegin)) 
          {
            /* fill in new ray */
            if (likely(r < numTotalRays))
              ctx.init((Ray&)*input_rays[r++],bvh,stack[ctxID]);
            
            /* terminate fiber */
            else 
            {
              ctx.pray = nullptr;
              contexts[ctxID] = ctx;

              /* switch to next fiber */
#if FIBERING
              ctxID =(ctxID+1)%2;
              ctx = contexts[ctxID];
#endif
              if (ctx.pray == nullptr) break;
            }
            continue;
          }

          ctx.stackPtr--;
          NodeRef cur = NodeRef(ctx.stackPtr->ptr);
          
          /*! if popped node is too far, pop next one */
          if (unlikely(*(float*)&ctx.stackPtr->dist > ctx.pray->tfar))
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
            bool nodeIntersected = BVHNNodeIntersector1<N,N,types,robust>::intersect(cur,ctx.vray,ctx.ray_near,ctx.ray_far,ctx.pray->time,tNear,mask);
            if (unlikely(!nodeIntersected)) break;
            
            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;
      
            /*! initialize the node traverser */
            BVHNNodeTraverser1<N,N,types> nodeTraverser(ctx.vray);
      
            /* select next child and push other children */
            nodeTraverser.traverseClosestHit(cur,mask,tNear,ctx.stackPtr,ctx.stackEnd);
      
#if 0 && FIBERING
          /* switch to other fiber */
          size_t nextCtxID=(ctxID+1)%2;
          if (!ctx.suspended && unlikely(contexts[nextCtxID].pray))
          {
            /* suspend current fiber */
            ctx.stackPtr->ptr = cur; ctx.stackPtr->dist = neg_inf; ctx.stackPtr++;
            ctx.suspended = true;
            contexts[ctxID] = ctx;
            
            /* switch to next fiber */
            ctxID = nextCtxID;
            ctx = contexts[ctxID];
            goto pop;
          } 
          ctx.suspended = false;
#endif
          }
      
#if FIBERING
          /* switch to other fiber */
          size_t nextCtxID=(ctxID+1)%2;
          if (!ctx.suspended && unlikely(contexts[nextCtxID].pray))
          {
            size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
            for (size_t i=0; i<num; i++) {
              for (size_t j=0; j<=sizeof(Primitive); j+=64)
                prefetchL2((char*)&prim[i]+j);
            }

            /* suspend current fiber */
            ctx.stackPtr->ptr = cur; ctx.stackPtr->dist = neg_inf; ctx.stackPtr++;
            ctx.suspended = true;
            contexts[ctxID] = ctx;
            
            /* switch to next fiber */
            ctxID = nextCtxID;
            ctx = contexts[ctxID];
            goto pop;
          } 
          ctx.suspended = false;
#endif
    
          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves,1,1,1);
          size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
          size_t lazy_node = 0;
          PrimitiveIntersector::intersect(ctx.pre,*ctx.pray,0,prim,num,bvh->scene,nullptr,lazy_node);
          ctx.ray_far = ctx.pray->tfar;
        }
      }
      AVX_ZERO_UPPER();
    }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#else

    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {
      __aligned(64) RayContext ray_ctx[MAX_RAYS_PER_OCTANT];
      __aligned(64) Precalculations pre[MAX_RAYS_PER_OCTANT]; 
      __aligned(64) StackItemMask  stack0[stackSizeSingle];  //!< stack of nodes 
      __aligned(64) StackItemMask  stack1[stackSizeSingle];  //!< stack of nodes 

      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** __restrict__ rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays == 8*sizeof(size_t) ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;
        assert(m_active);

        for (size_t i=0; i<numOctantRays; i++) {
          new (&ray_ctx[i]) RayContext(rays[i]);
          new (&pre[i]) Precalculations(*rays[i],bvh);
        }

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        const NearFarPreCompute pc(ray_ctx[0].rdir);

#if !TWO_STREAMS_FIBER_MODE
        const size_t fiberMask = m_active;
#else
        const size_t fiberMask = ((size_t)1 << ((__popcnt(m_active)+1)>>1))-1;
        assert( ((fiberMask | (~fiberMask)) & m_active) == m_active);
        assert( __popcnt(fiberMask) + __popcnt((~fiberMask) & m_active) == __popcnt(m_active));
#endif
        assert(fiberMask);
        
        StackItemMask* stackPtr      = stack0 + 1;
        StackItemMask* stackPtr_next = stack1 + 1;

        NodeRef cur               = bvh->root;
        size_t m_trav_active      = m_active & fiberMask; // lower half of active rays
        NodeRef cur_next          = bvh->root;
        size_t m_trav_active_next = m_active & (~fiberMask); // upper half of active rays
        if (m_trav_active_next == 0) cur_next = 0;

        assert(__popcnt(m_trav_active_next) <= 32);
#if TWO_STREAMS_FIBER_MODE
        RayFiberContext fiber[2];
        fiber[0].init(cur,m_trav_active,stackPtr,&fiber[1],0);
#if defined(__AVX512F__)
        const size_t offset = 0; // have 8-wide 64bit vector support
#else
        const size_t offset = __bsf(m_trav_active_next);
#endif
        fiber[1].init(cur_next,m_trav_active_next >> offset,stackPtr_next,&fiber[0],offset);
        if (m_trav_active_next == 0) fiber[0].next = &fiber[0];
        RayFiberContext *cur_fiber = &fiber[0];
#endif

        while (1) pop:
        {          
#if TWO_STREAMS_FIBER_MODE == 1
          cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
#endif

          const vfloat<K> inf(pos_inf);

          while (1)
          {
            /* context swap */
#if TWO_STREAMS_FIBER_MODE == 2
            //cur.prefetch_L1(types);
            cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
#endif

            if (unlikely(cur.isLeaf())) break;
            const Node* __restrict__ const node = cur.node();
            STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);
            assert(m_trav_active);

#if defined(__AVX512F__)
            const vlong<K/2> one((size_t)1);

            const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),pc.permX);
            const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),pc.permY);
            const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),pc.permZ);

            vfloat<K> dist(inf);
            vlong<K/2>   maskK(zero);
              
            size_t bits = m_trav_active;
            do
            {            
              STAT3(normal.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              const RayContext &ray = ray_ctx[i];
              const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));                
              const vlong<K/2> bitmask  = one << vlong<K/2>(i);
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = mask_or((vboold8)vmask,maskK,maskK,bitmask);
            } while(bits);              

            const vboold8 vmask8 =  maskK != vlong<K/2>(zero);
            const vbool<K> vmask(vmask8);
            if (unlikely(none(vmask))) 
            {
              /*! pop next node */
              STAT3(normal.trav_stack_pop,1,1,1);                          
              stackPtr--;
              cur = NodeRef(stackPtr->ptr);
              m_trav_active = stackPtr->mask;
              assert(m_trav_active);
              goto pop;
            }

            BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (size_t*)&maskK, stackPtr);

#else
            const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearX));
            const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearY));
            const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearZ));
            const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farX));
            const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farY));
            const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farZ));

            vfloat<K> dist(inf);
            vint<K>   maskK(zero);

#if !TWO_STREAMS_FIBER_MODE
            const RayContext *__restrict__ const cur_ray_ctx = ray_ctx;
#else
            const RayContext *__restrict__ const cur_ray_ctx = &ray_ctx[cur_fiber->getOffset()];
#endif
            size_t bits = m_trav_active;
            do
            {            
              STAT3(normal.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              const RayContext &ray = cur_ray_ctx[i];
              const vfloat<K> tNearX = msub(bminX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearY = msub(bminY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearZ = msub(bminZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tFarX  = msub(bmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tFarY  = msub(bmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tFarZ  = msub(bmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vint<K> bitmask  = vint<K>((int)1 << i);
#if defined(__AVX2__)
              const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ray.rdir.w)));
              const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(ray.org_rdir.w)));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = maskK | (bitmask & vint<K>(vmask));
#else
              const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = select(vmask,maskK | bitmask,maskK); 
#endif
            } while(bits);              

            const vbool<K> vmask = dist < inf;
            if (unlikely(none(vmask))) 
            {
              /*! pop next node */
              STAT3(normal.trav_stack_pop,1,1,1);                          
              stackPtr--;
              cur = NodeRef(stackPtr->ptr);
              m_trav_active = stackPtr->mask;
              assert(m_trav_active);
              goto pop;
            }

            BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (unsigned int*)&maskK, stackPtr);
            assert(m_trav_active);
#endif
          }

          /* current ray stream is done? */
          if (unlikely(cur == BVH::invalidNode))
          {
#if !TWO_STREAMS_FIBER_MODE
            break;
#else
            /* both ray streams are done? */ 
            if (cur_fiber->next == cur_fiber)
              break;
            else
            {
              cur_fiber->next->next = cur_fiber->next;
#if TWO_STREAMS_FIBER_MODE == 3
              cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
#endif
              goto pop;
            }
#endif
          }

#if TWO_STREAMS_FIBER_MODE == 3
          cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
          if (unlikely(!cur.isLeaf())) continue;
#endif

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);
          
          STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          
#if !TWO_STREAMS_FIBER_MODE
          size_t bits = m_trav_active;
#else
          size_t bits = m_trav_active << cur_fiber->getOffset();
#endif

          /*! intersect stream of rays with all primitives */
          size_t lazy_node = 0;
          PrimitiveIntersector::intersect(pre,bits,rays,ray_ctx,0,prim,num,bvh->scene,NULL,lazy_node);

          /*! pop next node */
          STAT3(normal.trav_stack_pop,1,1,1);                          
          stackPtr--;
          cur = NodeRef(stackPtr->ptr);

          m_trav_active = stackPtr->mask;
          assert(m_trav_active);
        } // traversal + intersection

        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
      }
    }

#endif

#if 1
    
    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {
      __aligned(64) RayContext ray_ctx[MAX_RAYS_PER_OCTANT];
      __aligned(64) Precalculations pre[MAX_RAYS_PER_OCTANT]; 
      __aligned(64) StackItemMask  stack0[stackSizeSingle];  //!< stack of nodes 
      __aligned(64) StackItemMask  stack1[stackSizeSingle];  //!< stack of nodes 
      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays ==  8*sizeof(size_t) ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;

        for (size_t i=0; i<numOctantRays; i++) {
          new (&ray_ctx[i]) RayContext(rays[i]);
          new (&pre[i]) Precalculations(*rays[i],bvh);
        }

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;

#if !TWO_STREAMS_FIBER_MODE
        const size_t fiberMask = m_active;
#else
        const size_t fiberMask = ((size_t)1 << ((__popcnt(m_active)+1)>>1))-1;
        assert( ((fiberMask | (~fiberMask)) & m_active) == m_active);
        assert( __popcnt(fiberMask) + __popcnt((~fiberMask) & m_active) == __popcnt(m_active));
#endif
        assert(fiberMask);
        
        StackItemMask* stackPtr      = stack0 + 1;
        StackItemMask* stackPtr_next = stack1 + 1;

        NodeRef cur               = bvh->root;
        size_t m_trav_active      = m_active & fiberMask; // lower half of active rays
        NodeRef cur_next          = bvh->root;
        size_t m_trav_active_next = m_active & (~fiberMask); // upper half of active rays

        if (m_trav_active_next == 0) cur_next = 0;


#if TWO_STREAMS_FIBER_MODE
        assert(__popcnt(m_trav_active_next) <= 32);

        RayFiberContext fiber[2];
        fiber[0].init(cur,m_trav_active,stackPtr,&fiber[1],0);
#if defined(__AVX512F__)
        const size_t offset = 0; // have 8-wide 64bit vector support
#else
        const size_t offset = m_trav_active_next != 0 ? __bsf(m_trav_active_next) : 0;
#endif

        fiber[1].init(cur_next,m_trav_active_next >> offset,stackPtr_next,&fiber[0],offset);
        if (m_trav_active_next == 0) fiber[0].next = &fiber[0];
        RayFiberContext *cur_fiber = &fiber[0];
#endif

        const NearFarPreCompute pc(ray_ctx[0].rdir);

        while (1) pop:
        {
#if TWO_STREAMS_FIBER_MODE == 1
            cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
#endif
          const vfloat<K> inf(pos_inf);

          while (1)
          {
#if TWO_STREAMS_FIBER_MODE == 2
            cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
#endif

            if (likely(cur.isLeaf())) break;
            assert(m_trav_active);

            const Node* __restrict__ const node = cur.node();
            STAT3(shadow.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

#if defined(__AVX512F__)
            const vlong<K/2> one((size_t)1);
            const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),pc.permX);
            const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),pc.permY);
            const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),pc.permZ);

            vfloat<K> dist(inf);
            vlong<K/2>   maskK(zero);

            size_t bits = m_trav_active;
            do
            {            
              STAT3(shadow.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              assert(i<MAX_RAYS_PER_OCTANT);
              RayContext &ray = ray_ctx[i];
              const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));                
              const vlong<K/2> bitmask  = one << vlong<K/2>(i);
              maskK = mask_or((vboold8)vmask,maskK,maskK,bitmask);
            } while(bits);          
            const vboold8 vmask = (maskK != vlong<K/2>(zero)); 
            if (unlikely(none(vmask))) 
            {
              /*! pop next node */
              STAT3(shadow.trav_stack_pop,1,1,1);                          
              do {
                //assert(stackPtr > stack);
                stackPtr--;
                cur = NodeRef(stackPtr->ptr);
                assert(stackPtr->mask);
                m_trav_active = stackPtr->mask & m_active;
              } while (unlikely(!m_trav_active));
              assert(m_trav_active);
              goto pop;
            }

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(size_t*)&maskK,stackPtr); 
#else
            const vfloat<K> bminX = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.nearX));
            const vfloat<K> bminY = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.nearY));
            const vfloat<K> bminZ = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.nearZ));
            const vfloat<K> bmaxX = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.farX));
            const vfloat<K> bmaxY = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.farY));
            const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.farZ));

            vfloat<K> dist(inf);
            vint<K>   maskK(zero);

#if !TWO_STREAMS_FIBER_MODE
            const RayContext *__restrict__ const cur_ray_ctx = ray_ctx;
#else
            const RayContext *__restrict__ const cur_ray_ctx = &ray_ctx[cur_fiber->getOffset()];
#endif
            size_t bits = m_trav_active;

            assert(__popcnt(m_trav_active) <= 32);
            do
            {            
              STAT3(shadow.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              const RayContext &ray = cur_ray_ctx[i];
              const vfloat<K> tNearX = msub(bminX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearY = msub(bminY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearZ = msub(bminZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tFarX  = msub(bmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tFarY  = msub(bmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tFarZ  = msub(bmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vint<K> bitmask  = vint<K>((int)1 << i);
#if defined(__AVX2__)
              const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ray.rdir.w)));
              const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(ray.org_rdir.w)));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = maskK | (bitmask & vint<K>(vmask));
#else
              const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = select(vmask,maskK | bitmask,maskK); 
#endif
            } while(bits);          
            const vbool<K> vmask = dist < inf;
            if (unlikely(none(vmask))) 
            {
              /*! pop next node */
              STAT3(shadow.trav_stack_pop,1,1,1);  
              do {
                stackPtr--;
                cur = NodeRef(stackPtr->ptr);
                assert(stackPtr->mask);
#if !TWO_STREAMS_FIBER_MODE
                m_trav_active = stackPtr->mask & m_active;
#else
                m_trav_active = stackPtr->mask & (m_active>>cur_fiber->getOffset());
#endif
              } while (unlikely(cur != BVH::invalidNode && m_trav_active == 0));
              //assert(__popcnt(m_trav_active) <= 32);
              goto pop;
            }

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr); 

#endif

          }
       leaf:

          /* current ray stream is done? */
          if (unlikely(cur == BVH::invalidNode))
          {
#if !TWO_STREAMS_FIBER_MODE
            break;
#else
            if (cur_fiber->next == cur_fiber)
              break;
            else
            {
              cur_fiber->next->next = cur_fiber->next;
#if TWO_STREAMS_FIBER_MODE == 3
              cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
#endif
              goto pop;
            }
#endif
          }

#if TWO_STREAMS_FIBER_MODE == 3
          if (likely(cur_fiber->next != cur_fiber))
          {
            cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
            if (unlikely(!cur.isLeaf())) { continue; }
          }
#endif

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(shadow.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
#if !TWO_STREAMS_FIBER_MODE
          size_t bits = m_trav_active & m_active;          
#else
          size_t bits = (m_trav_active<<cur_fiber->getOffset()) & m_active;          
#endif
          assert(bits);
          STAT3(shadow.trav_hit_boxes[__popcnt(bits)],1,1,1);                          

          m_active &= ~PrimitiveIntersector::occluded(pre,bits,rays,0,prim,num,bvh->scene,NULL,lazy_node);
          if (unlikely(m_active == 0)) break;

          /*! pop next node */
          STAT3(shadow.trav_stack_pop,1,1,1);                          
          do {
            stackPtr--;
            cur = NodeRef(stackPtr->ptr);
            assert(stackPtr->mask);
#if !TWO_STREAMS_FIBER_MODE
            m_trav_active = stackPtr->mask & m_active;
#else
            m_trav_active = stackPtr->mask & (m_active>>cur_fiber->getOffset());
#endif
          } while (unlikely(cur != BVH::invalidNode && m_trav_active == 0));
        } // traversal + intersection        
      }      
    }

#endif


#if 0

    /* experimental multi-stack mode */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {
      const size_t queue_size = 64;
      __aligned(64) RayContext ray_ctx[queue_size];
      __aligned(64) Precalculations pre[queue_size]; // FIXME: initialize
      __aligned(64) int queue[2][queue_size]; 
      size_t queue_left[2] = { 0, 0 }; 
      size_t queue_right[2] = { 0, 0 };

      NodeRef stack[queue_size][1024]; ssize_t stack_ptr[queue_size];

      for (size_t i=0; i<numTotalRays; i+=queue_size)
      {
        Ray** __restrict__ rays = input_rays + i;
        const size_t numRays = (i + queue_size >= numTotalRays) ? numTotalRays-i : queue_size;

        initRayContext(ray_ctx,rays,numRays);
        const NearFarPreCompute pc(ray_ctx[0].rdir);
      
        /* fill queues */
        const int q = bvh->root.isLeaf() != 0;
        for (size_t r=0; r<numRays; r++) 
          queue[q][queue_right[q]++] = r;

        /* push root node onto stack for each ray */
        for (size_t r=0; r<numRays; r++) {
          stack[r][0] = bvh->root;
          stack_ptr[r] = 1;
        }
        
        /* loop until finished */
        do
        {
          size_t trav_queue_left = queue_left[0];
          size_t trav_queue_right = queue_right[0];

          /* traverse all rays */
          while (trav_queue_right-trav_queue_left)
          {
            STAT3(shadow.trav_nodes,1,1,1);
            
            /* take next ray */
            const int r = queue[0][trav_queue_left % queue_size];
            trav_queue_left++;

            /* pop next node from stack */
            Ray& ray = *rays[r];
            NodeRef* stackr = stack[r];
            const RayContext& rayctx = ray_ctx[r];
            ssize_t sptr = stack_ptr[r];
            NodeRef cur = stack[r][--sptr];
            const Node* __restrict__ const node = cur.node();
          
            /* box intersection */
            const vfloat<K> bminX = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.nearX));
            const vfloat<K> bminY = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.nearY));
            const vfloat<K> bminZ = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.nearZ));
            const vfloat<K> bmaxX = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.farX));
            const vfloat<K> bmaxY = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.farY));
            const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+pc.farZ));
            
            const vfloat<K> tNearX = msub(bminX, rayctx.rdir.x, rayctx.org_rdir.x);
            const vfloat<K> tNearY = msub(bminY, rayctx.rdir.y, rayctx.org_rdir.y);
            const vfloat<K> tNearZ = msub(bminZ, rayctx.rdir.z, rayctx.org_rdir.z);
            const vfloat<K> tFarX  = msub(bmaxX, rayctx.rdir.x, rayctx.org_rdir.x);
            const vfloat<K> tFarY  = msub(bmaxY, rayctx.rdir.y, rayctx.org_rdir.y);
            const vfloat<K> tFarZ  = msub(bmaxZ, rayctx.rdir.z, rayctx.org_rdir.z);
            
#if defined(__AVX2__)
            const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ray.tnear)));
            const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(ray.tfar)));
            const vbool<K> vmask   = tNear <= tFar;
#else
            const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ray.tnear));
            const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ray.tfar));
            const vbool<K> vmask   = tNear <= tFar;
#endif
             size_t mask = movemask(vmask);

#if 0
            /* no child hit case */
            if (unlikely(mask == 0)) 
            {
              /* terminate rays */
              stack_ptr[r] = sptr;
              if (unlikely(sptr == 0)) continue;
              
              /* continue rays */
              NodeRef next = stackr[sptr-1];
              next.prefetch_L1();
              const int q = next.isLeaf() != 0;
              queue_right[0] = trav_queue_right;
              queue[q][queue_right[q]++  % queue_size ] = r;
              trav_queue_right = queue_right[0];
              continue;
            }
#endif
            
            /*! two children are hit */
            /*const size_t r0 = __bscf(mask);          
            NodeRef c0 = node->child(r0); 
            const size_t r1 = __bscf(mask);
            NodeRef c1 = node->child(r1); 
            if (likely(mask == 0)) {
              stackr[sptr++] = node->child(0);
              }*/

           
            /* push nodes to stack */
#if 0
            if (mask & 1) stackr[sptr++] = node->child(0); 
            if (mask & 2) stackr[sptr++] = node->child(1); 
            if (mask & 4) stackr[sptr++] = node->child(2); 
            if (mask & 8) stackr[sptr++] = node->child(3); 
#else      
            stackr[sptr] = node->child(0); sptr += vmask[0] & 1;
            stackr[sptr] = node->child(1); sptr += vmask[1] & 1;
            stackr[sptr] = node->child(2); sptr += vmask[2] & 1;
            stackr[sptr] = node->child(3); sptr += vmask[3] & 1;
#endif

            /* terminate rays */
            stack_ptr[r] = sptr;
            if (unlikely(sptr == 0)) continue;

            /* continue rays */
            NodeRef next = stackr[sptr-1];
            next.prefetch_L1();
            const int q = next.isLeaf() != 0;
            queue_right[0] = trav_queue_right;
            queue[q][queue_right[q]++  % queue_size ] = r;
            trav_queue_right = queue_right[0];
          }
          queue_left[0] = trav_queue_left;
          
          /* intersect all rays */
          while (queue_right[1]-queue_left[1])
          {
            STAT3(shadow.trav_leaves,1,1,1);

            /* take next ray */
            const int r = queue[1][queue_left[1] % queue_size];
            queue_left[1]++;
            NodeRef* stackr = stack[r];
            
            /* pop next node from stack */
            ssize_t sptr = stack_ptr[r];
            NodeRef cur = stackr[--sptr];

            /* primitive intersection */
            size_t lazy_node = 0;
            size_t num; Primitive* prim = (Primitive*)cur.leaf(num);
            if (PrimitiveIntersector::occluded(pre[r],*rays[r],0,prim,num,bvh->scene,nullptr,lazy_node)) {
              rays[r]->geomID = 0;
              continue;
            }
            
            /* terminate rays */
            stack_ptr[r] = sptr;
            if (unlikely(sptr == 0)) continue;

            /* continue rays */
            NodeRef next = stackr[sptr-1];
            next.prefetch_L1();
            const int q = next.isLeaf() != 0;
            queue[q][queue_right[q]++  % queue_size ] = r;
          }
        }
        while (queue_right[0]-queue_left[0]);
      }
    }

#endif

    DEFINE_INTERSECTORN(BVH4Line4iStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<LineMiIntersector1<4 COMMA 4 COMMA true> > >);
    //DEFINE_INTERSECTORN(BVH4Line4iMBStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<LineMiMBIntersector1<SIMD_MODE(4) COMMA true> > >);

    DEFINE_INTERSECTORN(BVH4Bezier1vStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >);
    DEFINE_INTERSECTORN(BVH4Bezier1iStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >);
    //DEFINE_INTERSECTORN(BVH4Bezier1vStreamIntersector_OBB,BVHNStreamIntersector<4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >);
    //DEFINE_INTERSECTORN(BVH4Bezier1iStreamIntersector_OBB,BVHNStreamIntersector<4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >);
    //DEFINE_INTERSECTORN(BVH4Bezier1iMBStreamIntersector_OBB,BVHNStreamIntersector<4 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >);

    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersectorMoeller,         BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >);
    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA false> > >);
    //DEFINE_INTERSECTORN(BVH4Triangle4vIntersector1Pluecker,BVHNStreamIntersector<4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<TriangleMvIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >);
    //DEFINE_INTERSECTORN(BVH4Triangle4iIntersector1Pluecker,BVHNStreamIntersector<4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<Triangle4iIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >);
    //DEFINE_INTERSECTORN(BVH4Triangle4vMBIntersector1Moeller,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >);

    DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersectorMoeller,        BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersectorMoellerNoFilter,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >);
    DEFINE_INTERSECTORN(BVH4Quad4iStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >);
    //DEFINE_INTERSECTORN(BVH4Quad4iMBStreamIntersectorPluecker,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<QuadMiMBIntersector1Pluecker<4 COMMA true> > >);

    //DEFINE_INTERSECTORN(BVH4Subdivpatch1CachedStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector1>);
    //DEFINE_INTERSECTORN(BVH4GridAOSStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersector1>);

    DEFINE_INTERSECTORN(BVH4VirtualStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ObjectIntersector1>);
    //DEFINE_INTERSECTORN(BVH4VirtualMBStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ObjectIntersector1>);

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8IntersectorStream Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)

    DEFINE_INTERSECTORN(BVH8Line4iStreamIntersector,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<LineMiIntersector1<4 COMMA 4 COMMA true> > >);
    //DEFINE_INTERSECTORN(BVH8Line4iMBStreamIntersector,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<LineMiMBIntersector1<SIMD_MODE(4) COMMA true> > >);
    
    //DEFINE_INTERSECTORN(BVH8Bezier1vStreamIntersector_OBB,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >);
    //DEFINE_INTERSECTORN(BVH8Bezier1iStreamIntersector_OBB,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >);
    //DEFINE_INTERSECTORN(BVH8Bezier1iMBStreamIntersector_OBB,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >);

    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorMoeller,         BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >);
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA false> > >);
    //DEFINE_INTERSECTORN(BVH8Triangle4vMBStreamIntersectorMoeller,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >);
    
    DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersectorMoeller,         BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >);
    DEFINE_INTERSECTORN(BVH8Quad4iStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >);
    //DEFINE_INTERSECTORN(BVH8Quad4iMBStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<QuadMiMBIntersector1Pluecker<4 COMMA true> > >);

    //DEFINE_INTERSECTORN(BVH8GridAOSStreamIntersector,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA true COMMA GridAOSIntersector1>);

#endif
  }
}



// ===================================================================================================================================================================
// ===================================================================================================================================================================
// ===================================================================================================================================================================


#if 0

      const size_t mask = movemask(vmask8);
      const size_t hits = __popcnt(mask);
      stackPtr--;
      vlong<K/2> cur8(stackPtr->ptr);
      vlong<K/2> m_trav_active8(stackPtr->mask);
      const vlong<K/2> nodes8(*(vlong<K/2>*)node->children);
      cur8           = vlong<K/2>::compact64bit(vmask8,cur8,nodes8);
      m_trav_active8 = vlong<K/2>::compact64bit(vmask8,m_trav_active8,maskK);
      cur            = vlong<K/2>::extract64bit(cur8);
      m_trav_active  = vlong<K/2>::extract64bit(m_trav_active8);

      stackPtr += hits;
      cur.prefetch();
      if (unlikely(hits > 1)) 
      {
        stackPtr-=hits-1;
        __aligned(64) unsigned int dist16[16];
        __aligned(64) size_t mask8[8];
        vfloat<K>::store((float*)dist16,dist);
        vlong<K/2>::store(mask8,maskK);
        BVHNNodeTraverserKHit<types,N,K>::traverseClosest2HitsOrMore(cur, m_trav_active, node, vbool<K>(vmask8), dist16, mask8, stackPtr,hits);
      }                


          if (likely(util == 1))
          {
            const size_t i = __bsf(m_trav_active);
            const vlong<K/2> maskK(m_trav_active);
            RayContext &ray = ray_ctx[i];

            while (likely(!cur.isLeaf()))
            {
              const Node* __restrict__ const node = cur.node();
              STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

              const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),permX);
              const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),permY);
              const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),permZ);
              STAT3(normal.trav_nodes,1,1,1);                          
              const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask      = le(0xff,tNear,align_shift_right<8>(tFar,tFar));                
              if (unlikely(none(vmask))) goto pop;

              BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, tNear, (size_t*)&maskK, stackPtr);
            }
          }
          else

#endif

