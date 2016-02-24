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

#include "bvh_intersector_stream.h"
#include "bvh_intersector_single.h"
#include "bvh_intersector_node.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglei.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle4i_intersector_pluecker.h"
#include "../geometry/quadv_intersector_moeller.h"
#include "../geometry/quadi_intersector_moeller.h"
#include "../geometry/quadi_intersector_pluecker.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/subdivpatch1cached.h"
#include "../geometry/object_intersector.h"
#include "../../common/scene.h"
#define DBG(x) 
//PRINT(x)
// todo: make offset constant in AVX512 mode

namespace embree
{
  namespace isa
  {

#if defined(__AVX__)

    static const size_t MAX_RAYS_PER_OCTANT = 64;
    
#define FIBERS 1

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
        size_t m_active = numOctantRays == 64 ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;
        assert(m_active);

        initRayContext(ray_ctx,rays,numOctantRays);

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        const NearFarPreCompute pc(ray_ctx[0].rdir);

        const size_t fiberMask = ((size_t)1 << ((__popcnt(m_active)+1)>>1))-1;
        assert(fiberMask);
        assert( ((fiberMask | (~fiberMask)) & m_active) == m_active);
        assert( __popcnt(fiberMask) + __popcnt((~fiberMask) & m_active) == __popcnt(m_active));
        
        StackItemMask* stackPtr      = stack0 + 1;
        StackItemMask* stackPtr_next = stack1 + 1;

        NodeRef cur               = bvh->root;
        size_t m_trav_active      = m_active & fiberMask; // lower half of active rays
        NodeRef cur_next          = bvh->root;
        size_t m_trav_active_next = m_active & (~fiberMask); // upper half of active rays
        if (m_trav_active_next == 0) cur_next = 0;

        assert(__popcnt(m_trav_active_next) <= 32);
#if FIBERS == 1
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
          const vfloat<K> inf(pos_inf);
          while (1)
          {
            /* context swap */

#if FIBERS == 0
            if (likely(cur_next))
            {
              std::swap(cur,cur_next);
              std::swap(m_trav_active,m_trav_active_next);
              std::swap(stackPtr,stackPtr_next);
            }
#else
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

            const RayContext *__restrict__ const cur_ray_ctx = &ray_ctx[cur_fiber->getOffset()];

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
#if FIBERS == 0
            /* both ray streams are done? */ 
            if (unlikely(cur_next == 0)) 
              break;
            else
            {
              cur           = cur_next;
              m_trav_active = m_trav_active_next;
              stackPtr      = stackPtr_next;
              cur_next      = 0;
              goto pop;
            }
#else
            if (cur_fiber->next == cur_fiber)
              break;
            else
            {
              cur_fiber->next->next = cur_fiber->next;
              goto pop;
            }
#endif
          }

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);
          
          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

          size_t lazy_node = 0;
          size_t bits = m_trav_active << cur_fiber->getOffset();
          size_t m_valid_intersection = 0;
          do {
            const size_t i = __bscf(bits);
            PrimitiveIntersector::intersect(pre[i],*(rays[i]),0,prim,num,bvh->scene,NULL,lazy_node);
            m_valid_intersection |= rays[i]->tfar < ray_ctx[i].org_rdir.w ? ((size_t)1 << i) : 0;
            ray_ctx[i].org_rdir.w = rays[i]->tfar;
          } while(unlikely(bits));

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
        size_t m_active = numOctantRays == 64 ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;

        initRayContext(ray_ctx,rays,numOctantRays);

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;

        const size_t fiberMask = ((size_t)1 << ((__popcnt(m_active)+1)>>1))-1;
        assert(fiberMask);
        assert( ((fiberMask | (~fiberMask)) & m_active) == m_active);
        assert( __popcnt(fiberMask) + __popcnt((~fiberMask) & m_active) == __popcnt(m_active));
        
        StackItemMask* stackPtr      = stack0 + 1;
        StackItemMask* stackPtr_next = stack1 + 1;

        NodeRef cur               = bvh->root;
        size_t m_trav_active      = m_active & fiberMask; // lower half of active rays
        NodeRef cur_next          = bvh->root;
        size_t m_trav_active_next = m_active & (~fiberMask); // upper half of active rays

        if (m_trav_active_next == 0) cur_next = 0;

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

        const NearFarPreCompute pc(ray_ctx[0].rdir);

        while (1) pop:
        {
          const vfloat<K> inf(pos_inf);

          while (1)
          {
            
            cur_fiber = cur_fiber->swapContext(cur,m_trav_active,stackPtr);
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
                assert(stackPtr > stack);
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

            const RayContext *__restrict__ const cur_ray_ctx = &ray_ctx[cur_fiber->getOffset()];
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
                m_trav_active = stackPtr->mask & (m_active>>cur_fiber->getOffset());
              } while (unlikely(cur != BVH::invalidNode && m_trav_active == 0));
              assert(__popcnt(m_trav_active) <= 32);
              goto pop;
            }

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr); 

#endif

          }

          /* current ray stream is done? */
          if (unlikely(cur == BVH::invalidNode))
          {
            if (cur_fiber->next == cur_fiber)
              break;
            else
            {
              cur_fiber->next->next = cur_fiber->next;
              goto pop;
            }
          }

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(shadow.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          size_t bits = (m_trav_active<<cur_fiber->getOffset()) & m_active;          
          assert(bits);
          //STAT3(shadow.trav_hit_boxes[__popcnt(bits)],1,1,1);                          
          do {
            const size_t i = __bscf(bits);            
            if (PrimitiveIntersector::occluded(pre[i],*(rays[i]),0,prim,num,bvh->scene,NULL,lazy_node))
            {
              m_active &= ~((size_t)1 << i);
              rays[i]->geomID = 0;
            }
          } while(bits);

          if (unlikely(m_active == 0)) 
            break;

          /*! pop next node */
          STAT3(shadow.trav_stack_pop,1,1,1);                          
          do {
            stackPtr--;
            cur = NodeRef(stackPtr->ptr);
            assert(stackPtr->mask);
            m_trav_active = stackPtr->mask & (m_active>>cur_fiber->getOffset());
          } while (unlikely(cur != BVH::invalidNode && m_trav_active == 0));
        } // traversal + intersection        
      }      
    }

    
    

#if defined(__AVX512F__)
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersector, BVHNStreamIntersector<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorNoFilter, BVHNStreamIntersector<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 16 COMMA false> > >);

    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersector, BVHNStreamIntersector<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersectorNoFilter, BVHNStreamIntersector<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 16 COMMA false> > >);

    DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersector, BVHNStreamIntersector<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersectorNoFilter, BVHNStreamIntersector<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >);

    DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersector, BVHNStreamIntersector<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersectorNoFilter, BVHNStreamIntersector<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >);

#else
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersector, BVHNStreamIntersector<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorNoFilter, BVHNStreamIntersector<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA false> > >);
    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersector, BVHNStreamIntersector<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersectorNoFilter, BVHNStreamIntersector<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA false> > >);

    DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersector, BVHNStreamIntersector<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersectorNoFilter, BVHNStreamIntersector<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >);
    DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersector, BVHNStreamIntersector<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersectorNoFilter, BVHNStreamIntersector<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >);

#endif


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

