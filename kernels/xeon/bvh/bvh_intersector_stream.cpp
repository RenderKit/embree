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

#define DBG(x) 
//PRINT(x)

#define BLANK  
// DBG(std::cout << std::endl)

// todo: permute = 2 x broadcast + mask, stack size for streams


#define FIBERS 2

namespace embree
{
  namespace isa
  {

#if defined(__AVX__)

    
#if 1

    static const size_t MAX_RAYS_PER_OCTANT = 64;

    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {
      __aligned(64) RayContext ray_ctx[MAX_RAYS_PER_OCTANT];
      __aligned(64) Precalculations pre[MAX_RAYS_PER_OCTANT]; 
      __aligned(64) StackItemMask  stacks[FIBERS][stackSizeSingle];  //!< stack of nodes 
      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** __restrict__ rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays == 64 ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;
        assert(m_active);
        for (size_t i=0;i<numOctantRays;i++)
        {
#if defined(__AVX512F__)
          vfloat<K> org(vfloat4(rays[i]->org));
          vfloat<K> dir(vfloat4(rays[i]->dir));
          vfloat<K> rdir       = select(0x7777,rcp_safe(dir),rays[i]->tnear);
          vfloat<K> org_rdir   = select(0x7777,org * rdir,rays[i]->tfar);
          vfloat<K> res = select(0xf,rdir,org_rdir);
          vfloat8 r = extractf256bit(res);
          *(vfloat8*)&ray_ctx[i] = r;          
#else
          Vec3fa &org = rays[i]->org;
          Vec3fa &dir = rays[i]->dir;
          ray_ctx[i].rdir       = rcp_safe(dir);
          ray_ctx[i].org_rdir   = org * ray_ctx[i].rdir;
          ray_ctx[i].rdir.w     = rays[i]->tnear;
          ray_ctx[i].org_rdir.w = rays[i]->tfar;
#endif
        }       
        

        /* stack sentinel */
        for (size_t i=0;i<FIBERS;i++)
          stacks[i][0].ptr = BVH::invalidNode;

        size_t m_active_fibers = m_active & (((size_t)1 << FIBERS)-1);
        m_active &= ~m_active_fibers;

        DBG(__popcnt(m_active_fibers));
        DBG(__popcnt(m_active));

        for (size_t bits = m_active_fibers, i=__bsf(bits), fiberID = 0; bits!=0; bits=__btc(bits,i), i=__bsf(bits),fiberID++) 
        {
          DBG(i);
          DBG(fiberID);

          ray_ctx[i].cur      = bvh->root;
          ray_ctx[i].stackPtr = &stacks[fiberID][1];
          ray_ctx[i].fiberID  = fiberID;
          ray_ctx[i].rayID    = i;
        }
                
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
#if defined(__AVX512F__)
        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);
 

        const vint<K> permX = select(vfloat<K>(ray_ctx[0].rdir.x) >= 0.0f,id,id2);
        const vint<K> permY = select(vfloat<K>(ray_ctx[0].rdir.y) >= 0.0f,id,id2);
        const vint<K> permZ = select(vfloat<K>(ray_ctx[0].rdir.z) >= 0.0f,id,id2);
        const vlong<K/2> one((size_t)1);
#else
        const size_t nearX = (rays[0]->dir.x < 0.0f) ? 1*sizeof(vfloat<N>) : 0*sizeof(vfloat<N>);
        const size_t nearY = (rays[0]->dir.y < 0.0f) ? 3*sizeof(vfloat<N>) : 2*sizeof(vfloat<N>);
        const size_t nearZ = (rays[0]->dir.z < 0.0f) ? 5*sizeof(vfloat<N>) : 4*sizeof(vfloat<N>);

        const size_t farX  = nearX ^ sizeof(vfloat<N>);
        const size_t farY  = nearY ^ sizeof(vfloat<N>);
        const size_t farZ  = nearZ ^ sizeof(vfloat<N>);
#endif

        const vfloat<K> inf(pos_inf);
        
        while (m_active_fibers) 
        {                    
          DBG(__popcnt(m_active_fibers));
          size_t m_cur_active_fibers = m_active_fibers;
          DBG(__popcnt(m_cur_active_fibers));
          STAT3(normal.trav_hit_boxes[__popcnt(m_cur_active_fibers)],1,1,1);

          do
          {
            assert(m_cur_active_fibers);
            const size_t ctxID = __bscf(m_cur_active_fibers);
            DBG(ctxID);
            RayContext &ctx = ray_ctx[ctxID];
            NodeRef cur     = ctx.cur;
            
            if (likely(!cur.isLeaf())) 
            {
              const Node* __restrict__ const node = cur.node();
              STAT3(normal.trav_nodes,1,1,1);                          

#if defined(__AVX512F__)
              const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),permX);
              const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),permY);
              const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),permZ);

              const vfloat<K> tNearFarX = msub(bminmaxX, ctx.rdir.x, ctx.org_rdir.x);
              const vfloat<K> tNearFarY = msub(bminmaxY, ctx.rdir.y, ctx.org_rdir.y);
              const vfloat<K> tNearFarZ = msub(bminmaxZ, ctx.rdir.z, ctx.org_rdir.z);
              const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ctx.rdir.w));
              const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ctx.org_rdir.w));
              const vbool<K> vmask      = le(0xff,tNear,align_shift_right<8>(tFar,tFar));                
              if (unlikely(none(vmask))) 
              {
                /*! pop next node */
                STAT3(normal.trav_stack_pop,1,1,1);                          
                ctx.stackPtr--;
                ctx.cur = NodeRef(ctx.stackPtr->ptr);
              }
              else                
              {
                BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, vmask, tNear, ctx.stackPtr);
                ctx.cur = cur;
              }                


#else
              const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearX));
              const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearY));
              const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearZ));
              const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farX));
              const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farY));
              const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farZ));

              const vfloat<K> tNearX = msub(bminX, ctx.rdir.x, ctx.org_rdir.x);
              const vfloat<K> tNearY = msub(bminY, ctx.rdir.y, ctx.org_rdir.y);
              const vfloat<K> tNearZ = msub(bminZ, ctx.rdir.z, ctx.org_rdir.z);
              const vfloat<K> tFarX  = msub(bmaxX, ctx.rdir.x, ctx.org_rdir.x);
              const vfloat<K> tFarY  = msub(bmaxY, ctx.rdir.y, ctx.org_rdir.y);
              const vfloat<K> tFarZ  = msub(bmaxZ, ctx.rdir.z, ctx.org_rdir.z);
#if defined(__AVX2__)
              const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ctx.rdir.w)));
              const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(ctx.org_rdir.w)));
              const vbool<K> vmask   = tNear <= tFar;
#else
              const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ctx.rdir.w));
              const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ctx.org_rdir.w));
              const vbool<K> vmask   = tNear <= tFar;
#endif
              if (unlikely(none(vmask))) 
              {
                /*! pop next node */
                STAT3(normal.trav_stack_pop,1,1,1);                          
                assert(ctx.stackPtr >= &stacks[ctx.fiberID][0]);
                ctx.stackPtr--;
                ctx.cur = NodeRef(ctx.stackPtr->ptr);
              }
              else
              {
#if defined(__AVX2__)
                BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, vmask, tNear, ctx.stackPtr);
#else
                FATAL("not yet implemented");
#endif
                ctx.cur = cur;
              }

#endif
            }
            else
            {
              DBG("INTERSECTION");

              /* current ray terminated? */
              if (unlikely(cur == BVH::invalidNode))
              {
                DBG("TERMINATE");
                /* deactivate current ray */
                m_active_fibers ^= (size_t)1 << ctx.rayID;

                DBG(__popcnt(m_active));

                /* refill */
                if (m_active)
                {
                  DBG("REFILL");

                  const size_t newID = __bscf(m_active);
                  DBG(newID);
                  DBG(ctx.fiberID);
                  DBG(__popcnt(m_active));

                  m_active_fibers |= (size_t)1 << newID;
                  ray_ctx[newID].cur      = bvh->root;
                  ray_ctx[newID].stackPtr = &stacks[ctx.fiberID][1];
                  ray_ctx[newID].fiberID  = ctx.fiberID;
                  ray_ctx[newID].rayID    = newID;
                }                  
              }
              else
              {

                /*! this is a leaf node */
                assert(cur != BVH::emptyNode);
                STAT3(normal.trav_leaves, 1, 1, 1);
                size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

                //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

                size_t lazy_node = 0;
                PrimitiveIntersector::intersect(pre[ctx.rayID],*rays[ctx.rayID],0,prim,num,bvh->scene,NULL,lazy_node);
                ctx.org_rdir.w = rays[ctx.rayID]->tfar;

                STAT3(normal.trav_stack_pop,1,1,1);                          
                assert(ctx.stackPtr >= &stacks[ctx.fiberID][0]);
                ctx.stackPtr--;
                ctx.cur = NodeRef(ctx.stackPtr->ptr);
                
              }
            } // intersection
          } while (m_cur_active_fibers);
          
          ///////////////////////////////////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////
        }
      }
    }

#elif 0

#if defined(__AVX512F__)
    static const size_t MAX_RAYS_PER_OCTANT = 64;
#else
    static const size_t MAX_RAYS_PER_OCTANT = 32;
#endif
    
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
        for (size_t i=0;i<numOctantRays;i++)
        {
#if defined(__AVX512F__)
          vfloat<K> org(vfloat4(rays[i]->org));
          vfloat<K> dir(vfloat4(rays[i]->dir));
          vfloat<K> rdir       = select(0x7777,rcp_safe(dir),rays[i]->tnear);
          vfloat<K> org_rdir   = select(0x7777,org * rdir,rays[i]->tfar);
          vfloat<K> res = select(0xf,rdir,org_rdir);
          vfloat8 r = extractf256bit(res);
          *(vfloat8*)&ray_ctx[i] = r;          
#else
          Vec3fa &org = rays[i]->org;
          Vec3fa &dir = rays[i]->dir;
          ray_ctx[i].rdir       = rcp_safe(dir);
          ray_ctx[i].org_rdir   = org * ray_ctx[i].rdir;
          ray_ctx[i].rdir.w     = rays[i]->tnear;
          ray_ctx[i].org_rdir.w = rays[i]->tfar;
#endif
        }       

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;
                
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
#if defined(__AVX512F__)
        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);
 

        const vint<K> permX = select(vfloat<K>(ray_ctx[0].rdir.x) >= 0.0f,id,id2);
        const vint<K> permY = select(vfloat<K>(ray_ctx[0].rdir.y) >= 0.0f,id,id2);
        const vint<K> permZ = select(vfloat<K>(ray_ctx[0].rdir.z) >= 0.0f,id,id2);
        const vlong<K/2> one((size_t)1);
#else
        const size_t nearX = (rays[0]->dir.x < 0.0f) ? 1*sizeof(vfloat<N>) : 0*sizeof(vfloat<N>);
        const size_t nearY = (rays[0]->dir.y < 0.0f) ? 3*sizeof(vfloat<N>) : 2*sizeof(vfloat<N>);
        const size_t nearZ = (rays[0]->dir.z < 0.0f) ? 5*sizeof(vfloat<N>) : 4*sizeof(vfloat<N>);

        const size_t farX  = nearX ^ sizeof(vfloat<N>);
        const size_t farY  = nearY ^ sizeof(vfloat<N>);
        const size_t farZ  = nearZ ^ sizeof(vfloat<N>);
#endif
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
        
        while (1) pop:
        {          
          const vfloat<K> inf(pos_inf);
          while (1)
          {
            /* context swap */
            if (likely(cur_next))
            {
              cur_next.prefetchL1C();
              std::swap(cur,cur_next);
              std::swap(m_trav_active,m_trav_active_next);
              std::swap(stackPtr,stackPtr_next);
            }

            if (unlikely(cur.isLeaf())) break;
            const Node* __restrict__ const node = cur.node();
            STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);
#if defined(__AVX512F__)
              const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),permX);
              const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),permY);
              const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),permZ);

              vfloat<K> dist(inf);
              vlong<K/2>   maskK(zero);
              size_t bits = m_trav_active;
              do
              {            
                STAT3(normal.trav_nodes,1,1,1);                          
                const size_t i = __bscf(bits);
                RayContext &ray = ray_ctx[i];
                const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
                const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
                const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
                const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
                const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
                const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));                
                //const vint<K> bitmask  = vint<K>((size_t)1 << i);
                const vlong<K/2> bitmask  = one << vlong<K/2>(i);
                dist   = select(vmask,min(tNear,dist),dist);
                maskK = mask_or((vboold8)vmask,maskK,maskK,bitmask);
              } while(bits);              


              const vbool<K> vmask = lt(0xff,dist,inf);

              if (unlikely(none(vmask))) 
              {
                /*! pop next node */
                STAT3(normal.trav_stack_pop,1,1,1);                          
                // todo: directly assign to *_next
                stackPtr--;
                cur = NodeRef(stackPtr->ptr);
                m_trav_active = stackPtr->mask;
                assert(m_trav_active);
                goto pop;
              }

              BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (size_t*)&maskK, stackPtr);

#else
            const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearX));
            const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearY));
            const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearZ));
            const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farX));
            const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farY));
            const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farZ));

            vfloat<K> dist(inf);
            vint<K>   maskK(zero);

            size_t bits = m_trav_active;
            do
            {            
              STAT3(normal.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              RayContext &ray = ray_ctx[i];
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
              //maskK = maskK | (bitmask & vint<K>((__m256i)vmask));
              maskK = select(vmask,maskK | bitmask,maskK); 
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
              // todo: directly assign to *_next
              stackPtr--;
              cur = NodeRef(stackPtr->ptr);
              m_trav_active = stackPtr->mask;
              assert(m_trav_active);
              goto pop;
            }

#if defined(__AVX2__)
            BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (unsigned int*)&maskK, stackPtr);
            assert(m_trav_active);
#else
            FATAL("not yet implemented");
#endif

#endif
          }
          DBG("INTERSECTION");

          /* current ray stream is done? */
          if (unlikely(cur == BVH::invalidNode))
          {
            /* both ray streams are done? */ 
            if (unlikely(cur_next == 0))
            {
              break;
            }
            else
            {
              cur           = cur_next;
              m_trav_active = m_trav_active_next;
              stackPtr      = stackPtr_next;
              cur_next      = 0;
              goto pop;
            }
          }

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

          size_t lazy_node = 0;
          size_t bits = m_trav_active;
          size_t m_valid_intersection = 0;
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) 
          {
            PrimitiveIntersector::intersect(pre[i],*(rays[i]),0,prim,num,bvh->scene,NULL,lazy_node);
            m_valid_intersection |= rays[i]->tfar < ray_ctx[i].org_rdir.w ? ((size_t)1 << i) : 0;
            ray_ctx[i].org_rdir.w = rays[i]->tfar;
          }

          /*! pop next node */
          STAT3(normal.trav_stack_pop,1,1,1);                          
          // todo: directly assign to *_next
          stackPtr--;
          cur = NodeRef(stackPtr->ptr);
          m_trav_active = stackPtr->mask;
          assert(m_trav_active);
        } // traversal + intersection

        //exit(0);
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
      }
    }
    
    
#else
    
    
    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {
      __aligned(64) RayContext ray_ctx[MAX_RAYS_PER_OCTANT];
      __aligned(64) Precalculations pre[MAX_RAYS_PER_OCTANT]; 
      __aligned(64) StackItemMask  stack[stackSizeSingle];  //!< stack of nodes 

      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** __restrict__ rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;
        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays == 64 ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;
        //STAT3(normal.trav_hit_boxes[__popcnt(m_active)],1,1,1);
        assert(m_active);
        for (size_t i=0;i<numOctantRays;i++)
        {
#if defined(__AVX512F__)
          vfloat<K> org(vfloat4(rays[i]->org));
          vfloat<K> dir(vfloat4(rays[i]->dir));
          vfloat<K> rdir       = select(0x7777,rcp_safe(dir),rays[i]->tnear);
          vfloat<K> org_rdir   = select(0x7777,org * rdir,rays[i]->tfar);
          vfloat<K> res = select(0xf,rdir,org_rdir);
          vfloat8 r = extractf256bit(res);
          *(vfloat8*)&ray_ctx[i] = r;          
#else
          Vec3fa &org = rays[i]->org;
          Vec3fa &dir = rays[i]->dir;
          ray_ctx[i].rdir       = rcp_safe(dir);
          ray_ctx[i].org_rdir   = org * ray_ctx[i].rdir;
          ray_ctx[i].rdir.w     = rays[i]->tnear;
          ray_ctx[i].org_rdir.w = rays[i]->tfar;
#endif

        }       

        StackItemMask* stackPtr = stack + 2;    //!< current stack pointer
        stack[0].ptr  = BVH::invalidNode;
        stack[0].mask = (size_t)-1;
        stack[1].ptr  = bvh->root;
        stack[1].mask = m_active;

        assert(__popcnt(m_active) <= MAX_RAYS_PER_OCTANT);
 
#if defined(__AVX512F__)
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);
 

        const vint<K> permX = select(vfloat<K>(ray_ctx[0].rdir.x) >= 0.0f,id,id2);
        const vint<K> permY = select(vfloat<K>(ray_ctx[0].rdir.y) >= 0.0f,id,id2);
        const vint<K> permZ = select(vfloat<K>(ray_ctx[0].rdir.z) >= 0.0f,id,id2);
        const vlong<K/2> one((size_t)1);
        // const vbool<K> maskX = ray_ctx[0].rdir.x >= 0.0f ? 0xf0 : 0x0f;
        // const vbool<K> maskY = ray_ctx[0].rdir.y >= 0.0f ? 0xf0 : 0x0f;
        // const vbool<K> maskZ = ray_ctx[0].rdir.z >= 0.0f ? 0xf0 : 0x0f;

        while (1) pop:
        {
          /*! pop next node */
          STAT3(normal.trav_stack_pop,1,1,1);                          
          //if (unlikely(stackPtr == stack)) break;
          stackPtr--;
          size_t m_trav_active = stackPtr->mask;
          size_t util = __popcnt(m_trav_active);
          NodeRef cur = NodeRef(stackPtr->ptr);
          assert(m_trav_active);

          const vfloat<K> inf(pos_inf);

#if 0
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
          {
            while (likely(!cur.isLeaf()))
            {
              const Node* __restrict__ const node = cur.node();
              STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

              const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),permX);
              const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),permY);
              const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),permZ);

              vfloat<K> dist(inf);
              vlong<K/2>   maskK(zero);
              size_t bits = m_trav_active;
              do
              {            
                STAT3(normal.trav_nodes,1,1,1);                          
                const size_t i = __bscf(bits);
                RayContext &ray = ray_ctx[i];
                const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
                const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
                const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
                const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
                const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
                const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));                
                //const vint<K> bitmask  = vint<K>((size_t)1 << i);
                const vlong<K/2> bitmask  = one << vlong<K/2>(i);
                dist   = select(vmask,min(tNear,dist),dist);
                maskK = mask_or((vboold8)vmask,maskK,maskK,bitmask);
              } while(bits);              


              const vbool<K> vmask = lt(0xff,dist,inf);

              if (unlikely(none(vmask))) goto pop;

              BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (size_t*)&maskK, stackPtr);
            }
          }
          DBG("INTERSECTION");

          if (unlikely(cur == BVH::invalidNode)) break;

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

          size_t lazy_node = 0;
          size_t bits = m_trav_active;
          size_t m_valid_intersection = 0;
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) 
          {
            PrimitiveIntersector::intersect(pre[i],*(rays[i]),0,prim,num,bvh->scene,NULL,lazy_node);
            m_valid_intersection |= rays[i]->tfar < ray_ctx[i].org_rdir.w ? ((size_t)1 << i) : 0;
            ray_ctx[i].org_rdir.w = rays[i]->tfar;
          }

        } // traversal + intersection
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

#else

        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        const size_t nearX = (rays[0]->dir.x < 0.0f) ? 1*sizeof(vfloat<N>) : 0*sizeof(vfloat<N>);
        const size_t nearY = (rays[0]->dir.y < 0.0f) ? 3*sizeof(vfloat<N>) : 2*sizeof(vfloat<N>);
        const size_t nearZ = (rays[0]->dir.z < 0.0f) ? 5*sizeof(vfloat<N>) : 4*sizeof(vfloat<N>);

        const size_t farX  = nearX ^ sizeof(vfloat<N>);
        const size_t farY  = nearY ^ sizeof(vfloat<N>);
        const size_t farZ  = nearZ ^ sizeof(vfloat<N>);

        while (1) pop:
        {
          /*! pop next node */
          STAT3(normal.trav_stack_pop,1,1,1);                          
          //if (unlikely(stackPtr == stack)) break;
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
          size_t m_trav_active = stackPtr->mask;
          assert(m_trav_active);          

          const vfloat<K> inf(pos_inf);
          while (likely(!cur.isLeaf()))
          {
            const Node* __restrict__ const node = cur.node();
            STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

            const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearX));
            const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearY));
            const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+nearZ));
            const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farX));
            const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farY));
            const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+farZ));

            vfloat<K> dist(inf);
            vint<K>   maskK(zero);
            assert(__popcnt((size_t)m_trav_active) <= MAX_RAYS_PER_OCTANT);

            size_t bits = m_trav_active;
            do
            {            
              STAT3(normal.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              RayContext &ray = ray_ctx[i];
              const vfloat<K> tNearX = msub(bminX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearY = msub(bminY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearZ = msub(bminZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tFarX  = msub(bmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tFarY  = msub(bmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tFarZ  = msub(bmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vint<K> bitmask  = vint<K>((unsigned int)1 << i);
#if defined(__AVX2__)
              const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ray.rdir.w)));
              const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(ray.org_rdir.w)));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              //maskK = maskK | (bitmask & vint<K>((__m256i)vmask));
              maskK = select(vmask,maskK | bitmask,maskK); 
#else
              const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = select(vmask,maskK | bitmask,maskK); 
#endif
            } while(bits);              

            const vbool<K> vmask = dist < inf;

            if (unlikely(none(vmask))) goto pop;

#if defined(__AVX2__)
            BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (unsigned int*)&maskK, stackPtr);
            assert(m_trav_active);
#else
            FATAL("not yet implemented");
#endif


          }
          DBG("INTERSECTION");

          if (unlikely(cur == BVH::invalidNode)) break;

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

          size_t lazy_node = 0;
          size_t bits = m_trav_active;
          size_t m_valid_intersection = 0;
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) 
          {
            PrimitiveIntersector::intersect(pre[i],*(rays[i]),0,prim,num,bvh->scene,NULL,lazy_node);
            m_valid_intersection |= rays[i]->tfar < ray_ctx[i].org_rdir.w ? ((size_t)1 << i) : 0;
            ray_ctx[i].org_rdir.w = rays[i]->tfar;
          }

        } // traversal + intersection

        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

#endif

      }
    }
#endif
    
    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {

      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        __aligned(64) RayContext ray_ctx[MAX_RAYS_PER_OCTANT];

        Precalculations pre[MAX_RAYS_PER_OCTANT]; 

        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays == 64 ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;
        for (size_t i=0;i<numOctantRays;i++)
        {
#if defined(__AVX512F__)
          vfloat<K> org(vfloat4(rays[i]->org));
          vfloat<K> dir(vfloat4(rays[i]->dir));
          vfloat<K> rdir       = select(0x7777,rcp_safe(dir),rays[i]->tnear);
          vfloat<K> org_rdir   = select(0x7777,org * rdir,rays[i]->tfar);
          vfloat<K> res = select(0xf,rdir,org_rdir);
          vfloat8 r = extractf256bit(res);
          *(vfloat8*)&ray_ctx[i] = r;          
#else
          Vec3fa &org = rays[i]->org;
          Vec3fa &dir = rays[i]->dir;
          ray_ctx[i].rdir       = rcp_safe(dir);
          ray_ctx[i].org_rdir   = org * ray_ctx[i].rdir;
          ray_ctx[i].rdir.w     = rays[i]->tnear;
          ray_ctx[i].org_rdir.w = rays[i]->tfar;
#endif

        }       


        StackItemMask  stack[stackSizeSingle];  //!< stack of nodes 
        StackItemMask* stackPtr = stack + 2;    //!< current stack pointer
        stack[0].ptr  = BVH::invalidNode;
        stack[0].mask = (size_t)-1;
        stack[1].ptr  = bvh->root;
        stack[1].mask = m_active;

#if defined(__AVX512F__)

        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);

        const vint<K> permX = select(vfloat<K>(ray_ctx[0].rdir.x) >= 0.0f,id,id2);
        const vint<K> permY = select(vfloat<K>(ray_ctx[0].rdir.y) >= 0.0f,id,id2);
        const vint<K> permZ = select(vfloat<K>(ray_ctx[0].rdir.z) >= 0.0f,id,id2);
        const vint<K> one(1);
#else 

        const size_t nearX = (rays[0]->dir.x < 0.0f) ? 1*sizeof(vfloat<N>) : 0*sizeof(vfloat<N>);
        const size_t nearY = (rays[0]->dir.y < 0.0f) ? 3*sizeof(vfloat<N>) : 2*sizeof(vfloat<N>);
        const size_t nearZ = (rays[0]->dir.z < 0.0f) ? 5*sizeof(vfloat<N>) : 4*sizeof(vfloat<N>);

        const size_t farX  = nearX ^ sizeof(vfloat<N>);
        const size_t farY  = nearY ^ sizeof(vfloat<N>);
        const size_t farZ  = nearZ ^ sizeof(vfloat<N>);

#endif

        while (1) pop:
        {
          /*! pop next node */
          STAT3(shadow.trav_stack_pop,1,1,1);                          
          //if (unlikely(stackPtr == stack)) break;
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
          size_t m_trav_active = stackPtr->mask & m_active;
          if (unlikely(!m_trav_active)) continue;

          assert(m_trav_active);

#if defined(__AVX512F__)
          const vfloat<K> inf(pos_inf);
          while (likely(!cur.isLeaf()))
          {
            const Node* __restrict__ const node = cur.node();
            STAT3(shadow.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

            const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),permX);
            const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),permY);
            const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),permZ);

            vfloat<K> dist(inf);
            vint<K>   maskK(zero);

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
              const vint<K> bitmask     = one << vint<K>(i);
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = mask_or(vmask,maskK,maskK,bitmask);
            } while(bits);          
            const vbool<K> vmask = lt(0xff,dist,inf);
            if (unlikely(none(vmask))) goto pop;

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr); 

          }

#else
          const vfloat<K> inf(pos_inf);
          while (likely(!cur.isLeaf()))
          {
            const Node* __restrict__ const node = cur.node();
            STAT3(shadow.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

            const vfloat<K> bminX = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+nearX));
            const vfloat<K> bminY = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+nearY));
            const vfloat<K> bminZ = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+nearZ));
            const vfloat<K> bmaxX = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+farX));
            const vfloat<K> bmaxY = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+farY));
            const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<K>*)((const char*)&node->lower_x+farZ));

            vfloat<K> dist(inf);
            vint<K>   maskK(zero);

            size_t bits = m_trav_active;
            do
            {            
              STAT3(shadow.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              RayContext &ray = ray_ctx[i];
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
              maskK = select(vmask,maskK | bitmask,maskK); 
              //maskK = maskK | (bitmask & vint<K>((__m256i)vmask));
#else
              const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask   = tNear <= tFar;
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = select(vmask,maskK | bitmask,maskK); 
#endif
            } while(bits);          
            const vbool<K> vmask = dist < inf;
            if (unlikely(none(vmask))) goto pop;

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr); 

          }
#endif
          DBG("INTERSECTION");

          if (unlikely(cur == BVH::invalidNode)) break;

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(shadow.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          //STAT3(shadow.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

          size_t lazy_node = 0;
          size_t bits = m_trav_active & m_active;
          size_t m_valid_intersection = 0;
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) 
            if (PrimitiveIntersector::occluded(pre[i],*(rays[i]),0,prim,num,bvh->scene,NULL,lazy_node))
            {
              m_active &= ~((size_t)1 << i);
              rays[i]->geomID = 0;
            }

          if (unlikely(m_active == 0)) break;
        } // traversal + intersection        
      }      
    }

#if defined(__AVX512F__)
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersector, BVHNStreamIntersector<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersector, BVHNStreamIntersector<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 16 COMMA false> > >);

#else
    DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersector, BVHNStreamIntersector<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA false> > >);
    DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersector, BVHNStreamIntersector<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA false> > >);
#endif


#endif

  }
}


// ===================================================================================================================================================================
// ===================================================================================================================================================================
// ===================================================================================================================================================================

    
