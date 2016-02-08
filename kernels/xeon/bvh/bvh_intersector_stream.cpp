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
// PRINT(x)

#define BLANK  
// DBG(std::cout << std::endl)

// todo: permute = 2 x broadcast + mask

namespace embree
{
  namespace isa
  {

#if defined(__AVX__)

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

        size_t m_active = 0;
        for (size_t i=0;i<numOctantRays;i++)
        {
          Vec3fa &org = rays[i]->org;
          Vec3fa &dir = rays[i]->dir;
          ray_ctx[i].rdir       = rcp_safe(dir);
          ray_ctx[i].org_rdir   = org * ray_ctx[i].rdir;
          ray_ctx[i].rdir.w     = rays[i]->tnear;
          ray_ctx[i].org_rdir.w = rays[i]->tfar;
          m_active |= rays[i]->tnear <= rays[i]->tfar ? ((size_t)1 << i) : 0; //todo: optimize
          pre[i] = Precalculations(*rays[i],bvh);
        }       

        StackItemMask* stackPtr = stack + 2;    //!< current stack pointer
        StackItemMask* stackEnd = stack + stackSizeSingle;
        stack[0].ptr  = BVH::invalidNode;
        stack[0].mask = (size_t)-1;
        stack[1].ptr  = bvh->root;
        stack[1].mask = m_active;
 
#if defined(__AVX512F__)
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);
 

        const vint<K> permX = select(vfloat<K>(ray_ctx[0].rdir.x) >= 0.0f,id,id2);
        const vint<K> permY = select(vfloat<K>(ray_ctx[0].rdir.y) >= 0.0f,id,id2);
        const vint<K> permZ = select(vfloat<K>(ray_ctx[0].rdir.z) >= 0.0f,id,id2);

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

#if 1
          if (likely(util == 1))
          {
            const size_t i = __bsf(m_trav_active);
            const vint<K> maskK(m_trav_active);
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

              BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, tNear, (unsigned int*)&maskK, stackPtr, stackEnd);
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
              vint<K>   maskK(zero);
              size_t bits = m_trav_active;
              do
              {            
                STAT3(normal.trav_nodes,1,1,1);                          
                const size_t i = __bscf(bits);
                RayContext &ray = ray_ctx[i];
                //if (unlikely(bits == 0))
                {
                  const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
                  const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
                  const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
                  const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
                  const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
                  const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));                
                  const vint<K> bitmask  = vint<K>((int)1 << i);
                  dist   = select(vmask,min(tNear,dist),dist);
                  maskK = mask_or(vmask,maskK,maskK,bitmask);
                  //break;
                }
                // else
                // {
                //   const size_t j = __bscf(bits);
                //   RayContext &ray1 = ray_ctx[j];
                //   const vfloat<K> tNearFarX0 = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
                //   const vfloat<K> tNearFarY0 = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
                //   const vfloat<K> tNearFarZ0 = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);

                //   const vfloat<K> tNearFarX1 = msub(bminmaxX, ray1.rdir.x, ray1.org_rdir.x);
                //   const vfloat<K> tNearFarY1 = msub(bminmaxY, ray1.rdir.y, ray1.org_rdir.y);
                //   const vfloat<K> tNearFarZ1 = msub(bminmaxZ, ray1.rdir.z, ray1.org_rdir.z);

                //   const vfloat<K> tNear0     = max(tNearFarX0,tNearFarY0,tNearFarZ0,vfloat<K>(ray.rdir.w));
                //   const vfloat<K> tFar0      = min(tNearFarX0,tNearFarY0,tNearFarZ0,vfloat<K>(ray.org_rdir.w));

                //   const vfloat<K> tNear1     = max(tNearFarX1,tNearFarY1,tNearFarZ1,vfloat<K>(ray1.rdir.w));
                //   const vfloat<K> tFar1      = min(tNearFarX1,tNearFarY1,tNearFarZ1,vfloat<K>(ray1.org_rdir.w));

                //   const vbool<K> vmask0      = le(tNear0,align_shift_right<8>(tFar0,tFar0));                
                //   const vint<K> bitmask0  = vint<K>((int)1 << i);
                //   dist   = select(vmask0,min(tNear0,dist),dist);
                //   maskK = mask_or(vmask0,maskK,maskK,bitmask0); 

                //   const vbool<K> vmask1      = le(tNear1,align_shift_right<8>(tFar1,tFar1));                
                //   const vint<K> bitmask1  = vint<K>((int)1 << j);
                //   dist   = select(vmask1,min(tNear1,dist),dist);
                //   maskK = mask_or(vmask1,maskK,maskK,bitmask1);                   
                // }

              } while(bits);              


              const vbool<K> vmask = lt(0xff,dist,inf);

              if (unlikely(none(vmask))) goto pop;

              BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (unsigned int*)&maskK, stackPtr, stackEnd);
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
            //size_t mask64[K]; for (size_t i=0;i<8;i++) mask64[i] = 0;

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
              const vbool<K> vmask   = asInt(tNear) <= asInt(tFar);
              dist   = select(vmask,mini(tNear,dist),dist);
              //maskK = maskK | (bitmask & vint<K>((__m256i)vmask));
              maskK = select(vmask,maskK | bitmask,maskK); 
              //size_t m64 = movemask(vmask);
              //for (size_t j=__bsf(m64); m64!=0; m64=__btc(m64,j), j=__bsf(m64)) mask64[j] |= (size_t)1 << i;
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
            BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (unsigned int*)&maskK, stackPtr, stackEnd);
            //BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, mask64, stackPtr, stackEnd);
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

    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, size_t flags)
    {

      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        __aligned(64) RayContext ray_ctx[MAX_RAYS_PER_OCTANT];

        Precalculations pre[MAX_RAYS_PER_OCTANT]; 

        size_t m_active = 0;
        for (size_t i=0;i<numOctantRays;i++)
        {
          Vec3fa &org = rays[i]->org;
          Vec3fa &dir = rays[i]->dir;
          ray_ctx[i].rdir       = rcp_safe(dir);
          ray_ctx[i].org_rdir   = org * ray_ctx[i].rdir;
          ray_ctx[i].rdir.w     = rays[i]->tnear;
          ray_ctx[i].org_rdir.w = rays[i]->tfar;
          m_active |= rays[i]->tnear <= rays[i]->tfar ? ((size_t)1 << i) : 0; //todo: optimize
          pre[i] = Precalculations(*rays[i],bvh);
        }       


        StackItemMask  stack[stackSizeSingle];  //!< stack of nodes 
        StackItemMask* stackPtr = stack + 2;    //!< current stack pointer
        StackItemMask* stackEnd = stack + stackSizeSingle;
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
              RayContext &ray = ray_ctx[i];
              const vfloat<K> tNearFarX = msub(bminmaxX, ray.rdir.x, ray.org_rdir.x);
              const vfloat<K> tNearFarY = msub(bminmaxY, ray.rdir.y, ray.org_rdir.y);
              const vfloat<K> tNearFarZ = msub(bminmaxZ, ray.rdir.z, ray.org_rdir.z);
              const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.rdir.w));
              const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray.org_rdir.w));
              const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));                
              const vint<K> bitmask  = vint<K>((int)1 << i);
              dist   = select(vmask,min(tNear,dist),dist);
              maskK = mask_or(vmask,maskK,maskK,bitmask);
            } while(bits);          
            const vbool<K> vmask = dist < inf;
            if (unlikely(none(vmask))) goto pop;

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr,stackEnd); 

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
              const vbool<K> vmask   = asInt(tNear) <= asInt(tFar);
              dist   = select(vmask,mini(tNear,dist),dist);
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

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr,stackEnd); 

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

#if 0
/* two rays traversal + refill */
template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
void BVHNIntersectorKHybrid2<N,K,types,robust,PrimitiveIntersectorK,single>::intersect(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
{
  __aligned(64) StackItemT<NodeRef>  stacks[2][stackSizeSingle];  //!< stack of nodes 

  /* filter out invalid rays */
  vbool<K> valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
  valid0 &= ray.valid();
#endif

  /* verify correct input */
  assert(all(valid0,ray.valid()));
  assert(all(valid0,ray.tnear >= 0.0f));
  assert(!(types & BVH_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));

  /* load ray */
  Vec3vfK &ray_org = ray.org;
  Vec3vfK &ray_dir = ray.dir;
  const Vec3vfK rdir = rcp_safe(ray_dir);
  const Vec3vfK org(ray_org), org_rdir = org * rdir;
  const vfloat<K> ray_tnear = select(valid0,ray.tnear,vfloat<K>(pos_inf));
  vfloat<K> ray_tfar  = select(valid0,ray.tfar ,vfloat<K>(neg_inf));
  const vfloat<K> inf = vfloat<K>(pos_inf);
  Precalculations pre(valid0,ray);

  Vec3viK nearXYZ;
  nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
  nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
  nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

  /* sentinel */
  for (size_t i=0;i<2;i++)
  {
    stacks[i]->ptr  = BVH::invalidNode;
    stacks[i]->dist = neg_inf;
  }

  vbool<K> m_active = ray_tnear <= ray_tfar;
  while(m_active)
  {
    BLANK;
    DBG("NEW CHUNK");
    DBG(m_active);
    const size_t first = __bsf(m_active);
    vbool<K> m_samesign =                   \
      (nearXYZ.x[first] == nearXYZ.x) &
      (nearXYZ.y[first] == nearXYZ.y) &
      (nearXYZ.z[first] == nearXYZ.z);
    assert(m_samesign);

    m_active &=~m_samesign;
    size_t m_active_chunk = movemask(m_samesign);
    DBG(m_samesign);


    const vint<K> id( step );
    const vint<K> id2 = align_shift_right<8>(id,id);
 
    const vint<K> permX = select(vfloat<K>(rdir.x[first]) >= 0.0f,id,id2);
    const vint<K> permY = select(vfloat<K>(rdir.y[first]) >= 0.0f,id,id2);
    const vint<K> permZ = select(vfloat<K>(rdir.z[first]) >= 0.0f,id,id2);

    struct Context {
      size_t rayID;
      NodeRef cur;
      StackItemT<NodeRef>* stackPtr;
      StackItemT<NodeRef>* stack;
    } context[2];

    const size_t ID0 = __bscf(m_active_chunk);
    const size_t ID1 = m_active_chunk ? __bscf(m_active_chunk) : (size_t)-1;

    context[0].rayID    = ID0;
    context[0].cur      = bvh->root;
    context[0].stackPtr = stacks[0] + 1;
    context[0].stack    = stacks[0];
    context[1].rayID    = ID1;
    context[1].cur      = bvh->root;
    context[1].stackPtr = stacks[1] + 1;
    context[1].stack    = stacks[1];

    if (unlikely(ID1 == (size_t)-1)) context[1].cur = BVH::invalidNode;
    DBG(ID0);
    DBG(ID1);
    DBG(context[0].stack);
    DBG(context[1].stack);

    size_t contextID = 0;
        
    while(1)        
    {
      BLANK;
      DBG("START TRAVERSAL");
      DBG(contextID);

      /* test whether we have 1 or 2 active rays */
      if (unlikely(context[1-contextID].cur == BVH::invalidNode))
      {
        DBG("SINGLE RAY TRAVERSAL");

        /* single ray path */
        assert(context[1-contextID].cur == BVH::invalidNode);
        const size_t rayID            = context[contextID].rayID;
        NodeRef cur                   = context[contextID].cur;
        StackItemT<NodeRef>* stackPtr = context[contextID].stackPtr;
        const vfloat<K> rdir_x         = rdir.x[rayID];
        const vfloat<K> rdir_y         = rdir.y[rayID];
        const vfloat<K> rdir_z         = rdir.z[rayID];
        const vfloat<K> org_rdir_x     = org_rdir.x[rayID];
        const vfloat<K> org_rdir_y     = org_rdir.y[rayID];
        const vfloat<K> org_rdir_z     = org_rdir.z[rayID];
        const vfloat<K> tnear          = ray_tnear[rayID];
        const vfloat<K> tfar           = ray_tfar[rayID];

        while(1)
        {
          if (unlikely(cur.isLeaf())) break;
          //STAT3(normal.trav_hit_boxes[1],1,1,1);                          
          STAT3(normal.trav_nodes,1,1,1);                          
          const Node* __restrict__ const node = cur.node();
          const vfloat<K> bminmaxX  = permute(vfloat<K>::load((float*)&node->lower_x),permX);
          const vfloat<K> bminmaxY  = permute(vfloat<K>::load((float*)&node->lower_y),permY);
          const vfloat<K> bminmaxZ  = permute(vfloat<K>::load((float*)&node->lower_z),permZ);
          const vfloat<K> tNearFarX = msub(bminmaxX, rdir_x, org_rdir_x);
          const vfloat<K> tNearFarY = msub(bminmaxY, rdir_y, org_rdir_y);
          const vfloat<K> tNearFarZ = msub(bminmaxZ, rdir_z, org_rdir_z);
          const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,tnear);
          const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,tfar);
          const vbool<K> vmask      = le(vbool<K>(0xff),tNear,align_shift_right<8>(tFar,tFar));
          if (unlikely(none(vmask)))
          {
            stackPtr--;
            cur = stackPtr->ptr;
          }
          else 
          {
            BVHNNodeTraverser1<8,16,types>::traverseClosestHit(cur,vmask,tNear,stackPtr,context[contextID].stack+stackSizeSingle);                
          }
        }
        context[contextID].stackPtr = stackPtr;
        context[contextID].cur      = cur;
      }
      else
      {
        DBG("TWO RAYS TRAVERSAL");

        /* two rays path */
        StackItemT<NodeRef>* stackPtr0 = context[0].stackPtr;
        const size_t rayID0            = context[0].rayID;
        NodeRef cur0                   = context[0].cur;

        StackItemT<NodeRef>* stackPtr1 = context[1].stackPtr;
        const size_t rayID1            = context[1].rayID;
        NodeRef cur1                   = context[1].cur;

        DBG(cur0);
        DBG(cur1);

        while(1)
        {
          DBG(cur0);
          DBG(cur1);

          contextID = 0;
          DBG(contextID);
          if (unlikely(cur0.isLeaf())) { break; }
          contextID = 1;
          DBG(contextID);
          if (unlikely(cur1.isLeaf())) { break; }
          DBG("NODE INTERSECTION");

          //STAT3(normal.trav_hit_boxes[2],1,1,1);                          

          STAT3(normal.trav_nodes,1,1,1);                          
          const Node* __restrict__ const node0 = cur0.node();
          const vfloat<K> bminmaxX0 = permute(vfloat<K>::load((float*)&node0->lower_x),permX);
          const vfloat<K> bminmaxY0 = permute(vfloat<K>::load((float*)&node0->lower_y),permY);
          const vfloat<K> bminmaxZ0 = permute(vfloat<K>::load((float*)&node0->lower_z),permZ);
          const vfloat<K> tNearFarX0 = msub(bminmaxX0, rdir.x[rayID0], org_rdir.x[rayID0]);
          const vfloat<K> tNearFarY0 = msub(bminmaxY0, rdir.y[rayID0], org_rdir.y[rayID0]);
          const vfloat<K> tNearFarZ0 = msub(bminmaxZ0, rdir.z[rayID0], org_rdir.z[rayID0]);
          const vfloat<K> tNear0     = max(tNearFarX0,tNearFarY0,tNearFarZ0,vfloat<K>(ray_tnear[rayID0]));
          const vfloat<K> tFar0      = min(tNearFarX0,tNearFarY0,tNearFarZ0,vfloat<K>(ray_tfar[rayID0]));
          const vbool<K> vmask0      = le(vbool<K>(0xff),tNear0,align_shift_right<8>(tFar0,tFar0));

          DBG(vmask0);

          if (unlikely(none(vmask0)))
          {
            DBG("STACK POP");
            stackPtr0--;
            cur0 = stackPtr0->ptr;
            DBG(cur0);
          }
          else 
          {
            DBG("SORT");
            DBG(stackPtr0-context[0].stack);
            BVHNNodeTraverser1<8,16,types>::traverseClosestHit(cur0,vmask0,tNear0,stackPtr0,context[0].stack+stackSizeSingle);                
          }

          STAT3(normal.trav_nodes,1,1,1);                          
          const Node* __restrict__ const node1 = cur1.node();
          const vfloat<K> bminmaxX1 = permute(vfloat<K>::load((float*)&node1->lower_x),permX);
          const vfloat<K> bminmaxY1 = permute(vfloat<K>::load((float*)&node1->lower_y),permY);
          const vfloat<K> bminmaxZ1 = permute(vfloat<K>::load((float*)&node1->lower_z),permZ);
          const vfloat<K> tNearFarX1 = msub(bminmaxX1, rdir.x[rayID1], org_rdir.x[rayID1]);
          const vfloat<K> tNearFarY1 = msub(bminmaxY1, rdir.y[rayID1], org_rdir.y[rayID1]);
          const vfloat<K> tNearFarZ1 = msub(bminmaxZ1, rdir.z[rayID1], org_rdir.z[rayID1]);
          const vfloat<K> tNear1     = max(tNearFarX1,tNearFarY1,tNearFarZ1,vfloat<K>(ray_tnear[rayID1]));
          const vfloat<K> tFar1      = min(tNearFarX1,tNearFarY1,tNearFarZ1,vfloat<K>(ray_tfar[rayID1]));
          const vbool<K> vmask1      = le(vbool<K>(0xff),tNear1,align_shift_right<8>(tFar1,tFar1));


          DBG(vmask1);

          if (unlikely(none(vmask1)))
          {
            DBG("STACK POP");
            stackPtr1--;
            cur1 = stackPtr1->ptr;
            DBG(cur1);
          }
          else 
          {
            DBG("SORT");
            DBG(stackPtr1-context[1].stack);
            BVHNNodeTraverser1<8,16,types>::traverseClosestHit(cur1,vmask1,tNear1,stackPtr1,context[1].stack+stackSizeSingle);                
          }              
        } /* traversal */           

        context[0].stackPtr = stackPtr0;
        context[0].cur      = cur0;
        context[1].stackPtr = stackPtr1;
        context[1].cur      = cur1;
        DBG(context[0].cur);
        DBG(context[1].cur);

      } /* two rays */
      BLANK;
      DBG("PRE-LEAF -> termination?");
      DBG(contextID);
      DBG(context[contextID].cur);
      DBG(context[1-contextID].cur);
          
      if (unlikely(context[contextID].cur == BVH::invalidNode))
      {
        DBG("ACTIVE RAY TERMINATED");
        /* all rays are terminated */
        if (unlikely(context[1-contextID].cur == BVH::invalidNode && m_active_chunk == 0)) break;

        DBG(m_active_chunk);

        /* can we refill the context with a new ray */
        if (likely(m_active_chunk))
        {              
          STAT3(normal.trav_hit_boxes[contextID],1,1,1); 

          context[contextID].rayID    = __bscf(m_active_chunk);
          context[contextID].cur      = bvh->root;
          context[contextID].stackPtr = context[contextID].stack + 1;              
          DBG(context[contextID].rayID);

        }
        else
          /* continue with the remaining valid context */
          contextID = 1 - contextID;

        DBG(contextID);

        continue;
      }

      DBG("LEAF");

      /*! this is a leaf node */
      STAT3(normal.trav_leaves, 1, 1, 1);
      NodeRef            cur = context[contextID].cur;
      const size_t rayID     = context[contextID].rayID;
      size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

      size_t lazy_node = 0;
      PrimitiveIntersectorK::intersect(pre, ray, rayID, prim, num, bvh->scene, lazy_node);

      /* stack compaction */
      if (unlikely(ray.tfar[rayID] < ray_tfar[rayID]))
      {
        DBG("STACK COMPACTION");
        StackItemT<NodeRef>* stackPtr = context[contextID].stackPtr;
        StackItemT<NodeRef>* stack    = context[contextID].stack;
        StackItemT<NodeRef>* left     = stack + 1;
        for (StackItemT<NodeRef>* right = stack+1; right<stackPtr; right++) 
        {
          /* optimize */
          if (unlikely(*(float*)&right->dist < ray.tfar[rayID]))
          {
            *(vint4*)left = *(vint4*)right; 
            left++;
          }
        }
        context[contextID].stackPtr = left;            
      }

      ray_tfar = min(ray_tfar,ray.tfar);
      /* update cur from stack */
      DBG("STACK POP");
      context[contextID].stackPtr--;
      context[contextID].cur = context[contextID].stackPtr->ptr;

      DBG(context[contextID].cur);
      DBG(context[contextID].stack);
      DBG(context[contextID].stackPtr);
      DBG(context[1-contextID].cur);

    } 
            
        
  }
    
}
#endif


// ===================================================================================================================================================================
// ===================================================================================================================================================================
// ===================================================================================================================================================================

    
