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

#include "bvh_intersector_stream.h"
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

#include "../common/scene.h"
#include <bitset>

namespace embree
{
  namespace isa
  {
    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    __forceinline void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersect(Accel::Intersectors* __restrict__ This,
                                                                                                       RayK<K>** inputPackets, size_t numOctantRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      __aligned(64) StackItemMaskCoherent stack[stackSizeSingle];  //!< stack of nodes
      assert(numOctantRays <= MAX_INTERNAL_STREAM_SIZE);

      __aligned(64) Packet<K,robust> packet[MAX_INTERNAL_STREAM_SIZE/K];
      __aligned(64) Frustum<N,Nx,K,robust> frusta;

      bool commonOctant = true;
      const size_t m_active = initPacketsAndFrusta<false>(inputPackets, numOctantRays, packet, frusta, commonOctant);
      if (unlikely(m_active == 0)) return;

      /* case of non-common origin */
      if (unlikely(!commonOctant))
      {
        const size_t numPackets = (numOctantRays+K-1)/K; 
        for (size_t i = 0; i < numPackets; i++)
          This->intersect(inputPackets[i]->tnear <= inputPackets[i]->tfar,*inputPackets[i],context);
        return;
      }

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      StackItemMaskCoherent* stackPtr = stack + 1;

      while (1) pop:
      {
        if (unlikely(stackPtr == stack)) break;

        STAT3(normal.trav_stack_pop,1,1,1);
        stackPtr--;
        /*! pop next node */
        NodeRef cur = NodeRef(stackPtr->child);
        size_t m_trav_active = stackPtr->mask;
        assert(m_trav_active);
        NodeRef parent = stackPtr->parent;

        while (1)
        {
          if (unlikely(cur.isLeaf())) break;
          const AlignedNode* __restrict__ const node = cur.alignedNode();
          parent = cur;

          __aligned(64) size_t maskK[N];
          for (size_t i = 0; i < N; i++) maskK[i] = m_trav_active;
          vfloat<Nx> dist;
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packet, node, frusta, maskK, dist);
          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserStreamHitCoherent<N, Nx, types>::traverseClosestHit(cur, m_trav_active, vbool<Nx>((int)m_node_hit), dist, (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /* non-root and leaf => full culling test for all rays */
        if (unlikely(parent != 0 && cur.isLeaf()))
        {
          const AlignedNode* __restrict__ const node = parent.alignedNode();
          size_t b = 0xff;
          for (size_t i=0;i<N;i++)
            if (node->child(i) == cur) 
            {
              b = i;
              break;
            }
          assert(b < N);
          assert(cur == node->child(b));
          m_trav_active = intersectAlignedNodePacket(packet, node, b, frusta.nf, m_trav_active);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        size_t bits = m_trav_active;

        /*! intersect stream of rays with all primitives */
        size_t lazy_node = 0;
#if defined(__SSE4_2__)
        STAT_USER(1,(__popcnt(bits)+K-1)/K*4);
#endif
        while(bits)
        {
          size_t i = __bsf(bits) / K;
          const size_t m_isec = ((((size_t)1 << K)-1) << (i*K));
          assert(m_isec & bits);
          bits &= ~m_isec;

          Packet<K,robust> &p = packet[i];
          vbool<K> m_valid = p.min_dist <= p.max_dist;
          PrimitiveIntersector::intersectK(m_valid, *inputPackets[i], context, prim, num, lazy_node);
          p.max_dist = min(p.max_dist, inputPackets[i]->tfar);
        };

      } // traversal + intersection
    }

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    __forceinline void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occluded(Accel::Intersectors* __restrict__ This,
                                                                                                      RayK<K>** inputPackets, size_t numOctantRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      assert(numOctantRays <= MAX_INTERNAL_STREAM_SIZE);

      /* inactive rays should have been filtered out before */
#if 1
      __aligned(64) Packet<K,false> packet[MAX_INTERNAL_STREAM_SIZE/K];

      assert(numOctantRays <= 32);
      const size_t numPackets = (numOctantRays+K-1)/K;
      size_t m_active = 0;
      for (size_t i = 0; i < numPackets; i++)
      {
        const vfloat<K> tnear  = inputPackets[i]->tnear;
        const vfloat<K> tfar   = inputPackets[i]->tfar;
        vbool<K> m_valid = (tnear <= tfar) & (tnear >= 0.0f);
        m_active |= (size_t)movemask(m_valid) << (K*i);
        const Vec3vf<K>& org     = inputPackets[i]->org;
        const Vec3vf<K>& dir     = inputPackets[i]->dir;
        vfloat<K> packet_min_dist = max(tnear, 0.0f);
        vfloat<K> packet_max_dist = select(m_valid, tfar, neg_inf);
        new (&packet[i]) Packet<K,false>(org,dir,packet_min_dist,packet_max_dist);
      }

      const int shiftTable[32] = { 
        (int)1 << 0, (int)1 << 1, (int)1 << 2, (int)1 << 3, (int)1 << 4, (int)1 << 5, (int)1 << 6, (int)1 << 7,  
        (int)1 << 8, (int)1 << 9, (int)1 << 10, (int)1 << 11, (int)1 << 12, (int)1 << 13, (int)1 << 14, (int)1 << 15,  
        (int)1 << 16, (int)1 << 17, (int)1 << 18, (int)1 << 19, (int)1 << 20, (int)1 << 21, (int)1 << 22, (int)1 << 23,  
        (int)1 << 24, (int)1 << 25, (int)1 << 26, (int)1 << 27, (int)1 << 28, (int)1 << 29, (int)1 << 30, (int)1 << 31
      };

      StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes
      StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
      stack[0].ptr = bvh->root;
      stack[0].dist = m_active;

      size_t terminated = ~m_active;

      while (1) pop:
      {
        if (unlikely(stackPtr == stack)) break;
        STAT3(shadow.trav_stack_pop,1,1,1);
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        size_t cur_mask = (size_t)stackPtr->dist & (~terminated);
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
          //STAT3(shadow.trav_hit_boxes[__popcnt(cur_mask)],1,1,1);
#endif

          assert(bits);
          vint<Nx> vmask(zero);
          do
          {   
            STAT3(shadow.trav_nodes,1,1,1);

            const size_t rayID = __bscf(bits);
            assert(rayID < MAX_INTERNAL_STREAM_SIZE);
            assert(rayID /K < numPackets);
            Packet<K,false> &p = packet[rayID /K];
            const size_t i = rayID % K;
            const vint<Nx> bitmask(shiftTable[rayID]);

            const vfloat<Nx> tNearX = msub(bminX, p.rdir.x[i], p.org_rdir.x[i]);
            const vfloat<Nx> tNearY = msub(bminY, p.rdir.y[i], p.org_rdir.y[i]);
            const vfloat<Nx> tNearZ = msub(bminZ, p.rdir.z[i], p.org_rdir.z[i]);
            const vfloat<Nx> tFarX  = msub(bmaxX, p.rdir.x[i], p.org_rdir.x[i]);
            const vfloat<Nx> tFarY  = msub(bmaxY, p.rdir.y[i], p.org_rdir.y[i]);
            const vfloat<Nx> tFarZ  = msub(bmaxZ, p.rdir.z[i], p.org_rdir.z[i]);
 
            const vfloat<Nx> tNear  = maxi(mini(tNearX,tFarX), mini(tNearY,tFarY), mini(tNearZ,tFarZ), vfloat<Nx>(p.min_dist[i]));
            const vfloat<Nx> tFar   = mini(maxi(tNearX,tFarX), maxi(tNearY,tFarY), maxi(tNearZ,tFarZ), vfloat<Nx>(p.max_dist[i]));
            const vbool<Nx> hit_mask   = tNear <= tFar;
#if defined(__AVX2__)
            vmask = vmask | (bitmask & vint<Nx>(hit_mask));
#else
            vmask = select(hit_mask, vmask | bitmask, vmask);
#endif
          } while(bits);     
          const vbool<Nx> valid_children = bminX <= bmaxX;

          size_t mask = movemask( (vmask != vint<Nx>(zero)) & valid_children);
          if (unlikely(mask == 0)) goto pop;

          __aligned(64) unsigned int child_mask[Nx];
          vint<Nx>::storeu(child_mask,vmask); // this explicit store here causes much better code generation

          /* select next child and push other children */
          //const BaseNode* node = cur.baseNode(types);

          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          assert(r < N);
          cur = node->child(r);         
          cur.prefetch(types);
          cur_mask = child_mask[r];

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
            cur_mask = child_mask[r];
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
            const size_t rayID = __bscf(bits);

            RayK<K> &ray = *inputPackets[rayID / K];
            const size_t k = rayID % K;
            if (PrimitiveIntersector::occluded(ray,k,context,prim,num,lazy_node)) {
              ray.geomID[k] = 0;
              terminated |= (size_t)1 << rayID;
            }
            /* lazy node */
            if (unlikely(lazy_node)) {
              stackPtr->ptr = lazy_node;
              stackPtr->dist = cur_mask;
              stackPtr++;
            }
          }

          if (unlikely(terminated == (size_t)-1)) { break; }
      }
    
#else
      __aligned(64) Packet<K,robust> packet[MAX_INTERNAL_STREAM_SIZE/K];
      __aligned(64) StackItemMaskCoherent stack[stackSizeSingle];  //!< stack of nodes
      __aligned(64) Frustum<N,Nx,K,robust> frusta;

      bool commonOctant = true;
      size_t m_active = initPacketsAndFrusta<true>(inputPackets, numOctantRays, packet, frusta, commonOctant);


      /* valid rays */
      if (unlikely(m_active == 0)) return;

      /* case of non-common origin */
      if (unlikely(!commonOctant))
      {
        const size_t numPackets = (numOctantRays+K-1)/K; 
        for (size_t i = 0; i < numPackets; i++)
          This->occluded(inputPackets[i]->tnear <= inputPackets[i]->tfar,*inputPackets[i],context);
        return;
      }

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      StackItemMaskCoherent* stackPtr = stack + 1;

      while (1) pop:
      {
        if (unlikely(stackPtr == stack)) break;

        STAT3(normal.trav_stack_pop,1,1,1);
        stackPtr--;
        /*! pop next node */
        NodeRef cur = NodeRef(stackPtr->child);
        size_t m_trav_active = stackPtr->mask & m_active;
        if (unlikely(!m_trav_active)) continue;
        assert(m_trav_active);
        NodeRef parent = stackPtr->parent;

        while (1)
        {
          if (unlikely(cur.isLeaf())) break;
          const AlignedNode* __restrict__ const node = cur.alignedNode();
          parent = cur;

          __aligned(64) size_t maskK[N];
          for (size_t i = 0; i < N; i++) maskK[i] = m_trav_active;

          vfloat<Nx> dist;
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packet, node, frusta, maskK, dist);
          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserStreamHitCoherent<N, Nx, types>::traverseAnyHit(cur, m_trav_active, vbool<Nx>((int)m_node_hit), (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /* non-root and leaf => full culling test for all rays */
        if (unlikely(parent != 0 && cur.isLeaf()))
        {
          const AlignedNode* __restrict__ const node = parent.alignedNode();
          size_t b = 0xff;
          for (size_t i=0;i<N;i++)
            if (node->child(i) == cur) { b = i; break; }
          assert(b < N);
          assert(cur == node->child(b));
          m_trav_active = intersectAlignedNodePacket(packet, node, b, frusta.nf, m_trav_active);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        size_t bits = m_trav_active & m_active;
        /*! intersect stream of rays with all primitives */
        size_t lazy_node = 0;
#if defined(__SSE4_2__)
        STAT_USER(1,(__popcnt(bits)+K-1)/K*4);
#endif
        while(bits)
        {
          size_t i = __bsf(bits) / K;
          const size_t m_isec = ((((size_t)1 << K)-1) << (i*K));
          assert(m_isec & bits);
          bits &= ~m_isec;
          Packet<K,robust> &p = packet[i];
          vbool<K> m_valid = p.min_dist <= p.max_dist;
          vbool<K> m_hit = PrimitiveIntersector::occludedK(m_valid, *inputPackets[i], context, prim, num, lazy_node);
          inputPackets[i]->geomID = select(m_hit & m_valid, vint<K>(zero), inputPackets[i]->geomID);
          m_active &= ~((size_t)movemask(m_hit) << (i*K));
        }
      } // traversal + intersection
#endif
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// ArrayIntersectorKStream Definitions
    ////////////////////////////////////////////////////////////////////////////////

    typedef ArrayIntersectorKStream<VSIZEX,TriangleMIntersectorKMoeller<4 COMMA VSIZEX COMMA VSIZEX COMMA true > > Triangle4IntersectorStreamMoeller;
    typedef ArrayIntersectorKStream<VSIZEX,TriangleMIntersectorKMoeller<4 COMMA VSIZEX COMMA VSIZEX COMMA false > > Triangle4IntersectorStreamMoellerNoFilter;
    typedef ArrayIntersectorKStream<VSIZEX,TriangleMvIntersectorKPluecker<4 COMMA VSIZEX COMMA VSIZEX COMMA true > > Triangle4vIntersectorStreamPluecker;
    typedef ArrayIntersectorKStream<VSIZEX,TriangleMiIntersectorKMoeller<4 COMMA VSIZEX COMMA VSIZEX COMMA true > > Triangle4iIntersectorStreamMoeller;
    typedef ArrayIntersectorKStream<VSIZEX,TriangleMiIntersectorKPluecker<4 COMMA VSIZEX COMMA VSIZEX COMMA true > > Triangle4iIntersectorStreamPluecker;
    typedef ArrayIntersectorKStream<VSIZEX,QuadMvIntersectorKMoeller<4 COMMA VSIZEX COMMA true > > Quad4vIntersectorStreamMoeller;
    typedef ArrayIntersectorKStream<VSIZEX,QuadMvIntersectorKMoeller<4 COMMA VSIZEX COMMA false > > Quad4vIntersectorStreamMoellerNoFilter;
    typedef ArrayIntersectorKStream<VSIZEX,QuadMiIntersectorKMoeller<4 COMMA VSIZEX COMMA true > > Quad4iIntersectorStreamMoeller;
    typedef ArrayIntersectorKStream<VSIZEX,QuadMvIntersectorKPluecker<4 COMMA VSIZEX COMMA true > > Quad4vIntersectorStreamPluecker;
    typedef ArrayIntersectorKStream<VSIZEX,QuadMiIntersectorKPluecker<4 COMMA VSIZEX COMMA true > > Quad4iIntersectorStreamPluecker;
    typedef ArrayIntersectorKStream<VSIZEX,ObjectIntersectorK<VSIZEX COMMA false > > ObjectIntersectorStream;


    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int N, int Nx, int K>
    void BVHNIntersectorStreamPacketFallback<N, Nx, K>::intersect(Accel::Intersectors* __restrict__ This, RayK<K>** inputRays, size_t numTotalRays, IntersectContext* context)
    {
      /* fallback to packets */
      for (size_t i = 0; i < numTotalRays; i += K)
      {
        const vint<K> vi = vint<K>(int(i)) + vint<K>(step);
        vbool<K> valid = vi < vint<K>(int(numTotalRays));
        RayK<K>& ray = *(inputRays[i / K]);
        valid &= ray.tnear <= ray.tfar;
        This->intersect(valid, ray, context);
      }
    }

    template<int N, int Nx, int K>
    void BVHNIntersectorStreamPacketFallback<N, Nx, K>::occluded(Accel::Intersectors* __restrict__ This, RayK<K>** inputRays, size_t numTotalRays, IntersectContext* context)
    {
      /* fallback to packets */
      for (size_t i = 0; i < numTotalRays; i += K)
      {
        const vint<K> vi = vint<K>(int(i)) + vint<K>(step);
        vbool<K> valid = vi < vint<K>(int(numTotalRays));
        RayK<K>& ray = *(inputRays[i / K]);
        valid &= ray.tnear <= ray.tfar;
        This->occluded(valid, ray, context);
      }
    }

  }
}
