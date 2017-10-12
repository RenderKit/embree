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
    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    __forceinline void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersect(Accel::Intersectors* __restrict__ This,
                                                                                                       RayK<K>** inputPackets,
                                                                                                       size_t numOctantRays,
                                                                                                       IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      __aligned(64) StackItemMaskCoherent stack[stackSizeSingle];  // stack of nodes
      assert(numOctantRays <= MAX_INTERNAL_STREAM_SIZE);

      __aligned(64) TravRayKStream<K, robust> packets[MAX_INTERNAL_STREAM_SIZE/K];
      __aligned(64) Frustum<robust> frustum;

      bool commonOctant = true;
      const size_t m_active = initPacketsAndFrustum<false>(inputPackets, numOctantRays, packets, frustum, commonOctant);
      if (unlikely(m_active == 0)) return;

      /* case of non-common origin */
      if (unlikely(!commonOctant))
      {
        const size_t numPackets = (numOctantRays+K-1)/K; 
        for (size_t i = 0; i < numPackets; i++)
          This->intersect(inputPackets[i]->tnear <= inputPackets[i]->tfar, *inputPackets[i], context);
        return;
      }

      stack[0].mask   = m_active;
      stack[0].parent = 0;
      stack[0].child  = bvh->root;

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
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packets, node, frustum, maskK, dist);
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
          m_trav_active = intersectAlignedNodePacket(packets, node, b, frustum.nf, m_trav_active);
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

          TravRayKStream<K, robust>& p = packets[i];
          vbool<K> m_valid = p.tnear <= p.tfar;
          PrimitiveIntersector::intersectK(m_valid, *inputPackets[i], context, prim, num, lazy_node);
          p.tfar = min(p.tfar, inputPackets[i]->tfar);
        };

      } // traversal + intersection
    }

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    __forceinline void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occluded(Accel::Intersectors* __restrict__ This,
                                                                                                      RayK<K>** inputPackets,
                                                                                                      size_t numOctantRays,
                                                                                                      IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*)This->ptr;
      __aligned(64) StackItemMaskCoherent stack[stackSizeSingle];  // stack of nodes
      assert(numOctantRays <= MAX_INTERNAL_STREAM_SIZE);

      /* inactive rays should have been filtered out before */
      __aligned(64) TravRayKStream<K, robust> packets[MAX_INTERNAL_STREAM_SIZE/K];
      __aligned(64) Frustum<robust> frustum;

      bool commonOctant = true;
      size_t m_active = initPacketsAndFrustum<true>(inputPackets, numOctantRays, packets, frustum, commonOctant);

      /* valid rays */
      if (unlikely(m_active == 0)) return;

      /* case of non-common origin */
      if (unlikely(!commonOctant))
      {
        const size_t numPackets = (numOctantRays+K-1)/K; 
        for (size_t i = 0; i < numPackets; i++)
          This->occluded(inputPackets[i]->tnear <= inputPackets[i]->tfar, *inputPackets[i], context);
        return;
      }

      stack[0].mask   = m_active;
      stack[0].parent = 0;
      stack[0].child  = bvh->root;

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
          for (size_t i = 0; i < N; i++)
            maskK[i] = m_trav_active;

          vfloat<Nx> dist;
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packets, node, frustum, maskK, dist);
          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserStreamHitCoherent<N, Nx, types>::traverseAnyHit(cur, m_trav_active, vbool<Nx>((int)m_node_hit), (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /* non-root and leaf => full culling test for all rays */
        if (unlikely(parent != 0 && cur.isLeaf()))
        {
          const AlignedNode* __restrict__ const node = parent.alignedNode();
          size_t b = 0xff;
          for (size_t i = 0; i < N; i++)
            if (node->child(i) == cur) { b = i; break; }
          assert(b < N);
          assert(cur == node->child(b));
          m_trav_active = intersectAlignedNodePacket(packets, node, b, frustum.nf, m_trav_active);
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
          TravRayKStream<K, robust>& p = packets[i];
          vbool<K> m_valid = p.tnear <= p.tfar;
          vbool<K> m_hit = PrimitiveIntersector::occludedK(m_valid, *inputPackets[i], context, prim, num, lazy_node);
          inputPackets[i]->geomID = select(m_hit & m_valid, vint<K>(zero), inputPackets[i]->geomID);
          m_active &= ~((size_t)movemask(m_hit) << (i*K));
        }

      } // traversal + intersection
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
    void BVHNIntersectorStreamPacketFallback<N, Nx, K>::intersect(Accel::Intersectors* __restrict__ This,
                                                                  RayK<K>** inputRays,
                                                                  size_t numTotalRays,
                                                                  IntersectContext* context)
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
    void BVHNIntersectorStreamPacketFallback<N, Nx, K>::occluded(Accel::Intersectors* __restrict__ This,
                                                                 RayK<K>** inputRays,
                                                                 size_t numTotalRays,
                                                                 IntersectContext* context)
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
