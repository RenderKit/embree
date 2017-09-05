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

// TODO: bvh->scene->intersect/occluded correct vs. global scene->intersect/occluded

#define MAX_RAYS 64

namespace embree
{
  namespace isa
  {
    /* enable traversal of either two small streams or one large stream */
#if !defined(__AVX512F__)
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(unsigned int);
#else
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(size_t);
#endif
    static_assert(MAX_RAYS_PER_OCTANT <= MAX_INTERNAL_STREAM_SIZE, "maximal internal stream size exceeded");

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int K>
    __forceinline size_t AOStoSOA(RayK<K>* rayK, Ray** inputRays, const size_t numTotalRays)
    {
      const size_t numPackets = (numTotalRays+K-1)/K; //todo : OPTIMIZE
      for (size_t i = 0; i < numPackets; i++)
        new (&rayK[i]) RayK<K>(zero,zero,zero,neg_inf);

      Vec3fa min_dir = pos_inf;
      Vec3fa max_dir = neg_inf;

      for (size_t i = 0; i < numTotalRays; i++) {
        const Vec3fa& org = inputRays[i]->org;
        const Vec3fa& dir = inputRays[i]->dir;
        min_dir = min(min_dir, dir);
        max_dir = max(max_dir, dir);
        const float tnear = max(0.0f, inputRays[i]->tnear);
        const float tfar  = inputRays[i]->tfar;
        const size_t packetID = i / K;
        const size_t slotID   = i % K;
        rayK[packetID].dir.x[slotID]  = dir.x;
        rayK[packetID].dir.y[slotID]  = dir.y;
        rayK[packetID].dir.z[slotID]  = dir.z;
        rayK[packetID].org.x[slotID]  = org.x;
        rayK[packetID].org.y[slotID]  = org.y;
        rayK[packetID].org.z[slotID]  = org.z;
        rayK[packetID].tnear[slotID]  = tnear;
        rayK[packetID].tfar[slotID]   = tfar;
        rayK[packetID].mask[slotID]   = inputRays[i]->mask;
        rayK[packetID].instID[slotID] = inputRays[i]->instID;
      }
      const size_t sign_min_dir = movemask(vfloat4(min_dir) < 0.0f);
      const size_t sign_max_dir = movemask(vfloat4(max_dir) < 0.0f);
      return ((sign_min_dir^sign_max_dir) & 0x7);
    }

    template<int K, bool occlusion>
    __forceinline void SOAtoAOS(Ray** inputRays, RayK<K>* rayK, const size_t numTotalRays)
    {
      for (size_t i = 0; i < numTotalRays; i++)
      {
        const size_t packetID = i / K;
        const size_t slotID   = i % K;
        const RayK<K>& ray = rayK[packetID];
        if (likely((unsigned)ray.geomID[slotID] != RTC_INVALID_GEOMETRY_ID))
        {
          if (occlusion)
            inputRays[i]->geomID = ray.geomID[slotID];
          else
          {
            inputRays[i]->tfar   = ray.tfar[slotID];
            inputRays[i]->Ng.x   = ray.Ng.x[slotID];
            inputRays[i]->Ng.y   = ray.Ng.y[slotID];
            inputRays[i]->Ng.z   = ray.Ng.z[slotID];
            inputRays[i]->u      = ray.u[slotID];
            inputRays[i]->v      = ray.v[slotID];
            inputRays[i]->geomID = ray.geomID[slotID];
            inputRays[i]->primID = ray.primID[slotID];
            inputRays[i]->instID = ray.instID[slotID];
          }
        }
      }
    }

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    __forceinline void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersectCoherentSOA(BVH* __restrict__ bvh, RayK<K>** inputRays, size_t numOctantRays, IntersectContext* context)
    {
      __aligned(64) StackItemMaskCoherent stack[stackSizeSingle];  //!< stack of nodes

      RayK<K>** __restrict__ inputPackets = (RayK<K>**)inputRays;
      assert(numOctantRays <= MAX_RAYS);

      __aligned(64) Packet packet[MAX_RAYS/K];
      __aligned(64) Frusta frusta;

      bool commonOctant = true;
      const size_t m_active = initPacketsAndFrusta<false>(inputPackets, numOctantRays, packet, frusta, commonOctant);
      if (unlikely(m_active == 0)) return;

      /* case of non-common origin */
      if (unlikely(!commonOctant))
      {
        const size_t numPackets = (numOctantRays+K-1)/K; 
        for (size_t i = 0; i < numPackets; i++)
          bvh->scene->intersect(inputPackets[i]->tnear <= inputPackets[i]->tfar,*inputPackets[i],context);
        return;
      }

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      const NearFarPreCompute pc(frusta.min_rdir);

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
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packet, node, pc, frusta, maskK, dist);
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
          char *ptr = (char*)&node->lower_x + b*sizeof(float);
          assert(cur == node->child(b));

          const vfloat<K> minX = vfloat<K>(*(const float*)((const char*)ptr + pc.nearX));
          const vfloat<K> minY = vfloat<K>(*(const float*)((const char*)ptr + pc.nearY));
          const vfloat<K> minZ = vfloat<K>(*(const float*)((const char*)ptr + pc.nearZ));
          const vfloat<K> maxX = vfloat<K>(*(const float*)((const char*)ptr + pc.farX));
          const vfloat<K> maxY = vfloat<K>(*(const float*)((const char*)ptr + pc.farY));
          const vfloat<K> maxZ = vfloat<K>(*(const float*)((const char*)ptr + pc.farZ));

          m_trav_active = intersectAlignedNodePacket(packet, minX, minY, minZ, maxX, maxY, maxZ, m_trav_active);
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

          Packet &p = packet[i];
          vbool<K> m_valid = p.min_dist <= p.max_dist;
          PrimitiveIntersector::intersectK(m_valid, *inputPackets[i], context, prim, num, lazy_node);
          p.max_dist = min(p.max_dist, inputPackets[i]->tfar);
        };

      } // traversal + intersection
    }

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    __forceinline void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occludedCoherentSOA(BVH* __restrict__ bvh, RayK<K>** inputRays, size_t numOctantRays, IntersectContext* context)
    {
      __aligned(64) StackItemMaskCoherent stack[stackSizeSingle];  //!< stack of nodes

      RayK<K>** __restrict__ inputPackets = (RayK<K>**)inputRays;
      assert(numOctantRays <= MAX_RAYS);

      /* inactive rays should have been filtered out before */
      __aligned(64) Packet packet[MAX_RAYS/K];
      __aligned(64) Frusta frusta;

      bool commonOctant = true;
      size_t m_active = initPacketsAndFrusta<true>(inputPackets, numOctantRays, packet, frusta, commonOctant);

      /* valid rays */
      if (unlikely(m_active == 0)) return;

      /* case of non-common origin */
      if (unlikely(!commonOctant))
      {
        const size_t numPackets = (numOctantRays+K-1)/K; 
        for (size_t i = 0; i < numPackets; i++)
          bvh->scene->occluded(inputPackets[i]->tnear <= inputPackets[i]->tfar,*inputPackets[i],context);
        return;
      }

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      const NearFarPreCompute pc(frusta.min_rdir);

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
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packet, node, pc, frusta, maskK, dist);
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
          char *ptr = (char*)&node->lower_x + b*sizeof(float);
          assert(cur == node->child(b));

          const vfloat<K> minX = vfloat<K>(*(const float*)((const char*)ptr + pc.nearX));
          const vfloat<K> minY = vfloat<K>(*(const float*)((const char*)ptr + pc.nearY));
          const vfloat<K> minZ = vfloat<K>(*(const float*)((const char*)ptr + pc.nearZ));
          const vfloat<K> maxX = vfloat<K>(*(const float*)((const char*)ptr + pc.farX));
          const vfloat<K> maxY = vfloat<K>(*(const float*)((const char*)ptr + pc.farY));
          const vfloat<K> maxZ = vfloat<K>(*(const float*)((const char*)ptr + pc.farZ));

          m_trav_active = intersectAlignedNodePacket(packet, minX, minY, minZ, maxX, maxY, maxZ, m_trav_active);
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
          Packet &p = packet[i];
          vbool<K> m_valid = p.min_dist <= p.max_dist;
          vbool<K> m_hit = PrimitiveIntersector::occludedK(m_valid, *inputPackets[i], context, prim, num, lazy_node);
          inputPackets[i]->geomID = select(m_hit & m_valid, vint<K>(zero), inputPackets[i]->geomID);
          m_active &= ~((size_t)movemask(m_hit) << (i*K));
        }

      } // traversal + intersection
    }

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersectCoherent(BVH* __restrict__ bvh, Ray** inputRays, size_t numTotalRays, IntersectContext* context)
    {
      if (likely(context->flags == IntersectContext::INPUT_RAY_DATA_AOS))
      {
        /* AOS to SOA conversion */
        RayK<K> rayK[MAX_RAYS / K];
        RayK<K>* rayK_ptr[MAX_RAYS / K];
        for (size_t i = 0; i < MAX_RAYS / K; i++) rayK_ptr[i] = &rayK[i];
        AOStoSOA(rayK, inputRays, numTotalRays);
        /* stream tracer as fast path */
        BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersectCoherentSOA(bvh, (RayK<K>**)rayK_ptr, numTotalRays, context);
        /* SOA to AOS conversion */
        SOAtoAOS<K, false>(inputRays, rayK, numTotalRays);
      }
      else
      {
        assert(context->getInputSOAWidth() == K);
        /* stream tracer as fast path */
        BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersectCoherentSOA(bvh, (RayK<K>**)inputRays, numTotalRays, context);
      }
    }

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occludedCoherent(BVH* __restrict__ bvh, Ray **inputRays, size_t numTotalRays, IntersectContext* context)
    {
      if (likely(context->flags == IntersectContext::INPUT_RAY_DATA_AOS))
      {
        /* AOS to SOA conversion */
        RayK<K> rayK[MAX_RAYS / K];
        RayK<K>* rayK_ptr[MAX_RAYS / K];
        for (size_t i = 0; i < MAX_RAYS / K; i++) rayK_ptr[i] = &rayK[i];
        AOStoSOA(rayK, inputRays, numTotalRays);
        /* stream tracer as fast path */
        BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occludedCoherentSOA(bvh, (RayK<K>**)rayK_ptr, numTotalRays, context);
        /* SOA to AOS conversion */
        SOAtoAOS<K, true>(inputRays, rayK, numTotalRays);
      }
      else
      {
        assert(context->getInputSOAWidth() == K);
        BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occludedCoherentSOA(bvh, (RayK<K>**)inputRays, numTotalRays, context);
      }
    }

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::intersect(BVH* __restrict__ bvh, Ray** inputRays, size_t numTotalRays, IntersectContext* context)
    {
#if ENABLE_COHERENT_STREAM_PATH == 1
      if (unlikely(PrimitiveIntersector::validIntersectorK && !robust && isCoherent(context->user->flags)))
      {
        intersectCoherent(bvh, inputRays, numTotalRays, context);
        return;
      }
#endif
      assert(context->flags == IntersectContext::INPUT_RAY_DATA_AOS);
      /* fallback to packet interface */

      /* replace this with Attila's AOS to SOA conversion code */
      for (size_t i = 0; i < numTotalRays; i += K)
      {
        const size_t n = min(numTotalRays - i, size_t(K));
        vbool<K> valid = vint<K>(step) < vint<K>(int(n));
        RayK<K> ray;
        
        for (size_t k = 0; k < n; k++)
        {
          Ray* __restrict__ ray_k = inputRays[i+k];
          assert(k < K);
          assert(i+k < numTotalRays);

          ray.org.x[k]  = ray_k->org.x;
          ray.org.y[k]  = ray_k->org.y;
          ray.org.z[k]  = ray_k->org.z;
          ray.dir.x[k]  = ray_k->dir.x;
          ray.dir.y[k]  = ray_k->dir.y;
          ray.dir.z[k]  = ray_k->dir.z;
          ray.tnear[k]  = ray_k->tnear;
          ray.tfar[k]   = ray_k->tfar;
          ray.time[k]   = ray_k->time;
          ray.mask[k]   = ray_k->mask;
          ray.instID[k] = ray_k->instID;
        }
        ray.geomID = RTC_INVALID_GEOMETRY_ID;
        /* filter out invalid rays */
        valid &= ray.tnear <= ray.tfar;
        //scene->intersect(valid, ray, context);
        bvh->scene->intersect(valid, ray, context);

        for (size_t k = 0; k < n; k++)
        {
          Ray* __restrict__ ray_k = inputRays[i+k];

          if (ray.geomID[k] != RTC_INVALID_GEOMETRY_ID)
          {
            ray_k->tfar   = ray.tfar[k];
            ray_k->Ng.x   = ray.Ng.x[k];
            ray_k->Ng.y   = ray.Ng.y[k];
            ray_k->Ng.z   = ray.Ng.z[k];
            ray_k->u      = ray.u[k];
            ray_k->v      = ray.v[k];
            ray_k->primID = ray.primID[k];
            ray_k->geomID = ray.geomID[k];
            ray_k->instID = ray.instID[k];
          }
        }
      }
    }


    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNIntersectorStream<N, Nx, K, types, robust, PrimitiveIntersector>::occluded(BVH* __restrict__ bvh, Ray **inputRays, size_t numTotalRays, IntersectContext* context)
    {
#if ENABLE_COHERENT_STREAM_PATH == 1
      if (unlikely(PrimitiveIntersector::validIntersectorK && !robust && isCoherent(context->user->flags)))
      {
        occludedCoherent(bvh, inputRays, numTotalRays, context);
        return;
      }
#endif

      /* replace this with Attila's AOS to SOA conversion code */
      for (size_t i = 0; i < numTotalRays; i += K)
      {
        const size_t n = min(numTotalRays - i, size_t(K));
        vbool<K> valid = vint<K>(step) < vint<K>(int(n));
        RayK<K> ray;
        
        for (size_t k = 0; k < n; k++)
        {
          Ray* __restrict__ ray_k = inputRays[i+k];
          assert(k < K);
          assert(i+k < numTotalRays);

          ray.org.x[k]  = ray_k->org.x;
          ray.org.y[k]  = ray_k->org.y;
          ray.org.z[k]  = ray_k->org.z;
          ray.dir.x[k]  = ray_k->dir.x;
          ray.dir.y[k]  = ray_k->dir.y;
          ray.dir.z[k]  = ray_k->dir.z;
          ray.tnear[k]  = ray_k->tnear;
          ray.tfar[k]   = ray_k->tfar;
          ray.time[k]   = ray_k->time;
          ray.mask[k]   = ray_k->mask;
          ray.instID[k] = ray_k->instID;
        }
        ray.geomID = RTC_INVALID_GEOMETRY_ID;
        /* filter out invalid rays */
        valid &= ray.tnear <= ray.tfar;
        bvh->scene->occluded(valid, ray, context);

        for (size_t k = 0; k < n; k++)
        {
          Ray* __restrict__ ray_k = inputRays[i+k];

          if (ray.geomID[k] != RTC_INVALID_GEOMETRY_ID)
          {
            ray_k->geomID = ray.geomID[k];
          }
        }
      }
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
  }
}
