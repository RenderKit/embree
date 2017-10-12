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

#pragma once

#include "node_intersector_packet_stream.h"
#include "node_intersector_frustum.h"
#include "bvh_traverser_stream.h"

namespace embree
{
  namespace isa 
  {
    /*! BVH ray stream intersector. */
    template<int N, int Nx, int K, int types, bool robust, typename PrimitiveIntersector>
    class BVHNIntersectorStream
    {
      static const int Nxd = (Nx == N) ? N : Nx/2;

      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector::PrimitiveK Primitive;
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;
      typedef typename BVH::AlignedNode AlignedNode;
      typedef typename BVH::AlignedNodeMB AlignedNodeMB;

      template<bool occluded>
      __forceinline static size_t initPacketsAndFrustum(RayK<K>** inputPackets, size_t numOctantRays,
                                                        TravRayKStream<K, robust>* packets, Frustum<robust>& frustum, bool& commonOctant)
      {
        const size_t numPackets = (numOctantRays+K-1)/K;

        Vec3vf<K> tmp_min_rdir(pos_inf);
        Vec3vf<K> tmp_max_rdir(neg_inf);
        Vec3vf<K> tmp_min_org(pos_inf);
        Vec3vf<K> tmp_max_org(neg_inf);
        vfloat<K> tmp_min_dist(pos_inf);
        vfloat<K> tmp_max_dist(neg_inf);

        size_t m_active = 0;
        for (size_t i = 0; i < numPackets; i++)
        {
          const vfloat<K> tnear = inputPackets[i]->tnear;
          const vfloat<K> tfar  = inputPackets[i]->tfar;
          vbool<K> m_valid = (tnear <= tfar) & (tnear >= 0.0f);
          if (occluded) m_valid &= inputPackets[i]->geomID != 0;

#if defined(EMBREE_IGNORE_INVALID_RAYS)
          m_valid &= inputPackets[i]->valid();
#endif

          m_active |= (size_t)movemask(m_valid) << (i*K);

          vfloat<K> packet_min_dist = max(tnear, 0.0f);
          vfloat<K> packet_max_dist = select(m_valid, tfar, neg_inf);
          tmp_min_dist = min(tmp_min_dist, packet_min_dist);
          tmp_max_dist = max(tmp_max_dist, packet_max_dist);

          const Vec3vf<K>& org = inputPackets[i]->org;
          const Vec3vf<K>& dir = inputPackets[i]->dir;

          new (&packets[i]) TravRayKStream<K, robust>(org, dir, packet_min_dist, packet_max_dist);

          tmp_min_rdir = min(tmp_min_rdir, select(m_valid, packets[i].rdir, Vec3vf<K>(pos_inf)));
          tmp_max_rdir = max(tmp_max_rdir, select(m_valid, packets[i].rdir, Vec3vf<K>(neg_inf)));
          tmp_min_org  = min(tmp_min_org , select(m_valid,org , Vec3vf<K>(pos_inf)));
          tmp_max_org  = max(tmp_max_org , select(m_valid,org , Vec3vf<K>(neg_inf)));
        }

        m_active &= (numOctantRays == (8 * sizeof(size_t))) ? (size_t)-1 : (((size_t)1 << numOctantRays)-1);

        
        const Vec3fa reduced_min_rdir(reduce_min(tmp_min_rdir.x),
                                      reduce_min(tmp_min_rdir.y),
                                      reduce_min(tmp_min_rdir.z));

        const Vec3fa reduced_max_rdir(reduce_max(tmp_max_rdir.x),
                                      reduce_max(tmp_max_rdir.y),
                                      reduce_max(tmp_max_rdir.z));

        const Vec3fa reduced_min_origin(reduce_min(tmp_min_org.x),
                                        reduce_min(tmp_min_org.y),
                                        reduce_min(tmp_min_org.z));

        const Vec3fa reduced_max_origin(reduce_max(tmp_max_org.x),
                                        reduce_max(tmp_max_org.y),
                                        reduce_max(tmp_max_org.z));

        commonOctant =
          (reduced_max_rdir.x < 0.0f || reduced_min_rdir.x >= 0.0f) &&
          (reduced_max_rdir.y < 0.0f || reduced_min_rdir.y >= 0.0f) &&
          (reduced_max_rdir.z < 0.0f || reduced_min_rdir.z >= 0.0f);
        
        const float frustum_min_dist = reduce_min(tmp_min_dist);
        const float frustum_max_dist = reduce_max(tmp_max_dist);

        frustum.init(reduced_min_origin, reduced_max_origin,
                     reduced_min_rdir, reduced_max_rdir,
                     frustum_min_dist, frustum_max_dist,
                     N);
        
        return m_active;
      }

      
      __forceinline static size_t intersectAlignedNodePacket(const TravRayKStream<K,robust>* packets,
                                                             const AlignedNode* __restrict__ node,
                                                             size_t bid,
                                                             const NearFarPrecalculations& nf,
                                                             size_t m_active)
      {
        assert(m_active);
        const size_t startPacketID = __bsf(m_active) / K;
        const size_t endPacketID   = __bsr(m_active) / K;
        size_t m_trav_active = 0;
        for (size_t i = startPacketID; i <= endPacketID; i++)
        {
          const size_t m_hit = intersectNodeK<N>(node, bid, packets[i], nf);
          m_trav_active |= m_hit << (i*K);
        } 
        return m_trav_active;
      }
      
      __forceinline static size_t traverseCoherentStream(size_t m_trav_active,
                                                         TravRayKStream<K, robust>* packets,
                                                         const AlignedNode* __restrict__ node,
                                                         const Frustum<robust>& frustum,
                                                         size_t* maskK,
                                                         vfloat<Nx>& dist)
      {
        size_t m_node_hit = intersectNode<N,Nx>(node, frustum, dist);
        const size_t first_index    = __bsf(m_trav_active);
        const size_t first_packetID = first_index / K;
        const size_t first_rayID    = first_index % K;
        size_t m_first_hit = intersectNode1<N,Nx>(node, packets[first_packetID], first_rayID, frustum.nf);

        /* this make traversal independent of the ordering of rays */
        size_t m_node = m_node_hit ^ m_first_hit;
        while (unlikely(m_node))
        {
          const size_t b = __bscf(m_node);
          const size_t m_current = m_trav_active & intersectAlignedNodePacket(packets, node, b, frustum.nf, m_trav_active);
          m_node_hit ^= m_current ? (size_t)0 : ((size_t)1 << b);
          maskK[b] = m_current;
        }
        return m_node_hit;
      }

      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

    public:
      static void intersect(Accel::Intersectors* This, RayK<K>** inputRays, size_t numRays, IntersectContext* context);
      static void occluded (Accel::Intersectors* This, RayK<K>** inputRays, size_t numRays, IntersectContext* context);
    };


    /*! BVH ray stream intersector with direct fallback to packets. */
    template<int N, int Nx, int K>
    class BVHNIntersectorStreamPacketFallback
    {
    public:
      static void intersect(Accel::Intersectors* This, RayK<K>** inputRays, size_t numRays, IntersectContext* context);
      static void occluded (Accel::Intersectors* This, RayK<K>** inputRays, size_t numRays, IntersectContext* context);
    };
  }
}
