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

#include "bvh.h"
#include "../common/ray.h"
#include "../common/stack_item.h"
#include "bvh_traverser1.h"

#define ENABLE_COHERENT_STREAM_PATH 1

namespace embree
{
  namespace isa 
  {


    // ==================================================================================================
    // ==================================================================================================
    // ==================================================================================================

    struct __aligned(8) StackItemMaskCoherent
    {
      size_t mask;
      size_t parent;
      size_t child;
    };

    template<int N, int Nx, int types>
    class BVHNNodeTraverserStreamHitCoherent
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;

    public:
      template<class T>
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t& m_trav_active,
                                                   const vbool<Nx>& vmask,
                                                   const vfloat<Nx>& tNear,
                                                   const T* const tMask,
                                                   StackItemMaskCoherent*& stackPtr)
      {
        const NodeRef parent = cur;
        size_t mask = movemask(vmask);
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        const size_t r0 = __bscf(mask);          
        assert(r0 < 8);
        cur = node->child(r0);         
        cur.prefetch(types);
        m_trav_active = tMask[r0];        
        assert(cur != BVH::emptyNode);
        if (unlikely(mask == 0)) return;

        const unsigned int* const tNear_i = (unsigned int*)&tNear;

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur; 
        unsigned int d0 = tNear_i[r0];
        const size_t r1 = __bscf(mask);
        assert(r1 < 8);
        NodeRef c1 = node->child(r1); 
        c1.prefetch(types); 
        unsigned int d1 = tNear_i[r1];

        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          if (d0 < d1) { 
            assert(tNear[r1] >= 0.0f);
            stackPtr->mask    = tMask[r1]; 
            stackPtr->parent  = parent;
            stackPtr->child   = c1;
            stackPtr++; 
            cur = c0; 
            m_trav_active = tMask[r0]; 
            return; 
          }
          else { 
            assert(tNear[r0] >= 0.0f);
            stackPtr->mask    = tMask[r0]; 
            stackPtr->parent  = parent;
            stackPtr->child   = c0;
            stackPtr++; 
            cur = c1; 
            m_trav_active = tMask[r1]; 
            return; 
          }
        }

        /*! slow path for more than two hits */
        size_t hits = movemask(vmask);
        const vint<Nx> dist_i = select(vmask, (asInt(tNear) & 0xfffffff8) | vint<Nx>(step), 0);
#if defined(__AVX512F__) && !defined(__AVX512VL__) // KNL
        const vint<N> tmp = extractN<N,0>(dist_i);
        const vint<Nx> dist_i_sorted = usort_descending(tmp);
#else
        const vint<Nx> dist_i_sorted = usort_descending(dist_i);
#endif
        const vint<Nx> sorted_index = dist_i_sorted & 7;

        size_t i = 0;
        for (;;)
        {
          const unsigned int index = sorted_index[i];
          assert(index < 8);
          cur = node->child(index);
          m_trav_active = tMask[index];
          assert(m_trav_active);
          cur.prefetch(types);
          __bscf(hits);
          if (unlikely(hits==0)) break;
          i++;
          assert(cur != BVH::emptyNode);
          assert(tNear[index] >= 0.0f);
          stackPtr->mask    = m_trav_active;
          stackPtr->parent  = parent;
          stackPtr->child   = cur;
          stackPtr++;
        }
      }

      template<class T>
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t& m_trav_active,
                                               const vbool<Nx>& vmask,
                                               const T* const tMask,
                                               StackItemMaskCoherent*& stackPtr)
      {
        const NodeRef parent = cur;
        size_t mask = movemask(vmask);
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        m_trav_active = tMask[r];

        /* simple in order sequence */
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        stackPtr->mask    = m_trav_active;
        stackPtr->parent  = parent;
        stackPtr->child   = cur;
        stackPtr++;

        for (; ;)
        {
          r = __bscf(mask);
          cur = node->child(r);
          cur.prefetch(types);
          m_trav_active = tMask[r];
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return;
          stackPtr->mask    = m_trav_active;
          stackPtr->parent  = parent;
          stackPtr->child   = cur;
          stackPtr++;
        }
      }
    };

    // ==================================================================================================
    // ==================================================================================================
    // ==================================================================================================



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

      // =============================================================================================
      // =============================================================================================
      // =============================================================================================

      struct Packet
      {
        Vec3vf<K> rdir;
        Vec3vf<K> org_rdir;
        vfloat<K> min_dist;
        vfloat<K> max_dist;
      };

      struct Frusta
      {
        Vec3fa min_rdir; 
        Vec3fa max_rdir; 
        Vec3fa min_org_rdir; 
        Vec3fa max_org_rdir; 
        float min_dist;
        float max_dist;
      };

      struct NearFarPreCompute
      {
#if defined(__AVX512F__)
        vint16 permX, permY, permZ;
#endif
        size_t nearX, nearY, nearZ;
        size_t farX, farY, farZ;

        __forceinline NearFarPreCompute(const Vec3fa& dir)
        {
#if defined(__AVX512F__)
          /* optimization works only for 8-wide BVHs with 16-wide SIMD */
          const vint<16> id(step);
          const vint<16> id2 = align_shift_right<16/2>(id, id);
          permX = select(vfloat<16>(dir.x) >= 0.0f, id, id2);
          permY = select(vfloat<16>(dir.y) >= 0.0f, id, id2);
          permZ = select(vfloat<16>(dir.z) >= 0.0f, id, id2);
#endif
          nearX = (dir.x < 0.0f) ? 1*sizeof(vfloat<N>) : 0*sizeof(vfloat<N>);
          nearY = (dir.y < 0.0f) ? 3*sizeof(vfloat<N>) : 2*sizeof(vfloat<N>);
          nearZ = (dir.z < 0.0f) ? 5*sizeof(vfloat<N>) : 4*sizeof(vfloat<N>);
          farX  = nearX ^ sizeof(vfloat<N>);
          farY  = nearY ^ sizeof(vfloat<N>);
          farZ  = nearZ ^ sizeof(vfloat<N>);
        }
      };

      template<bool occluded>
      __forceinline static size_t initPacketsAndFrusta(RayK<K>** inputPackets, const size_t numOctantRays, Packet* const packet, Frusta& frusta, bool &commonOctant)
      {
        const size_t numPackets = (numOctantRays+K-1)/K;

        Vec3vf<K>   tmp_min_rdir(pos_inf);
        Vec3vf<K>   tmp_max_rdir(neg_inf);
        Vec3vf<K>   tmp_min_org(pos_inf);
        Vec3vf<K>   tmp_max_org(neg_inf);
        vfloat<K> tmp_min_dist(pos_inf);
        vfloat<K> tmp_max_dist(neg_inf);

        size_t m_active = 0;
        for (size_t i = 0; i < numPackets; i++)
        {
          const vfloat<K> tnear  = inputPackets[i]->tnear;
          const vfloat<K> tfar   = inputPackets[i]->tfar;
          vbool<K> m_valid = (tnear <= tfar) & (tnear >= 0.0f);
          //if (occluded) m_valid &= inputPackets[i]->geomID == RTC_INVALID_GEOMETRY_ID;

#if defined(EMBREE_IGNORE_INVALID_RAYS)
          m_valid &= inputPackets[i]->valid();
#endif

          m_active |= (size_t)movemask(m_valid) << (i*K);

          packet[i].min_dist = max(tnear, 0.0f);
          packet[i].max_dist = select(m_valid, tfar, neg_inf);
          tmp_min_dist = min(tmp_min_dist, packet[i].min_dist);
          tmp_max_dist = max(tmp_max_dist, packet[i].max_dist);

          const Vec3vf<K>& org     = inputPackets[i]->org;
          const Vec3vf<K>& dir     = inputPackets[i]->dir;
          const Vec3vf<K> rdir     = rcp_safe(dir);
          const Vec3vf<K> org_rdir = org * rdir;
        
          packet[i].rdir     = rdir;
          packet[i].org_rdir = org_rdir;

          tmp_min_rdir = min(tmp_min_rdir, select(m_valid,rdir, Vec3vf<K>(pos_inf)));
          tmp_max_rdir = max(tmp_max_rdir, select(m_valid,rdir, Vec3vf<K>(neg_inf)));
          tmp_min_org  = min(tmp_min_org , select(m_valid,org , Vec3vf<K>(pos_inf)));
          tmp_max_org  = max(tmp_max_org , select(m_valid,org , Vec3vf<K>(neg_inf)));
        }

        m_active &= (numOctantRays == (8 * sizeof(size_t))) ? (size_t)-1 : (((size_t)1 << numOctantRays)-1);
        const Vec3fa reduced_min_rdir( reduce_min(tmp_min_rdir.x), 
                                       reduce_min(tmp_min_rdir.y),
                                       reduce_min(tmp_min_rdir.z) );

        const Vec3fa reduced_max_rdir( reduce_max(tmp_max_rdir.x), 
                                       reduce_max(tmp_max_rdir.y),
                                       reduce_max(tmp_max_rdir.z) );

        const Vec3fa reduced_min_origin( reduce_min(tmp_min_org.x), 
                                         reduce_min(tmp_min_org.y),
                                         reduce_min(tmp_min_org.z) );

        const Vec3fa reduced_max_origin( reduce_max(tmp_max_org.x), 
                                         reduce_max(tmp_max_org.y),
                                         reduce_max(tmp_max_org.z) );

        const float frusta_min_dist = reduce_min(tmp_min_dist);
        const float frusta_max_dist = reduce_max(tmp_max_dist);

        commonOctant =
          (reduced_max_rdir.x < 0.0f || reduced_min_rdir.x >= 0.0f) &&
          (reduced_max_rdir.y < 0.0f || reduced_min_rdir.y >= 0.0f) &&
          (reduced_max_rdir.z < 0.0f || reduced_min_rdir.z >= 0.0f);

        
        const Vec3fa frusta_min_rdir = select(ge_mask(reduced_min_rdir, Vec3fa(zero)), reduced_min_rdir, reduced_max_rdir);
        const Vec3fa frusta_max_rdir = select(ge_mask(reduced_min_rdir, Vec3fa(zero)), reduced_max_rdir, reduced_min_rdir);

        const Vec3fa frusta_min_org_rdir = frusta_min_rdir * select(ge_mask(reduced_min_rdir, Vec3fa(zero)), reduced_max_origin, reduced_min_origin);
        const Vec3fa frusta_max_org_rdir = frusta_max_rdir * select(ge_mask(reduced_min_rdir, Vec3fa(zero)), reduced_min_origin, reduced_max_origin);

        frusta.min_rdir     = frusta_min_rdir;
        frusta.max_rdir     = frusta_max_rdir;
        frusta.min_org_rdir = frusta_min_org_rdir;
        frusta.max_org_rdir = frusta_max_org_rdir;
        frusta.min_dist     = frusta_min_dist;
        frusta.max_dist     = frusta_max_dist;

        return m_active;
      }

      __forceinline static size_t intersectAlignedNodePacket(const Packet* const packet,
                                                             const vfloat<K>& minX,
                                                             const vfloat<K>& minY,
                                                             const vfloat<K>& minZ,
                                                             const vfloat<K>& maxX,
                                                             const vfloat<K>& maxY,
                                                             const vfloat<K>& maxZ,
                                                             const size_t m_active)
      {
        assert(m_active);
        const size_t startPacketID = __bsf(m_active) / K;
        const size_t endPacketID   = __bsr(m_active) / K;
        size_t m_trav_active = 0;
        for (size_t i = startPacketID; i <= endPacketID; i++)
        {
          //STAT3(normal.trav_nodes,1,1,1);                          
          const Packet& p = packet[i];
          const vfloat<K> tminX = msub(minX, p.rdir.x, p.org_rdir.x);
          const vfloat<K> tminY = msub(minY, p.rdir.y, p.org_rdir.y);
          const vfloat<K> tminZ = msub(minZ, p.rdir.z, p.org_rdir.z);
          const vfloat<K> tmaxX = msub(maxX, p.rdir.x, p.org_rdir.x);
          const vfloat<K> tmaxY = msub(maxY, p.rdir.y, p.org_rdir.y);
          const vfloat<K> tmaxZ = msub(maxZ, p.rdir.z, p.org_rdir.z);
          const vfloat<K> tmin  = maxi(tminX, tminY, tminZ, p.min_dist);
          const vfloat<K> tmax  = mini(tmaxX, tmaxY, tmaxZ, p.max_dist);
          const vbool<K> vmask  = tmin <= tmax;
          const size_t m_hit = movemask(vmask);
          m_trav_active |= m_hit << (i*K);
        } 
        return m_trav_active;
      }

      __forceinline static size_t traverseCoherentStream(const size_t m_trav_active,
                                                         Packet* const packet,
                                                         const AlignedNode* __restrict__ const node,
                                                         const NearFarPreCompute& pc,
                                                         const Frusta& frusta,
                                                         size_t* const maskK,
                                                         vfloat<Nx>& dist)
      {
        /* interval-based culling test */
        const vfloat<Nx> bminX = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x + pc.nearX));
        const vfloat<Nx> bminY = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x + pc.nearY));
        const vfloat<Nx> bminZ = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x + pc.nearZ));
        const vfloat<Nx> bmaxX = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x + pc.farX));
        const vfloat<Nx> bmaxY = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x + pc.farY));
        const vfloat<Nx> bmaxZ = vfloat<Nx>(*(const vfloat<N>*)((const char*)&node->lower_x + pc.farZ));

        const vfloat<Nx> fminX = msub(bminX, vfloat<Nx>(frusta.min_rdir.x), vfloat<Nx>(frusta.min_org_rdir.x));
        const vfloat<Nx> fminY = msub(bminY, vfloat<Nx>(frusta.min_rdir.y), vfloat<Nx>(frusta.min_org_rdir.y));
        const vfloat<Nx> fminZ = msub(bminZ, vfloat<Nx>(frusta.min_rdir.z), vfloat<Nx>(frusta.min_org_rdir.z));
        const vfloat<Nx> fmaxX = msub(bmaxX, vfloat<Nx>(frusta.max_rdir.x), vfloat<Nx>(frusta.max_org_rdir.x));
        const vfloat<Nx> fmaxY = msub(bmaxY, vfloat<Nx>(frusta.max_rdir.y), vfloat<Nx>(frusta.max_org_rdir.y));
        const vfloat<Nx> fmaxZ = msub(bmaxZ, vfloat<Nx>(frusta.max_rdir.z), vfloat<Nx>(frusta.max_org_rdir.z));
        const vfloat<Nx> fmin  = maxi(fminX, fminY, fminZ, vfloat<Nx>(frusta.min_dist));
        const vfloat<Nx> fmax  = mini(fmaxX, fmaxY, fmaxZ, vfloat<Nx>(frusta.max_dist));
        const vbool<Nx> vmask_node_hit = fmin <= fmax;

        //STAT3(normal.trav_nodes,1,1,1);                          

        size_t m_node_hit = movemask(vmask_node_hit) & (((size_t)1 << N)-1);
        // ==================
        const size_t first_index    = __bsf(m_trav_active);
        const size_t first_packetID = first_index / K;
        const size_t first_rayID    = first_index % K;

        Packet &p = packet[first_packetID]; 
        //STAT3(normal.trav_nodes,1,1,1);                          

        const vfloat<Nx> rminX = msub(bminX, vfloat<Nx>(p.rdir.x[first_rayID]), vfloat<Nx>(p.org_rdir.x[first_rayID]));
        const vfloat<Nx> rminY = msub(bminY, vfloat<Nx>(p.rdir.y[first_rayID]), vfloat<Nx>(p.org_rdir.y[first_rayID]));
        const vfloat<Nx> rminZ = msub(bminZ, vfloat<Nx>(p.rdir.z[first_rayID]), vfloat<Nx>(p.org_rdir.z[first_rayID]));
        const vfloat<Nx> rmaxX = msub(bmaxX, vfloat<Nx>(p.rdir.x[first_rayID]), vfloat<Nx>(p.org_rdir.x[first_rayID]));
        const vfloat<Nx> rmaxY = msub(bmaxY, vfloat<Nx>(p.rdir.y[first_rayID]), vfloat<Nx>(p.org_rdir.y[first_rayID]));
        const vfloat<Nx> rmaxZ = msub(bmaxZ, vfloat<Nx>(p.rdir.z[first_rayID]), vfloat<Nx>(p.org_rdir.z[first_rayID]));
        const vfloat<Nx> rmin  = maxi(rminX, rminY, rminZ, vfloat<Nx>(p.min_dist[first_rayID]));
        const vfloat<Nx> rmax  = mini(rmaxX, rmaxY, rmaxZ, vfloat<Nx>(p.max_dist[first_rayID]));

        const vbool<Nx> vmask_first_hit = rmin <= rmax;

        size_t m_first_hit = movemask(vmask_first_hit) & (((size_t)1 << N)-1);

        // ==================

        /* this causes a traversal order dependence with respect to the order of rays within the stream */
        //dist = select(vmask_first_hit, rmin, fmin);
        /* this is independent of the ordering of rays */
        dist = fmin;            
            

        size_t m_node = m_node_hit ^ m_first_hit;
        while(unlikely(m_node)) 
        {
          const size_t b = __bscf(m_node);
          const vfloat<K> minX = vfloat<K>(bminX[b]);
          const vfloat<K> minY = vfloat<K>(bminY[b]);
          const vfloat<K> minZ = vfloat<K>(bminZ[b]);
          const vfloat<K> maxX = vfloat<K>(bmaxX[b]);
          const vfloat<K> maxY = vfloat<K>(bmaxY[b]);
          const vfloat<K> maxZ = vfloat<K>(bmaxZ[b]);
          const size_t m_current = m_trav_active & intersectAlignedNodePacket(packet, minX, minY, minZ, maxX, maxY, maxZ, m_trav_active);
          m_node_hit ^= m_current ? (size_t)0 : ((size_t)1 << b);
          maskK[b] = m_current;
        }
        return m_node_hit;
      }

      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

      // =============================================================================================
      // =============================================================================================
      // =============================================================================================


      static void intersectCoherent(BVH* bvh, RayK<K>** inputRays, size_t numValidStreams, IntersectContext* context);
      static void occludedCoherent(BVH* bvh, RayK<K>** inputRays, size_t numValidStreams, IntersectContext* context);
      
    public:
      static void intersect(BVH* bvh, RayK<K>** inputRays, size_t numRays, IntersectContext* context);
      static void occluded (BVH* bvh, RayK<K>** inputRays, size_t numRays, IntersectContext* context);
    };
  }
}
