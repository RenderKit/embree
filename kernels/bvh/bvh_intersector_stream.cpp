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
#include "../common/scene.h"
#include <bitset>

// todo: parent ptr also for single stream, should improve culling.

//#define DBG_PRINT(x) PRINT(x)
#define DBG_PRINT(x)

#define ENABLE_CO_PATH 0
#define MAX_RAYS 64

namespace embree
{
  namespace isa
  {
/* enable traversal of either two small streams or one large stream */
#define TWO_STREAMS_FIBER_MODE 0 // 0 = no fiber, 1 = switch at pop, 2 = switch at each node, 3 = switch at leaf

#if TWO_STREAMS_FIBER_MODE == 0 && !defined(__AVX512F__)
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(unsigned int);
#else
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(size_t);
#endif
    static_assert(MAX_RAYS_PER_OCTANT <= MAX_INTERNAL_STREAM_SIZE,"maximal internal stream size exceeded");

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================


#if defined(__AVX512F__)
    template<int K, bool dist_update, bool robust>
    __forceinline vbool<K> intersectNode(const RayContext<K,robust> &ctx,
                                         const vfloat<K> &bminmaxX,
                                         const vfloat<K> &bminmaxY,
                                         const vfloat<K> &bminmaxZ,
                                         vfloat<K> &dist)
    {
      if (!robust)
      {
        const vfloat<K> tNearFarX = msub(bminmaxX, ctx.rdir.x, ctx.org_rdir.x);
        const vfloat<K> tNearFarY = msub(bminmaxY, ctx.rdir.y, ctx.org_rdir.y);
        const vfloat<K> tNearFarZ = msub(bminmaxZ, ctx.rdir.z, ctx.org_rdir.z);
        const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ctx.rdir.w));
        const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ctx.org_rdir.w));
        const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));  
        if (dist_update) dist     = select(vmask,min(tNear,dist),dist);
        return vmask;       
      }
      else
      {
        const Vec3fa &org = ctx.org_rdir;
        const vfloat<K> tNearFarX = (bminmaxX - org.x) * ctx.rdir.x; 
        const vfloat<K> tNearFarY = (bminmaxY - org.y) * ctx.rdir.y;
        const vfloat<K> tNearFarZ = (bminmaxZ - org.z) * ctx.rdir.z;
        const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ctx.rdir.w));
        const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(org.w));
        const float round_down    = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
        const float round_up      = 1.0f+2.0f*float(ulp);
        const vbool<K> vmask      = le(tNear*round_down,align_shift_right<8>(tFar,tFar)*round_up);  
        if (dist_update) dist     = select(vmask,min(tNear,dist),dist);
        return vmask;       
      }
    }
#endif

    template<int K, bool dist_update, bool robust>
    __forceinline vbool<K> intersectNode(const RayContext<K,robust> &ctx,
                                         const vfloat<K> &bminX,
                                         const vfloat<K> &bminY,
                                         const vfloat<K> &bminZ,
                                         const vfloat<K> &bmaxX,
                                         const vfloat<K> &bmaxY,
                                         const vfloat<K> &bmaxZ,
                                         vfloat<K> &dist)
    {
      if (!robust)
      {
        const vfloat<K> tNearX = msub(bminX, ctx.rdir.x, ctx.org_rdir.x);
        const vfloat<K> tNearY = msub(bminY, ctx.rdir.y, ctx.org_rdir.y);
        const vfloat<K> tNearZ = msub(bminZ, ctx.rdir.z, ctx.org_rdir.z);
        const vfloat<K> tFarX  = msub(bmaxX, ctx.rdir.x, ctx.org_rdir.x);
        const vfloat<K> tFarY  = msub(bmaxY, ctx.rdir.y, ctx.org_rdir.y);
        const vfloat<K> tFarZ  = msub(bmaxZ, ctx.rdir.z, ctx.org_rdir.z);

#if defined(__AVX2__)
        const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ctx.rdir.w)));
        const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(ctx.org_rdir.w)));
#else
        const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ctx.rdir.w));
        const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ctx.org_rdir.w));
#endif


#if defined(__AVX512F__)
        const unsigned int maskN = ((unsigned int)1 << N)-1;
        const vbool<K> vmask   = le(maskN,tNear,tFar);        
#else
        const vbool<K> vmask   = tNear <= tFar;
#endif
        if (dist_update) dist  = select(vmask,min(tNear,dist),dist);
        return vmask;    
      }
      else
      {
        const Vec3fa &org = ctx.org_rdir;
        const vfloat<K> tNearX = (bminX - org.x) * ctx.rdir.x;
        const vfloat<K> tNearY = (bminY - org.y) * ctx.rdir.y;
        const vfloat<K> tNearZ = (bminZ - org.z) * ctx.rdir.z;
        const vfloat<K> tFarX  = (bmaxX - org.x) * ctx.rdir.x;
        const vfloat<K> tFarY  = (bmaxY - org.y) * ctx.rdir.y;
        const vfloat<K> tFarZ  = (bmaxZ - org.z) * ctx.rdir.z;
        const float round_down = 1.0f-2.0f*float(ulp); 
        const float round_up   = 1.0f+2.0f*float(ulp);
#if defined(__AVX2__)
        const vfloat<K> tNear  = maxi(maxi(tNearX,tNearY),maxi(tNearZ,vfloat<K>(ctx.rdir.w)));
        const vfloat<K> tFar   = mini(mini(tFarX,tFarY),mini(tFarZ,vfloat<K>(org.w)));
#else
        const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ctx.rdir.w));
        const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(org.w));
#endif

#if defined(__AVX512F__)
        const unsigned int maskN = ((unsigned int)1 << N)-1;
        const vbool<K> vmask   = le(maskN,round_down*tNear,round_up*tFar);        
#else
        const vbool<K> vmask   = round_down*tNear <= round_up*tFar;
#endif
        if (dist_update) dist  = select(vmask,min(tNear,dist),dist);
        return vmask;    
      }
    }

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================


    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect_co_soa(BVH* __restrict__ bvh, RayK<K> **input_rays, size_t numValidStreams, const RTCIntersectContext* context)
    {      
#if defined(__AVX__) 
      __aligned(64) StackItemMaskCoherent  stack[stackSizeSingle];  //!< stack of nodes 

      RayK<K>** __restrict__ input_packets = (RayK<K>**)input_rays;
      const size_t numOctantRays = K*numValidStreams;
      assert(numOctantRays <= MAX_RAYS);

      /* inactive rays should have been filtered out before */

      __aligned(64) struct Packet {
        Vec3vfK rdir;
        Vec3vfK org_rdir;
        vfloat<K> min_dist;
        vfloat<K> max_dist;
      } packet[MAX_RAYS/K];

      Vec3vfK   tmp_min_rdir(pos_inf); 
      Vec3vfK   tmp_max_rdir(neg_inf);
      vfloat<K> tmp_min_dist(pos_inf);
      vfloat<K> tmp_max_dist(neg_inf);

      size_t m_active = 0;
      const size_t numPackets = numValidStreams;

      for (size_t i=0; i<numPackets; i++) 
      {
        const vfloat<K> tnear  = input_packets[i]->tnear;
        const vfloat<K> tfar   = input_packets[i]->tfar;
        const vbool<K> m_valid = (tnear <= tfar) & (tnear >= 0.0f);
        m_active |= (size_t)movemask(m_valid) << (i*K);
        packet[i].min_dist = max(tnear,0.0f);
        packet[i].max_dist = select(m_valid,tfar,neg_inf);
        tmp_min_dist = min(tmp_min_dist,packet[i].min_dist);
        tmp_max_dist = max(tmp_max_dist,packet[i].max_dist);        

        const Vec3vfK& org     = input_packets[i]->org;
        const Vec3vfK& dir     = input_packets[i]->dir;
        const Vec3vfK rdir     = rcp_safe(dir);
        const Vec3vfK org_rdir = org * rdir;
        
        packet[i].rdir     = rdir;
        packet[i].org_rdir = org_rdir;

        tmp_min_rdir = min(tmp_min_rdir,select(m_valid,rdir,Vec3vfK(pos_inf)));
        tmp_max_rdir = max(tmp_max_rdir,select(m_valid,rdir,Vec3vfK(neg_inf)));
      }

      if (unlikely(m_active == 0)) return;
        
      const Vec3fa reduced_min_rdir( reduce_min(tmp_min_rdir.x), 
                                     reduce_min(tmp_min_rdir.y),
                                     reduce_min(tmp_min_rdir.z) );

      const Vec3fa reduced_max_rdir( reduce_max(tmp_max_rdir.x), 
                                     reduce_max(tmp_max_rdir.y),
                                     reduce_max(tmp_max_rdir.z) );

      /* fallback for different direction signs */
      if (unlikely(sign(reduced_min_rdir) != sign(reduced_max_rdir))) //todo: maybe just disable IA test
      {
        DBG_PRINT("FALLBACK");
        for (size_t i=0; i<numPackets; i++) 
          bvh->scene->intersect(input_packets[i]->tnear <= input_packets[i]->tfar,*input_packets[i],context);
        return;
      }

      const float frusta_min_dist = reduce_min(tmp_min_dist);
      const float frusta_max_dist = reduce_min(tmp_max_dist);

      DBG_PRINT(frusta_min_dist);
      DBG_PRINT(frusta_max_dist);


      const Vec3fa frusta_min_rdir = select(ge_mask(reduced_min_rdir,Vec3fa(zero)),reduced_min_rdir,reduced_max_rdir);
      const Vec3fa frusta_max_rdir = select(ge_mask(reduced_min_rdir,Vec3fa(zero)),reduced_max_rdir,reduced_min_rdir);

      const Vec3fa origin(input_packets[0]->org.x[0],
                          input_packets[0]->org.y[0],
                          input_packets[0]->org.z[0]);

      const Vec3fa frusta_min_org_rdir = frusta_min_rdir * origin;
      const Vec3fa frusta_max_org_rdir = frusta_max_rdir * origin;

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;
      stack[0].childID = (unsigned int)-1;
      stack[0].dist    = (unsigned int)-1;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      const NearFarPreCompute pc(frusta_min_rdir);

      StackItemMaskCoherent* stackPtr   = stack + 1;

      while (1) pop:
      {          

        if (unlikely(stackPtr == stack)) break;

        DBG_PRINT("POP");
        STAT3(normal.trav_stack_pop,1,1,1);                          
        stackPtr--;
        /*! pop next node */
        NodeRef cur = NodeRef(stackPtr->child);
        size_t m_trav_active = stackPtr->mask;
        assert(m_trav_active);
        DBG_PRINT(std::bitset<64>(m_trav_active));

        //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          
#if defined(__AVX512F__)
        const vbool<K> maskN = ((unsigned int)1 << N)-1;
#endif
            
        /* non-root and leaf => full culling test for all rays */
        if (unlikely(stackPtr->parent != 0 && cur.isLeaf()))
        {
          DBG_PRINT("cull leaf after pop");
          DBG_PRINT(std::bitset<64>(m_trav_active));

          NodeRef parent = NodeRef(stackPtr->parent);
          const Node* __restrict__ const node = parent.node();
          const size_t b   = stackPtr->childID;
          char *ptr = (char*)&node->lower_x + b*sizeof(float);
          assert(cur == node->child(b));

          const vfloat<K> minX = vfloat<K>(*(float*)((const char*)ptr+pc.nearX));
          const vfloat<K> minY = vfloat<K>(*(float*)((const char*)ptr+pc.nearY));
          const vfloat<K> minZ = vfloat<K>(*(float*)((const char*)ptr+pc.nearZ));
          const vfloat<K> maxX = vfloat<K>(*(float*)((const char*)ptr+pc.farX));
          const vfloat<K> maxY = vfloat<K>(*(float*)((const char*)ptr+pc.farY));
          const vfloat<K> maxZ = vfloat<K>(*(float*)((const char*)ptr+pc.farZ));
          
          const size_t startPacketID = __bsf(m_trav_active) / K;
          const size_t endPacketID   = __bsr(m_trav_active) / K;

          m_trav_active = 0;
          for (size_t i=startPacketID;i<=endPacketID;i++)
          {
            STAT3(normal.trav_nodes,1,1,1);                          
            Packet &p = packet[i]; 
            const vfloat<K> tminX = msub(minX, p.rdir.x, p.org_rdir.x);
            const vfloat<K> tminY = msub(minY, p.rdir.y, p.org_rdir.y);
            const vfloat<K> tminZ = msub(minZ, p.rdir.z, p.org_rdir.z);
            const vfloat<K> tmaxX = msub(maxX, p.rdir.x, p.org_rdir.x);
            const vfloat<K> tmaxY = msub(maxY, p.rdir.y, p.org_rdir.y);
            const vfloat<K> tmaxZ = msub(maxZ, p.rdir.z, p.org_rdir.z);
            const vfloat<K> tmin  = maxi(maxi(tminX,tminY),maxi(tminZ,p.min_dist)); 
            const vfloat<K> tmax  = mini(mini(tmaxX,tmaxY),mini(tmaxZ,p.max_dist)); 
            const vbool<K> vmask   = tmin <= tmax;  
            const size_t m_hit = movemask(vmask);
            m_trav_active |= m_hit << (i*K);
          } 
          DBG_PRINT(std::bitset<64>(m_trav_active));

          if (m_trav_active == 0) goto pop;
        }

        while (1)
        {
          if (unlikely(cur.isLeaf())) break;
          const Node* __restrict__ const node = cur.node();

          DBG_PRINT("TRAVERSAL");
          DBG_PRINT(__popcnt(m_trav_active));

          __aligned(64) size_t maskK[N];
          //const vlong4 initMaskK(m_trav_active);
          //vlong4::store(&maskK[0],initMaskK);
          //vlong4::store(&maskK[4],initMaskK);

          for (size_t i=0;i<N;i++) maskK[i] = m_trav_active; 

          /* interval-based culling test */
          STAT3(normal.trav_nodes,1,1,1);                          

          const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearX));
          const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearY));
          const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearZ));
          const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farX));
          const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farY));
          const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farZ));

          const vfloat<K> fminX = msub(bminX, vfloat<K>(frusta_min_rdir.x), vfloat<K>(frusta_min_org_rdir.x));
          const vfloat<K> fminY = msub(bminY, vfloat<K>(frusta_min_rdir.y), vfloat<K>(frusta_min_org_rdir.y));
          const vfloat<K> fminZ = msub(bminZ, vfloat<K>(frusta_min_rdir.z), vfloat<K>(frusta_min_org_rdir.z));
          const vfloat<K> fmaxX = msub(bmaxX, vfloat<K>(frusta_max_rdir.x), vfloat<K>(frusta_max_org_rdir.x));
          const vfloat<K> fmaxY = msub(bmaxY, vfloat<K>(frusta_max_rdir.y), vfloat<K>(frusta_max_org_rdir.y));
          const vfloat<K> fmaxZ = msub(bmaxZ, vfloat<K>(frusta_max_rdir.z), vfloat<K>(frusta_max_org_rdir.z));
          const vfloat<K> fmin  = maxi(maxi(fminX,fminY),maxi(fminZ,frusta_min_dist)); 
          const vfloat<K> fmax  = mini(mini(fmaxX,fmaxY),mini(fmaxZ,frusta_max_dist)); 
#if defined(__AVX512F__)
          const vbool<K> vmask_node_hit = le(maskN,fmin,fmax);        
#else
          const vbool<K> vmask_node_hit = fmin <= fmax;  
#endif

          size_t m_node_hit = movemask(vmask_node_hit);

          // ==================
          const size_t first_index    = __bsf(m_trav_active);
          const size_t first_packetID = first_index / K;
          const size_t first_rayID    = first_index % K;

          Packet &p = packet[first_packetID]; 
          STAT3(normal.trav_nodes,1,1,1);                          

          const vfloat<K> rminX = msub(bminX, vfloat<K>(p.rdir.x[first_rayID]), vfloat<K>(p.org_rdir.x[first_rayID]));
          const vfloat<K> rminY = msub(bminY, vfloat<K>(p.rdir.y[first_rayID]), vfloat<K>(p.org_rdir.y[first_rayID]));
          const vfloat<K> rminZ = msub(bminZ, vfloat<K>(p.rdir.z[first_rayID]), vfloat<K>(p.org_rdir.z[first_rayID]));
          const vfloat<K> rmaxX = msub(bmaxX, vfloat<K>(p.rdir.x[first_rayID]), vfloat<K>(p.org_rdir.x[first_rayID]));
          const vfloat<K> rmaxY = msub(bmaxY, vfloat<K>(p.rdir.y[first_rayID]), vfloat<K>(p.org_rdir.y[first_rayID]));
          const vfloat<K> rmaxZ = msub(bmaxZ, vfloat<K>(p.rdir.z[first_rayID]), vfloat<K>(p.org_rdir.z[first_rayID]));
          const vfloat<K> rmin  = maxi(maxi(rminX,rminY),maxi(rminZ,p.min_dist[first_rayID])); 
          const vfloat<K> rmax  = mini(mini(rmaxX,rmaxY),mini(rmaxZ,p.max_dist[first_rayID])); 

#if defined(__AVX512F__)
          const vbool<K> vmask_first_hit = le(maskN,rmin,rmax);        
#else
          const vbool<K> vmask_first_hit = rmin <= rmax;  
#endif

          size_t m_first_hit = movemask(vmask_first_hit);

          // ==================

          assert(__popcnt(m_node_hit) <= 8 );
          
          vfloat<K> dist = select(vmask_first_hit,rmin,fmin);            
          //vfloat<K> dist = fmin;            

          DBG_PRINT(dist);

          size_t m_node = m_node_hit ^ (m_first_hit /*  & m_leaf */);
          while(unlikely(m_node)) //todo: iterate over nodes instead of packets?
          {
            const size_t b   = __bscf(m_node); // box
            size_t m_current = m_trav_active;
            assert(m_current);
            const vfloat<K> minX = vfloat<K>(bminX[b]);
            const vfloat<K> minY = vfloat<K>(bminY[b]);
            const vfloat<K> minZ = vfloat<K>(bminZ[b]);
            const vfloat<K> maxX = vfloat<K>(bmaxX[b]);
            const vfloat<K> maxY = vfloat<K>(bmaxY[b]);
            const vfloat<K> maxZ = vfloat<K>(bmaxZ[b]);

            const size_t startPacketID = __bsf(m_trav_active) / K;
            const size_t endPacketID   = __bsr(m_trav_active) / K;

            for (size_t i=startPacketID;i<=endPacketID;i++)
            {
              STAT3(normal.trav_nodes,1,1,1);                          
              assert(i < numPackets);
              Packet &p = packet[i]; 
              const vfloat<K> tminX = msub(minX, p.rdir.x, p.org_rdir.x);
              const vfloat<K> tminY = msub(minY, p.rdir.y, p.org_rdir.y);
              const vfloat<K> tminZ = msub(minZ, p.rdir.z, p.org_rdir.z);
              const vfloat<K> tmaxX = msub(maxX, p.rdir.x, p.org_rdir.x);
              const vfloat<K> tmaxY = msub(maxY, p.rdir.y, p.org_rdir.y);
              const vfloat<K> tmaxZ = msub(maxZ, p.rdir.z, p.org_rdir.z);
              const vfloat<K> tmin  = maxi(maxi(tminX,tminY),maxi(tminZ,p.min_dist)); 
              const vfloat<K> tmax  = mini(mini(tmaxX,tmaxY),mini(tmaxZ,p.max_dist)); 
              const vbool<K> vmask = tmin <= tmax;  
              const size_t m_hit = movemask(vmask);
              m_current &= ~((m_hit^(((size_t)1 << K)-1)) << (i*K));
            } 

            m_node_hit ^= m_current ? (size_t)0 : ((size_t)1 << b);
            maskK[b] = m_current;
          }

          //STAT3(normal.trav_hit_boxes[__popcnt(movemask(vmask))],1,1,1);                          

          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserKHitCoherent<types,N,K>::traverseClosestHit(cur, m_trav_active, vbool<K>((int)m_node_hit), dist, (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        DBG_PRINT("leaf");
        DBG_PRINT(__popcnt(m_trav_active));

        size_t bits = m_trav_active;
#if 1
        /*! intersect stream of rays with all primitives */
        size_t lazy_node = 0;
        size_t valid_isec = 0;
        STAT_USER(1,(__popcnt(bits)+K-1)/K*4);
        STAT_USER(2,(__popcnt(bits)+1)/2);
        STAT_USER(3,(__popcnt(bits)+3)/4);
        do
        {
          size_t i = __bsf(bits) / K;
          const size_t m_isec = ((((size_t)1 << K)-1) << (i*K));
          assert(m_isec & bits);
          bits &= ~m_isec;


          vbool<K> m_valid = (input_packets[i]->tnear <= input_packets[i]->tfar); 
          PrimitiveIntersector::intersectChunk(m_valid,*input_packets[i],context,prim,num,bvh->scene,lazy_node);
          Packet &p = packet[i]; 
          valid_isec |= movemask((input_packets[i]->tfar < p.max_dist) & m_valid); 
          p.max_dist = min(p.max_dist,input_packets[i]->tfar);
          //p.max_dist = select(m_valid,input_packets[i]->tfar,p.max_dist);
        } while(bits);


#endif
#if 0
        /* update tfar in ray context on successful hit */
        if(unlikely(valid_isec))
        {
          vfloat<K> max_dist(neg_inf);
          for (size_t i=0; i<numPackets; i++) 
          {
            Packet &p = packet[i]; 
            max_dist = max(max_dist,p.max_dist);            
          }
          frusta_max_dist = reduce_max(max_dist);
        }
#endif



      } // traversal + intersection
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

#endif
    }

    template<class T>
    __forceinline T ia_mul_min(const T &a0, const T &a1,
                               const T &b0, const T &b1)
    {
      return min(min(a0*b0,a0*b1),min(a1*b0,a1*b1));
    }

    template<class T>
    __forceinline T ia_mul_max(const T &a0, const T &a1,
                               const T &b0, const T &b1)
    {
      return max(max(a0*b0,a0*b1),max(a1*b0,a1*b1));
    }


    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect_co(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, const RTCIntersectContext* context)
    {
#if defined(__AVX__) 
      __aligned(64) Precalculations pre[MAX_RAYS]; 
      __aligned(64) StackItemMaskCoherent  stack[stackSizeSingle];  //!< stack of nodes 

      __aligned(64) struct Packet {
        Vec3vfK rdir;
        Vec3vfK org_rdir;
        vfloat<K> min_dist;
        vfloat<K> max_dist;
      } packet[MAX_RAYS/K];

      Ray** __restrict__ rays = input_rays;
      const size_t numOctantRays = numTotalRays;

      /* inactive rays should have been filtered out before */
      assert(numOctantRays <= MAX_RAYS);
      size_t m_active = numOctantRays == 8*sizeof(size_t) ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;

      if (unlikely(m_active == 0)) return;

      /* do per ray precalculations */
      Vec3fa tmp_min_rdir(pos_inf); // todo: avx min/max
      Vec3fa tmp_max_rdir(neg_inf);
      Vec3fa tmp_min_origin(pos_inf); // todo: avx min/max
      Vec3fa tmp_max_origin(neg_inf);

      float  frusta_min_dist(pos_inf);
      float  frusta_max_dist(neg_inf);

      //const size_t numPackets = (numOctantRays+K-1)/K;
      const size_t lastPacket = (numOctantRays-1)/K;

      /* fill potentially partly filled packet */
      packet[lastPacket].min_dist = 0.0f;
      packet[lastPacket].max_dist = neg_inf;

      /* compute IA frusta */
      for (size_t i=0; i<numOctantRays; i++) {
        new (&pre[i]) Precalculations(*rays[i],bvh);
        const Vec3fa& org = rays[i]->org;
        const Vec3fa& dir = rays[i]->dir;
        const Vec3fa rdir = rcp_safe(dir);
        const Vec3fa org_rdir = org * rdir;
        const float tnear = max(0.0f,rays[i]->tnear);
        const float tfar  = rays[i]->tfar;
        tmp_min_origin    = min(tmp_min_origin,org);
        tmp_max_origin    = max(tmp_max_origin,org);                      
        tmp_min_rdir      = min(tmp_min_rdir,rdir);
        tmp_max_rdir      = max(tmp_max_rdir,rdir);                      
        frusta_min_dist   = min(frusta_min_dist,tnear);
        frusta_max_dist   = max(frusta_max_dist,tfar);
        
        const size_t packetID = i / K;
        const size_t slotID   = i % K;

        packet[packetID].rdir.x[slotID]     = rdir.x;
        packet[packetID].rdir.y[slotID]     = rdir.y;
        packet[packetID].rdir.z[slotID]     = rdir.z;
        packet[packetID].org_rdir.x[slotID] = org_rdir.x;
        packet[packetID].org_rdir.y[slotID] = org_rdir.y;
        packet[packetID].org_rdir.z[slotID] = org_rdir.z;
        packet[packetID].min_dist[slotID]   = tnear;
        packet[packetID].max_dist[slotID]   = tfar;
      }

      const Vec3fa frusta_min_rdir = select(ge_mask(tmp_min_rdir,Vec3fa(zero)),tmp_min_rdir,tmp_max_rdir);
      const Vec3fa frusta_max_rdir = select(ge_mask(tmp_min_rdir,Vec3fa(zero)),tmp_max_rdir,tmp_min_rdir);

      const Vec3fa frusta_min_org_rdir = frusta_min_rdir*select(ge_mask(tmp_min_rdir,Vec3fa(zero)),tmp_max_origin,tmp_min_origin);
      const Vec3fa frusta_max_org_rdir = frusta_max_rdir*select(ge_mask(tmp_min_rdir,Vec3fa(zero)),tmp_min_origin,tmp_max_origin);

#if 0
      PRINT(tmp_min_rdir*tmp_min_origin);
      PRINT(tmp_min_rdir*tmp_max_origin);
      PRINT(tmp_max_rdir*tmp_min_origin);
      PRINT(tmp_max_rdir*tmp_max_origin);

      PRINT(tmp_min_origin);
      PRINT(tmp_max_origin);

      PRINT(frusta_min_org_rdir);
      PRINT(frusta_max_org_rdir);
      PRINT(ia_mul_min(frusta_min_rdir,frusta_max_rdir,tmp_min_origin,tmp_max_origin));
      PRINT(ia_mul_max(frusta_min_rdir,frusta_max_rdir,tmp_min_origin,tmp_max_origin));
      exit(0);
#endif

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;
      stack[0].childID = (unsigned int)-1;
      stack[0].dist    = (unsigned int)-1;


      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      const NearFarPreCompute pc(frusta_min_rdir);

      StackItemMaskCoherent* stackPtr   = stack + 1;


      while (1) pop:
      {          

        if (unlikely(stackPtr == stack)) break;

        DBG_PRINT("POP");
        STAT3(normal.trav_stack_pop,1,1,1);                          
        stackPtr--;
        /*! pop next node */
        NodeRef cur = NodeRef(stackPtr->child);
        size_t m_trav_active = stackPtr->mask;
        assert(m_trav_active);

        //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

        /* non-root and leaf => full culling test for all rays */
        if (unlikely(stackPtr->parent != 0 && cur.isLeaf()))
        {
          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          
          NodeRef parent = NodeRef(stackPtr->parent);
          const Node* __restrict__ const node = parent.node();
          const size_t b   = stackPtr->childID;
          char *ptr = (char*)&node->lower_x + b*sizeof(float);
          assert(cur == node->child(b));

          const vfloat<K> minX = vfloat<K>(*(float*)((const char*)ptr+pc.nearX));
          const vfloat<K> minY = vfloat<K>(*(float*)((const char*)ptr+pc.nearY));
          const vfloat<K> minZ = vfloat<K>(*(float*)((const char*)ptr+pc.nearZ));
          const vfloat<K> maxX = vfloat<K>(*(float*)((const char*)ptr+pc.farX));
          const vfloat<K> maxY = vfloat<K>(*(float*)((const char*)ptr+pc.farY));
          const vfloat<K> maxZ = vfloat<K>(*(float*)((const char*)ptr+pc.farZ));
          
          const size_t startPacketID = __bsf(m_trav_active) / K;
          const size_t endPacketID   = __bsr(m_trav_active) / K;

          m_trav_active = 0;
          for (size_t i=startPacketID;i<=endPacketID;i++)
          {
            STAT3(normal.trav_nodes,1,1,1);                          
            Packet &p = packet[i]; 
            const vfloat<K> tminX = msub(minX, p.rdir.x, p.org_rdir.x);
            const vfloat<K> tminY = msub(minY, p.rdir.y, p.org_rdir.y);
            const vfloat<K> tminZ = msub(minZ, p.rdir.z, p.org_rdir.z);
            const vfloat<K> tmaxX = msub(maxX, p.rdir.x, p.org_rdir.x);
            const vfloat<K> tmaxY = msub(maxY, p.rdir.y, p.org_rdir.y);
            const vfloat<K> tmaxZ = msub(maxZ, p.rdir.z, p.org_rdir.z);
            const vfloat<K> tmin  = maxi(maxi(tminX,tminY),maxi(tminZ,p.min_dist)); 
            const vfloat<K> tmax  = mini(mini(tmaxX,tmaxY),mini(tmaxZ,p.max_dist)); 
            const vbool<K> vmask   = tmin <= tmax;  
            const size_t m_hit = movemask(vmask);
            m_trav_active |= m_hit << (i*K);
          } 
          if (m_trav_active == 0) goto pop;
        }

        while (1)
        {
          if (unlikely(cur.isLeaf())) break;
          const Node* __restrict__ const node = cur.node();

          DBG_PRINT("TRAVERSAL");
          DBG_PRINT(__popcnt(m_trav_active));

          __aligned(64) size_t maskK[N];
          //const vlong4 initMaskK(m_trav_active);
          //vlong4::store(&maskK[0],initMaskK);
          //vlong4::store(&maskK[4],initMaskK);

          for (size_t i=0;i<N;i++) maskK[i] = m_trav_active; 

          /* interval-based culling test */
          STAT3(normal.trav_nodes,1,1,1);                          

          const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearX));
          const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearY));
          const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearZ));
          const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farX));
          const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farY));
          const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farZ));

          const vfloat<K> fminX = msub(bminX, vfloat<K>(frusta_min_rdir.x), vfloat<K>(frusta_min_org_rdir.x));
          const vfloat<K> fminY = msub(bminY, vfloat<K>(frusta_min_rdir.y), vfloat<K>(frusta_min_org_rdir.y));
          const vfloat<K> fminZ = msub(bminZ, vfloat<K>(frusta_min_rdir.z), vfloat<K>(frusta_min_org_rdir.z));
          const vfloat<K> fmaxX = msub(bmaxX, vfloat<K>(frusta_max_rdir.x), vfloat<K>(frusta_max_org_rdir.x));
          const vfloat<K> fmaxY = msub(bmaxY, vfloat<K>(frusta_max_rdir.y), vfloat<K>(frusta_max_org_rdir.y));
          const vfloat<K> fmaxZ = msub(bmaxZ, vfloat<K>(frusta_max_rdir.z), vfloat<K>(frusta_max_org_rdir.z));
          const vfloat<K> fmin  = maxi(maxi(fminX,fminY),maxi(fminZ,frusta_min_dist)); 
          const vfloat<K> fmax  = mini(mini(fmaxX,fmaxY),mini(fmaxZ,frusta_max_dist)); 
          const vbool<K> vmask_node_hit  = fmin <= fmax;  
          size_t m_node_hit = movemask(vmask_node_hit);

          //STAT3(normal.trav_hit_boxes[__popcnt(m_node_hit)],1,1,1);                          
            
          // ==================
          const size_t first_index    = __bsf(m_trav_active);
          const size_t first_packetID = first_index / K;
          const size_t first_rayID    = first_index % K;

          Packet &p = packet[first_packetID]; 
          STAT3(normal.trav_nodes,1,1,1);                          

          const vfloat<K> rminX = msub(bminX, vfloat<K>(p.rdir.x[first_rayID]), vfloat<K>(p.org_rdir.x[first_rayID]));
          const vfloat<K> rminY = msub(bminY, vfloat<K>(p.rdir.y[first_rayID]), vfloat<K>(p.org_rdir.y[first_rayID]));
          const vfloat<K> rminZ = msub(bminZ, vfloat<K>(p.rdir.z[first_rayID]), vfloat<K>(p.org_rdir.z[first_rayID]));
          const vfloat<K> rmaxX = msub(bmaxX, vfloat<K>(p.rdir.x[first_rayID]), vfloat<K>(p.org_rdir.x[first_rayID]));
          const vfloat<K> rmaxY = msub(bmaxY, vfloat<K>(p.rdir.y[first_rayID]), vfloat<K>(p.org_rdir.y[first_rayID]));
          const vfloat<K> rmaxZ = msub(bmaxZ, vfloat<K>(p.rdir.z[first_rayID]), vfloat<K>(p.org_rdir.z[first_rayID]));
          const vfloat<K> rmin  = maxi(maxi(rminX,rminY),maxi(rminZ,p.min_dist[first_rayID])); 
          const vfloat<K> rmax  = mini(mini(rmaxX,rmaxY),mini(rmaxZ,p.max_dist[first_rayID])); 
          const vbool<K> vmask_first_hit  = rmin <= rmax;  
          size_t m_first_hit = movemask(vmask_first_hit);

          // ==================

          assert(__popcnt(m_node_hit) <= 8 );
          
          vfloat<K> dist = select(vmask_first_hit,rmin,fmin);            
          //vfloat<K> dist = fmin;            

          DBG_PRINT(dist);

          size_t m_node = m_node_hit ^ (m_first_hit /*  & m_leaf */);

          while(unlikely(m_node)) //todo: iterate over nodes instead of packets?
          {
            const size_t b   = __bscf(m_node); // box
            size_t m_current = m_trav_active;
            assert(m_current);
            const vfloat<K> minX = vfloat<K>(bminX[b]);
            const vfloat<K> minY = vfloat<K>(bminY[b]);
            const vfloat<K> minZ = vfloat<K>(bminZ[b]);
            const vfloat<K> maxX = vfloat<K>(bmaxX[b]);
            const vfloat<K> maxY = vfloat<K>(bmaxY[b]);
            const vfloat<K> maxZ = vfloat<K>(bmaxZ[b]);

            const size_t startPacketID = __bsf(m_trav_active) / K;
            const size_t endPacketID   = __bsr(m_trav_active) / K;

            for (size_t i=startPacketID;i<=endPacketID;i++)
            {
              STAT3(normal.trav_nodes,1,1,1);                          
              Packet &p = packet[i]; 
              const vfloat<K> tminX = msub(minX, p.rdir.x, p.org_rdir.x);
              const vfloat<K> tminY = msub(minY, p.rdir.y, p.org_rdir.y);
              const vfloat<K> tminZ = msub(minZ, p.rdir.z, p.org_rdir.z);
              const vfloat<K> tmaxX = msub(maxX, p.rdir.x, p.org_rdir.x);
              const vfloat<K> tmaxY = msub(maxY, p.rdir.y, p.org_rdir.y);
              const vfloat<K> tmaxZ = msub(maxZ, p.rdir.z, p.org_rdir.z);
              const vfloat<K> tmin  = maxi(maxi(tminX,tminY),maxi(tminZ,p.min_dist)); 
              const vfloat<K> tmax  = mini(mini(tmaxX,tmaxY),mini(tmaxZ,p.max_dist)); 
              const vbool<K> vmask   = tmin <= tmax;  
              const size_t m_hit = movemask(vmask);
              m_current &= ~((m_hit^(((size_t)1 << K)-1)) << (i*K));
            } 

            m_node_hit ^= m_current ? (size_t)0 : ((size_t)1 << b);
            maskK[b] = m_current;
          }

          //STAT3(normal.trav_hit_boxes[__popcnt(movemask(vmask))],1,1,1);                          

          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserKHitCoherent<types,N,K>::traverseClosestHit(cur, m_trav_active, vbool<K>((int)m_node_hit), dist, (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        DBG_PRINT("leaf");
        DBG_PRINT(__popcnt(m_trav_active));

        //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          
        size_t bits = m_trav_active;

        /*! intersect stream of rays with all primitives */
        size_t lazy_node = 0;
        size_t valid_isec MAYBE_UNUSED = PrimitiveIntersector::intersect(pre,bits,rays,context,0,prim,num,bvh->scene,NULL,lazy_node);

        
        /* update tfar in ray context on successful hit */
        if(unlikely(valid_isec))
        {
          do {
            const size_t i = __bscf(valid_isec);
            const size_t packetID = i / K;
            const size_t slotID   = i % K;
            packet[packetID].max_dist[slotID] = rays[i]->tfar;
          } while(valid_isec);

#if 0
          /* update max frust distance */
          vfloat<K> max_dist = vfloat<K>::load(&rays_max_dist[0]);
          for (size_t i=K;i<MAX_RAYS;i++)
            max_dist = max(max_dist,vfloat<K>::load(&rays_max_dist[i]));
          frusta_max_dist = reduce_max(max_dist);
#endif
        }


      } // traversal + intersection

        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

#endif
    }
    
    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================

    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, const RTCIntersectContext* context)
    {
      __aligned(64) RContext ray_ctx[MAX_RAYS_PER_OCTANT];
      __aligned(64) Precalculations pre[MAX_RAYS_PER_OCTANT]; 
      __aligned(64) StackItemMask  stack0[stackSizeSingle];  //!< stack of nodes 
#if TWO_STREAMS_FIBER_MODE
      __aligned(64) StackItemMask  stack1[stackSizeSingle];  //!< stack of nodes 
#endif

#if defined(__AVX__) && ENABLE_CO_PATH == 1
      if (unlikely(isCoherent(context->flags)))
      {
        //BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect_co_soa(bvh, (RayK<K> **)input_rays, numTotalRays, context);        
        BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect_co(bvh, input_rays, numTotalRays, context);        
        return;
      }
#endif
      
      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** __restrict__ rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays == 8*sizeof(size_t) ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;

        if (m_active == 0) return;

        /* do per ray precalculations */
        for (size_t i=0; i<numOctantRays; i++) {
          new (&ray_ctx[i]) RContext(rays[i]);
          new (&pre[i]) Precalculations(*rays[i],bvh);
        }

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

#if TWO_STREAMS_FIBER_MODE
        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;
#endif

#if DISTANCE_TEST == 1
        stack0[0].dist = 0;
        stack1[0].dist = 0;
#endif
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
#if TWO_STREAMS_FIBER_MODE
        StackItemMask* stackPtr_next = stack1 + 1;
#endif

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
            //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);
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
              const RContext &ray = ray_ctx[i];
              const vlong<K/2> bitmask  = one << vlong<K/2>(i);

              const vbool<K> vmask = intersectNode<K,true,robust>(ray,bminmaxX,bminmaxY,bminmaxZ,dist);

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
            const RContext *__restrict__ const cur_ray_ctx = ray_ctx;
#else
            const RContext *__restrict__ const cur_ray_ctx = &ray_ctx[cur_fiber->getOffset()];
#endif
            size_t bits = m_trav_active;
            do
            {            
              STAT3(normal.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              const RContext &ray = cur_ray_ctx[i];
              const vint<K> bitmask  = vint<K>((int)1 << i);
              const vbool<K> vmask = intersectNode<K,true, robust> (ray,bminX,bminY,bminZ,bmaxX,bmaxY,bmaxZ,dist);

#if defined(__AVX2__)
              maskK = maskK | (bitmask & vint<K>(vmask));
#else
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
          
          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          
#if !TWO_STREAMS_FIBER_MODE
          size_t bits = m_trav_active;
#else
          size_t bits = m_trav_active << cur_fiber->getOffset();
#endif

          /*! intersect stream of rays with all primitives */
          size_t lazy_node = 0;
          size_t valid_isec MAYBE_UNUSED = PrimitiveIntersector::intersect(pre,bits,rays,context,0,prim,num,bvh->scene,NULL,lazy_node);

          /* update tfar in ray context on successful hit */
          size_t isec_bits = valid_isec;
          while(isec_bits)
          {
            const size_t i = __bscf(isec_bits);
            ray_ctx[i].update(rays[i]);
          }

#if DISTANCE_TEST == 1
          if (unlikely(valid_isec))
          {
            StackItemMask *new_sptr = &stack0[1];
            for (StackItemMask *sptr = new_sptr;sptr!=stackPtr;sptr++)
            {
              assert(sptr < stackPtr);
              size_t mask = sptr->mask;
              size_t bits = mask & valid_isec;
              while (bits) {
                const size_t i = __bscf(bits);
                const RContext &ray = ray_ctx[i];
                const size_t mask_i = (sptr->dist > ray.tfar_ui()) ? ((size_t)1 << i) : 0;
                mask &= ~mask_i;
              };
              if (!mask) continue;
              new_sptr->ptr  = sptr->ptr; 
              new_sptr->mask = mask; 
              new_sptr->dist = sptr->dist;   
              new_sptr++;
            }
            stackPtr = new_sptr;
          }
#endif
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
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded(BVH* __restrict__ bvh, Ray **input_rays, size_t numTotalRays, const RTCIntersectContext* context)
    {
      __aligned(64) RContext ray_ctx[MAX_RAYS_PER_OCTANT];
      __aligned(64) Precalculations pre[MAX_RAYS_PER_OCTANT]; 
      __aligned(64) StackItemMask  stack0[stackSizeSingle];  //!< stack of nodes 
#if TWO_STREAMS_FIBER_MODE
      __aligned(64) StackItemMask  stack1[stackSizeSingle];  //!< stack of nodes
 #endif

      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;

        /* inactive rays should have been filtered out before */
        size_t m_active = numOctantRays ==  8*sizeof(size_t) ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;

        if (m_active == 0) return;

        /* do per ray precalculations */
        for (size_t i=0; i<numOctantRays; i++) {
          new (&ray_ctx[i]) RContext(rays[i]);
          new (&pre[i]) Precalculations(*rays[i],bvh);
        }

        stack0[0].ptr  = BVH::invalidNode;
        stack0[0].mask = (size_t)-1;

#if TWO_STREAMS_FIBER_MODE
        stack1[0].ptr  = BVH::invalidNode;
        stack1[0].mask = (size_t)-1;
#endif

#if !TWO_STREAMS_FIBER_MODE
        const size_t fiberMask = m_active;
#else
        const size_t fiberMask = ((size_t)1 << ((__popcnt(m_active)+1)>>1))-1;
        assert( ((fiberMask | (~fiberMask)) & m_active) == m_active);
        assert( __popcnt(fiberMask) + __popcnt((~fiberMask) & m_active) == __popcnt(m_active));
#endif
        assert(fiberMask);
        
        StackItemMask* stackPtr      = stack0 + 1;
#if TWO_STREAMS_FIBER_MODE
        StackItemMask* stackPtr_next = stack1 + 1;
#endif

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
            //STAT3(shadow.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);

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
              RContext &ray = ray_ctx[i];
              const vlong<K/2> bitmask  = one << vlong<K/2>(i);
              const vbool<K> vmask = intersectNode<K,false,robust>(ray,bminmaxX,bminmaxY,bminmaxZ,dist);
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
            const RContext *__restrict__ const cur_ray_ctx = ray_ctx;
#else
            const RContext *__restrict__ const cur_ray_ctx = &ray_ctx[cur_fiber->getOffset()];
#endif
            size_t bits = m_trav_active;

            assert(__popcnt(m_trav_active) <= 32);
            do
            {            
              STAT3(shadow.trav_nodes,1,1,1);                          
              const size_t i = __bscf(bits);
              const RContext &ray = cur_ray_ctx[i];
              const vint<K> bitmask  = vint<K>((int)1 << i);

              const vbool<K> vmask = intersectNode<K,false,robust>(ray,bminX,bminY,bminZ,bmaxX,bmaxY,bmaxZ,dist);

#if defined(__AVX2__)
              maskK = maskK | (bitmask & vint<K>(vmask));
#else
              maskK = select(vmask,maskK | bitmask,maskK); 
#endif
            } while(bits);          
            const vbool<K> vmask = maskK != vint<K>(zero); 

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
              goto pop;
            }

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr); 

#endif

          }

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
          //STAT3(shadow.trav_hit_boxes[__popcnt(bits)],1,1,1);                          

          m_active &= ~PrimitiveIntersector::occluded(pre,bits,rays,context,0,prim,num,bvh->scene,NULL,lazy_node);
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



    IF_ENABLED_LINES(DEFINE_INTERSECTORN(BVH4Line4iStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<LineMiIntersector1<4 COMMA 4 COMMA true> > >));
    
    IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH4Bezier1vStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH4Bezier1iStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >));
    
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersectorMoeller,         BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4StreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA false> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4vStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<TriangleMvIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4iStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<Triangle4iIntersector1Pluecker<SIMD_MODE(4) COMMA true> > >));
    
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersectorMoeller,        BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4vStreamIntersectorMoellerNoFilter,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA false> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4iStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));
    
    IF_ENABLED_USER(DEFINE_INTERSECTORN(BVH4VirtualStreamIntersector,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN1 COMMA false COMMA ObjectIntersector1>));


    //IF_ENABLED_LINES(DEFINE_INTERSECTORN(BVH4Line4iMBStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<LineMiMBIntersector1<SIMD_MODE(4) COMMA true> > >));
    //IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH4Bezier1vStreamIntersector_OBB,BVHNStreamIntersector<4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >));
    //IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH4Bezier1iStreamIntersector_OBB,BVHNStreamIntersector<4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >));
    //IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH4Bezier1iMBStreamIntersector_OBB,BVHNStreamIntersector<4 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >));

    //IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH4Triangle4vMBStreamIntersectorMoeller,BVHNStreamIntersector<SIMD_MODE(4) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >));
    //IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH4Quad4iMBStreamIntersectorPluecker,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<QuadMiMBIntersector1Pluecker<4 COMMA true> > >));
    //IF_ENABLED_SUBDIV(DEFINE_INTERSECTORN(BVH4Subdivpatch1CachedStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector1>));
    //IF_ENABLED_SUBDIV(DEFINE_INTERSECTORN(BVH4GridAOSStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersector1>));
    //IF_ENABLED_USER(DEFINE_INTERSECTORN(BVH4VirtualMBStreamIntersector,BVHNStreamIntersector<4 COMMA BVH_AN2 COMMA false COMMA ObjectIntersector1>));
    
    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8IntersectorStream Definitions
    ////////////////////////////////////////////////////////////////////////////////
    
#if defined(__AVX__)
    
    // disabled for now
    //IF_ENABLED_LINES(DEFINE_INTERSECTORN(BVH8Line4iStreamIntersector,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<LineMiIntersector1<4 COMMA 4 COMMA true> > >));

#if !defined(__AVX512F__)
    typedef ArrayIntersectorKStream<8,
                                    TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true >,
                                    TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA true > > Triangle4IntersectorStreamMoellerTrumbore;
    typedef ArrayIntersectorKStream<8,
                                    TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA false >,
                                    TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA false > > Triangle4IntersectorStreamMoellerTrumboreNoFilter;
    typedef ArrayIntersectorKStream<8,
                                    QuadMvIntersector1MoellerTrumbore<4 COMMA true >,
                                    QuadMvIntersectorKMoellerTrumbore<4 COMMA 8 COMMA true > > Quad4vIntersectorStreamMoellerTrumbore;
    typedef ArrayIntersectorKStream<8,
                                    QuadMvIntersector1MoellerTrumbore<4 COMMA false >,
                                    QuadMvIntersectorKMoellerTrumbore<4 COMMA 8 COMMA false > > Quad4vIntersectorStreamMoellerTrumboreNoFilter;
    typedef ArrayIntersectorKStream<8,
                                    QuadMiIntersector1Pluecker<4 COMMA true >,
                                    QuadMiIntersectorKPluecker<4 COMMA 8 COMMA true > > Quad4iIntersectorStreamPluecker;
#else
    typedef ArrayIntersectorKStream<16,
                                    TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true >,
                                    TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA true > > Triangle4IntersectorStreamMoellerTrumbore;
    typedef ArrayIntersectorKStream<16,
                                    TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA false >,
                                    TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA false > > Triangle4IntersectorStreamMoellerTrumboreNoFilter;

    typedef ArrayIntersectorKStream<16,
                                    QuadMvIntersector1MoellerTrumbore<4 COMMA true >,
                                    QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true > > Quad4vIntersectorStreamMoellerTrumbore;
    typedef ArrayIntersectorKStream<16,
                                    QuadMvIntersector1MoellerTrumbore<4 COMMA false >,
                                    QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA false > > Quad4vIntersectorStreamMoellerTrumboreNoFilter;

    typedef ArrayIntersectorKStream<16,
                                    QuadMiIntersector1Pluecker<4 COMMA true >,
                                    QuadMiIntersectorKPluecker<4 COMMA 16 COMMA true > > Quad4iIntersectorStreamPluecker;

#endif
    
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorMoeller,         BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA Triangle4IntersectorStreamMoellerTrumbore>));
    IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA Triangle4IntersectorStreamMoellerTrumboreNoFilter>));

    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersectorMoeller,         BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA Quad4vIntersectorStreamMoellerTrumbore>));
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH8Quad4vStreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA Quad4vIntersectorStreamMoellerTrumboreNoFilter>));
    
    IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH8Quad4iStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA true COMMA Quad4iIntersectorStreamPluecker>));


    //IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH8Triangle4vMBStreamIntersectorMoeller,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >));

    //IF_ENABLED_TRIS(DEFINE_INTERSECTORN(BVH8Triangle4StreamIntersectorMoellerNoFilter, BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA false> > >));

    //IF_ENABLED_LINES(DEFINE_INTERSECTORN(BVH8Line4iMBStreamIntersector,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<LineMiMBIntersector1<SIMD_MODE(4) COMMA true> > >));
    
    //IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH8Bezier1vStreamIntersector_OBB,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >));
    //IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH8Bezier1iStreamIntersector_OBB,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >));
    //IF_ENABLED_HAIR(DEFINE_INTERSECTORN(BVH8Bezier1iMBStreamIntersector_OBB,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >));

    //IF_ENABLED_QUADS(DEFINE_INTERSECTORN(BVH8Quad4iMBStreamIntersectorPluecker,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<QuadMiMBIntersector1Pluecker<4 COMMA true> > >));
    
    //IF_ENABLED_SUBDIV(DEFINE_INTERSECTORN(BVH8GridAOSStreamIntersector,BVHNStreamIntersector<SIMD_MODE(8) COMMA BVH_AN1 COMMA true COMMA GridAOSIntersector1>));
    
#endif
  }
}



// ===================================================================================================================================================================
// ===================================================================================================================================================================
// ===================================================================================================================================================================
