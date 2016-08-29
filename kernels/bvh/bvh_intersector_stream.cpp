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

    template<int K>
    __forceinline size_t AOStoSOA(RayK<K> *rayK, Ray **input_rays, const size_t numTotalRays)
    {
      const size_t numPackets = (numTotalRays+K-1)/K; //todo : OPTIMIZE
      for (size_t i=0; i<numPackets; i++) 
      {
        rayK[i].tnear   = 0.0f;
        rayK[i].tfar    = neg_inf;
        rayK[i].time    = 0.0f;
        rayK[i].mask    = -1;
        rayK[i].geomID  = RTC_INVALID_GEOMETRY_ID;
      }

      Vec3fa min_dir = pos_inf;
      Vec3fa max_dir = neg_inf;

      for (size_t i=0; i<numTotalRays; i++) {
        const Vec3fa& org = input_rays[i]->org;
        const Vec3fa& dir = input_rays[i]->dir;
        min_dir = min(min_dir,dir);
        max_dir = max(max_dir,dir);
        const float tnear = max(0.0f,input_rays[i]->tnear);
        const float tfar  = input_rays[i]->tfar;
        const size_t packetID = i / K;
        const size_t slotID   = i % K;
        rayK[packetID].dir.x[slotID]     = dir.x;
        rayK[packetID].dir.y[slotID]     = dir.y;
        rayK[packetID].dir.z[slotID]     = dir.z;
        rayK[packetID].org.x[slotID]     = org.x;
        rayK[packetID].org.y[slotID]     = org.y;
        rayK[packetID].org.z[slotID]     = org.z;
        rayK[packetID].tnear[slotID]     = tnear;
        rayK[packetID].tfar[slotID]      = tfar;
        rayK[packetID].mask[slotID]      = input_rays[i]->mask;
        rayK[packetID].instID[slotID]    = input_rays[i]->instID;
      }      
      const size_t sign_min_dir = movemask(vfloat4(min_dir) < 0.0f);
      const size_t sign_max_dir = movemask(vfloat4(max_dir) < 0.0f);
      return ((sign_min_dir^sign_max_dir) & 0x7);
    }

    template<int K,bool occlusion>
    __forceinline void SOAtoAOS(Ray ** input_rays, RayK<K> *rayK, const size_t numTotalRays)
    {
      for (size_t i=0; i<numTotalRays; i++) 
      {
        const size_t packetID = i / K;
        const size_t slotID   = i % K;
        const RayK<K> &ray = rayK[packetID];
        if (likely((unsigned)ray.geomID[slotID] != RTC_INVALID_GEOMETRY_ID))
        {
          if (occlusion)
            input_rays[i]->geomID = ray.geomID[slotID];
          else
          {
            input_rays[i]->tfar   = ray.tfar[slotID];
            input_rays[i]->Ng.x   = ray.Ng.x[slotID];
            input_rays[i]->Ng.y   = ray.Ng.y[slotID];
            input_rays[i]->Ng.z   = ray.Ng.z[slotID];
            input_rays[i]->u      = ray.u[slotID];
            input_rays[i]->v      = ray.v[slotID];
            input_rays[i]->geomID = ray.geomID[slotID];
            input_rays[i]->primID = ray.primID[slotID];
            input_rays[i]->instID = ray.instID[slotID];
          }
        }

      }
    }

    // =====================================================================================================
    // =====================================================================================================
    // =====================================================================================================


    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect_coherent_soa(BVH* __restrict__ bvh, RayK<K> **input_rays, size_t numOctantRays, const RTCIntersectContext* context)
    {      
#if defined(__AVX__) 
      __aligned(64) StackItemMaskCoherent  stack[stackSizeSingle];  //!< stack of nodes 

      RayK<K>** __restrict__ input_packets = (RayK<K>**)input_rays;
      assert(numOctantRays <= MAX_RAYS);

      __aligned(64) Packet packet[MAX_RAYS/K];
      __aligned(64) Frusta frusta;

      const size_t m_active = initPacketsAndFrusta(input_packets,numOctantRays,packet,frusta);

      if (unlikely(m_active == 0)) return; 

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;
      stack[0].childID = (unsigned int)-1;
      stack[0].dist    = (unsigned int)-1;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      const NearFarPreCompute pc(frusta.min_rdir);

      StackItemMaskCoherent* stackPtr   = stack + 1;

      while (1) pop:
      {          

        if (unlikely(stackPtr == stack)) break;

        STAT3(normal.trav_stack_pop,1,1,1);                          
        stackPtr--;
        /*! pop next node */
        NodeRef cur = NodeRef(stackPtr->child);
        size_t m_trav_active = stackPtr->mask;
        assert(m_trav_active);
         
        /* non-root and leaf => full culling test for all rays */
        if (unlikely(stackPtr->parent != 0 && cur.isLeaf()))
        {
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

          m_trav_active = intersectNodePacket(packet,minX,minY,minZ,maxX,maxY,maxZ,m_trav_active);
          if (m_trav_active == 0) goto pop;
        }

        while (1)
        {
          if (unlikely(cur.isLeaf())) break;
          const Node* __restrict__ const node = cur.node();

          __aligned(64) size_t maskK[N];
          for (size_t i=0;i<N;i++) maskK[i] = m_trav_active; 
          vfloat<K> dist;
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packet, node, pc, frusta, maskK, dist);
          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserKHitCoherent<types,N,K>::traverseClosestHit(cur, m_trav_active, vbool<K>((int)m_node_hit), dist, (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        size_t bits = m_trav_active;

        /*! intersect stream of rays with all primitives */
        size_t lazy_node = 0;
        STAT_USER(1,(__popcnt(bits)+K-1)/K*4);
        do
        {
          size_t i = __bsf(bits) / K;
          const size_t m_isec = ((((size_t)1 << K)-1) << (i*K));
          assert(m_isec & bits);
          bits &= ~m_isec;

          vbool<K> m_valid = (input_packets[i]->tnear <= input_packets[i]->tfar); 
          PrimitiveIntersector::intersectChunk(m_valid,*input_packets[i],context,prim,num,bvh->scene,lazy_node);
          Packet &p = packet[i]; 
          p.max_dist = min(p.max_dist,input_packets[i]->tfar);
        } while(bits);

      } // traversal + intersection
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

#endif
    }



    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
    void BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded_coherent_soa(BVH* __restrict__ bvh, RayK<K> **input_rays, size_t numOctantRays, const RTCIntersectContext* context)
    {      
#if defined(__AVX__) 
      __aligned(64) StackItemMaskCoherent  stack[stackSizeSingle];  //!< stack of nodes 

      RayK<K>** __restrict__ input_packets = (RayK<K>**)input_rays;
      assert(numOctantRays <= MAX_RAYS);

      /* inactive rays should have been filtered out before */
      __aligned(64) Packet packet[MAX_RAYS/K];
      __aligned(64) Frusta frusta;

      size_t m_active = initPacketsAndFrusta(input_packets,numOctantRays,packet,frusta);

      /* valid rays */
      if (unlikely(m_active == 0)) return; 

      stack[0].mask    = m_active;
      stack[0].parent  = 0;
      stack[0].child   = bvh->root;
      stack[0].childID = (unsigned int)-1;
      stack[0].dist    = (unsigned int)-1;

      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////

      const NearFarPreCompute pc(frusta.min_rdir);

      StackItemMaskCoherent* stackPtr   = stack + 1;

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

        /* non-root and leaf => full culling test for all rays */
        if (unlikely(stackPtr->parent != 0 && cur.isLeaf()))
        {
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
          
          m_trav_active = intersectNodePacket(packet,minX,minY,minZ,maxX,maxY,maxZ,m_trav_active);

          if (m_trav_active == 0) goto pop;
        }

        while (1)
        {
          if (unlikely(cur.isLeaf())) break;
          const Node* __restrict__ const node = cur.node();

          __aligned(64) size_t maskK[N];
          for (size_t i=0;i<N;i++) maskK[i] = m_trav_active; 

          vfloat<K> dist;
          const size_t m_node_hit = traverseCoherentStream(m_trav_active, packet, node, pc, frusta, maskK, dist);

          if (unlikely(m_node_hit == 0)) goto pop;

          BVHNNodeTraverserKHitCoherent<types,N,K>::traverseAnyHit(cur, m_trav_active, vbool<K>((int)m_node_hit), (size_t*)maskK, stackPtr);
          assert(m_trav_active);
        }

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves, 1, 1, 1);
        size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

        size_t bits = m_trav_active & m_active;
        /*! intersect stream of rays with all primitives */
        size_t lazy_node = 0;
        STAT_USER(1,(__popcnt(bits)+K-1)/K*4);
        while(bits)
        {
          size_t i = __bsf(bits) / K;
          const size_t m_isec = ((((size_t)1 << K)-1) << (i*K));
          assert(m_isec & bits);
          bits &= ~m_isec;

          vbool<K> m_valid = (input_packets[i]->tnear <= input_packets[i]->tfar); 
          vbool<K> m_hit = PrimitiveIntersector::occludedChunk(m_valid,*input_packets[i],context,prim,num,bvh->scene,lazy_node);
          input_packets[i]->geomID = select(m_hit,vint<K>(zero),input_packets[i]->geomID);
          m_active &= ~((size_t)movemask(m_hit) << (i*K));
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
      __aligned(64) StackItemMask  stack[stackSizeSingle];  //!< stack of nodes 

#if ENABLE_COHERENT_STREAM_PATH == 1 

      if (unlikely(PrimitiveIntersector::validChunkIntersector && !robust && isCoherent(context->flags)))
      {
        /* AOS to SOA conversion */
        RayK<K> rayK[MAX_RAYS / K];
        RayK<K> *rayK_ptr[MAX_RAYS / K];
        for (size_t i=0;i<MAX_RAYS / K;i++) rayK_ptr[i] = &rayK[i];
        AOStoSOA(rayK,input_rays,numTotalRays);
        const size_t numPackets = (numTotalRays+K-1)/K;
        if (unlikely(numPackets == 1))
        {
          /* packet tracer as fallback */
          bvh->scene->intersect(rayK[0].tnear <= rayK[0].tfar,rayK[0],context);
        }
        else
        {          
          /* stream tracer as fast path */
          BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::intersect_coherent_soa(bvh,  (RayK<K>**)rayK_ptr, numTotalRays, context);     

        }
        /* SOA to AOS conversion */
        SOAtoAOS<K,false>(input_rays,rayK,numTotalRays);                    
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

        stack[0].ptr  = BVH::invalidNode;
        stack[0].mask = (size_t)-1;
        stack[1].ptr  = bvh->root;
        stack[1].mask = m_active;

        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////

        const NearFarPreCompute pc(ray_ctx[0].rdir);
        
        StackItemMask* stackPtr      = stack + 2;

        //NodeRef cur               = bvh->root;
        //size_t m_trav_active      = m_active; 

        while (1) pop:
        {          
          /*! pop next node */
          STAT3(normal.trav_stack_pop,1,1,1);                          
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);            
          size_t m_trav_active = stackPtr->mask;
          assert(m_trav_active);


          const vfloat<K> inf(pos_inf);

          while (1)
          {

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
            if (unlikely(none(vmask))) goto pop;

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

            const RContext *__restrict__ const cur_ray_ctx = ray_ctx;
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
            if (unlikely(none(vmask))) goto pop;

            BVHNNodeTraverserKHit<types,N,K>::traverseClosestHit(cur, m_trav_active, vmask, dist, (unsigned int*)&maskK, stackPtr);
            assert(m_trav_active);
#endif
          }

          /* current ray stream is done? */
          if (unlikely(cur == BVH::invalidNode))
            break;

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);
          
          //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          
          size_t bits = m_trav_active;

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
      __aligned(64) StackItemMask  stack[stackSizeSingle];  //!< stack of nodes 

#if ENABLE_COHERENT_STREAM_PATH == 1 
      if (unlikely(PrimitiveIntersector::validChunkIntersector && !robust && isCoherent(context->flags)))
      {

        /* AOS to SOA conversion */
        RayK<K> rayK[MAX_RAYS / K];
        RayK<K> *rayK_ptr[MAX_RAYS / K];
        for (size_t i=0;i<MAX_RAYS / K;i++) rayK_ptr[i] = &rayK[i];
        AOStoSOA(rayK,input_rays,numTotalRays);
        const size_t numPackets = (numTotalRays+K-1)/K;
        if (unlikely(numPackets == 1))
        {
          /* packet tracer as fallback */
          bvh->scene->intersect(rayK[0].tnear <= rayK[0].tfar,rayK[0],context);
        }
        else
        {
          /* stream tracer as fast path */
          BVHNStreamIntersector<N, K, types, robust, PrimitiveIntersector>::occluded_coherent_soa(bvh,  (RayK<K>**)rayK_ptr, numTotalRays, context);     
        }
        /* SOA to AOS conversion */
        SOAtoAOS<K,true>(input_rays,rayK,numTotalRays);                    
        return;
      }
#endif

      for (size_t r=0;r<numTotalRays;r+=MAX_RAYS_PER_OCTANT)
      {
        Ray** rays = input_rays + r;
        const size_t numOctantRays = (r + MAX_RAYS_PER_OCTANT >= numTotalRays) ? numTotalRays-r : MAX_RAYS_PER_OCTANT;
        size_t m_active = numOctantRays ==  8*sizeof(size_t) ? (size_t)-1 : (((size_t)1 << numOctantRays))-1;
        if (unlikely(m_active == 0)) continue;

        /* do per ray precalculations */
        for (size_t i=0; i<numOctantRays; i++) {
          new (&ray_ctx[i]) RContext(rays[i]);
          new (&pre[i]) Precalculations(*rays[i],bvh);
        }

        stack[0].ptr  = BVH::invalidNode;
        stack[0].mask = (size_t)-1;
        stack[1].ptr  = bvh->root;
        stack[1].mask = m_active;

        StackItemMask* stackPtr      = stack + 2;

        const NearFarPreCompute pc(ray_ctx[0].rdir);

        while (1) pop:
        {
          /*! pop next node */
          STAT3(shadow.trav_stack_pop,1,1,1);                          
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
          assert(stackPtr->mask);
          size_t m_trav_active = stackPtr->mask & m_active;
          if (unlikely(m_trav_active == 0 && cur != BVH::invalidNode)) continue;

          const vfloat<K> inf(pos_inf);

          while (1)
          {
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
            if (unlikely(none(vmask))) goto pop;            

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

            const RContext *__restrict__ const cur_ray_ctx = ray_ctx;
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

            if (unlikely(none(vmask))) goto pop;

            BVHNNodeTraverserKHit<types,N,K>::traverseAnyHit(cur,m_trav_active,vmask,(unsigned int*)&maskK,stackPtr); 
#endif

          }

          /* current ray stream is done? */
          if (unlikely(cur == BVH::invalidNode))
            break;

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(shadow.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          size_t bits = m_trav_active & m_active;          

          assert(bits);
          m_active &= ~PrimitiveIntersector::occluded(pre,bits,rays,context,0,prim,num,bvh->scene,NULL,lazy_node);
          if (unlikely(m_active == 0)) break;

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
                                    TriangleMIntersector1MoellerTrumbore<4 COMMA 8 COMMA true >,
                                    TriangleMIntersectorKMoellerTrumbore<4 COMMA 8 COMMA 8 COMMA true > > Triangle4IntersectorStreamMoellerTrumbore;
    typedef ArrayIntersectorKStream<8,
                                    TriangleMIntersector1MoellerTrumbore<4 COMMA 8 COMMA false >,
                                    TriangleMIntersectorKMoellerTrumbore<4 COMMA 8 COMMA 8 COMMA false > > Triangle4IntersectorStreamMoellerTrumboreNoFilter;
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
