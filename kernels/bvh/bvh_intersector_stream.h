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

#pragma once

#include "bvh.h"
#include "../common/ray.h"
#include "../common/stack_item.h"
#include "bvh_traverser1.h"

#if defined(__X86_64__)
#define ENABLE_COHERENT_STREAM_PATH 1
#else
#define ENABLE_COHERENT_STREAM_PATH 0
#endif

namespace embree
{
  namespace isa 
  {

    /*! An item on the stack holds the node ID and distance of that node. */
    struct __aligned(8) StackItemMask
    {
      size_t mask;
      size_t ptr; 
    };

    template<int types, int N, int K>
      class BVHNNodeTraverserKHit
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;


    public:

      template<class T>
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t &m_trav_active,
                                                   const vbool<K> &vmask,
                                                   const vfloat<K>& tNear,
                                                   const T * const tMask,
                                                   StackItemMask*& stackPtr)
      {
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

        const unsigned int * const tNear_i = (unsigned int*)&tNear;

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
            stackPtr->ptr = c1; 
            stackPtr->mask = tMask[r1]; 
            stackPtr++; 
            cur = c0; 
            m_trav_active = tMask[r0]; 
            return; 
          }
          else { 
            stackPtr->ptr = c0; 
            stackPtr->mask = tMask[r0]; 
            stackPtr++; 
            cur = c1; 
            m_trav_active = tMask[r1]; 
            return; 
          }
        }
        /*! slow path for more than two hits */
        const size_t hits = __popcnt(movemask(vmask));
        const vint<K> dist_i = select(vmask,(asInt(tNear) & 0xfffffff8) | vint<K>( step ),0x7fffffff);
#if defined(__AVX512F__)
        const vint8 tmp = _mm512_castsi512_si256(dist_i);
        const vint<K> dist_i_sorted = sortNetwork(tmp);
#else
        const vint<K> dist_i_sorted = sortNetwork(dist_i);
#endif
        const vint<K> sorted_index = dist_i_sorted & 7;

        size_t i = hits-1;
        for (;;)
        {
          const unsigned int index = sorted_index[i];
          assert(index < 8);
          cur = node->child(index);
          m_trav_active = tMask[index];
          assert(m_trav_active);
          cur.prefetch(types);
          if (unlikely(i==0)) break;
          i--;
          assert(cur != BVH::emptyNode);
          stackPtr->ptr = cur; 
          stackPtr->mask = m_trav_active;
          stackPtr++;
        }
      }

      template<class T, class M>
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t &m_trav_active,
                                               const M &vmask,
                                               const T * const tMask,
                                               StackItemMask*& stackPtr)
      {
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
        stackPtr->ptr  = cur;
        stackPtr->mask = m_trav_active;
        stackPtr++;

        for (; ;)
        {
          r = __bscf(mask);
          cur = node->child(r);          
          cur.prefetch(types);
          m_trav_active = tMask[r];
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return;
          stackPtr->ptr  = cur;
          stackPtr->mask = m_trav_active;
          stackPtr++;
        }
      }
    };

    // ==================================================================================================
    // ==================================================================================================
    // ==================================================================================================

    struct __aligned(8) StackItemMaskCoherent
    {
      size_t mask;
      size_t parent;
      size_t child;
      unsigned int childID;
      unsigned int dist;
    };

    template<int types, int N, int K>
      class BVHNNodeTraverserKHitCoherent
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;


    public:

      template<class T>
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t &m_trav_active,
                                               const vbool<K> &vmask,
                                               const T * const tMask,
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
        stackPtr->childID = (unsigned int)r;
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
          stackPtr->childID = (unsigned int)r;
          stackPtr++;
        }
      }

      template<class T>
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t &m_trav_active,
                                                   const vbool<K> &vmask,
                                                   const vfloat<K>& tNear,
                                                   const T * const tMask,
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

        const unsigned int * const tNear_i = (unsigned int*)&tNear;

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
            stackPtr->childID = (unsigned int)r1;
            stackPtr->dist    = d1;
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
            stackPtr->childID = (unsigned int)r0;
            stackPtr->dist    = d0;
            stackPtr++; 
            cur = c1; 
            m_trav_active = tMask[r1]; 
            return; 
          }
        }
        /*! slow path for more than two hits */
        const size_t hits = __popcnt(movemask(vmask));
        const vint<K> dist_i = select(vmask,(asInt(tNear) & 0xfffffff8) | vint<K>( step ),0x7fffffff);
#if defined(__AVX512F__)
        const vint8 tmp = _mm512_castsi512_si256(dist_i);
        const vint<K> dist_i_sorted = sortNetwork(tmp);
#else
        const vint<K> dist_i_sorted = sortNetwork(dist_i);
#endif
        const vint<K> sorted_index = dist_i_sorted & 7;

        size_t i = hits-1;
        for (;;)
        {
          const unsigned int index = sorted_index[i];
          assert(index < 8);
          cur = node->child(index);
          m_trav_active = tMask[index];
          assert(m_trav_active);
          cur.prefetch(types);
          if (unlikely(i==0)) break;
          i--;
          assert(cur != BVH::emptyNode);
          assert(tNear[index] >= 0.0f);
          stackPtr->mask    = m_trav_active;
          stackPtr->parent  = parent;
          stackPtr->child   = cur;
          stackPtr->childID = index;
          stackPtr->dist = tNear_i[index];
          stackPtr++;
        }
      }

    };

    // ==================================================================================================
    // ==================================================================================================
    // ==================================================================================================


    template<int K, bool robust>
    class __aligned(32) RayContext 
    {
    public:
      Vec3fa rdir;      //     rdir.w = tnear;
      Vec3fa org_rdir;  // org_rdir.w = tfar; org_rdir = org in robust mode        

      __forceinline RayContext() {}

      __forceinline RayContext(Ray* ray)
      {
#if defined(__AVX512F__)
        vfloat<K> org(vfloat4(ray->org));
        vfloat<K> dir(vfloat4(ray->dir));
        vfloat<K> rdir       = select(0x7777,rcp_safe(dir),max(vfloat<K>(ray->tnear),vfloat<K>(zero)));
        vfloat<K> org_rdir   = robust ? select(0x7777,org,ray->tfar) : select(0x7777,org * rdir,ray->tfar);
        vfloat<K> res = select(0xf,rdir,org_rdir);
        vfloat8 r = extractf256bit(res);
        *(vfloat8*)this = r;          
#else
        Vec3fa& org = ray->org;
        Vec3fa& dir = ray->dir;
        rdir       = rcp_safe(dir);
        org_rdir   = robust ? org : org * rdir;
        rdir.w     = max(0.0f,ray->tnear);
        org_rdir.w = ray->tfar;
#endif
      }

      __forceinline void update(const Ray* ray) {
        org_rdir.w = ray->tfar;
      }

      __forceinline unsigned int tfar_ui() const {
        return *(unsigned int*)&org_rdir.w;
      }

      __forceinline float tfar() const {
        return org_rdir.w;
      }

    };



    /*! BVH ray stream intersector. */
    template<int N, int K, int types, bool robust, typename PrimitiveIntersector>
      class BVHNStreamIntersector
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector::Precalculations Precalculations;
      typedef typename PrimitiveIntersector::Primitive Primitive;
      typedef BVHN<N> BVH;
      typedef RayContext<K,robust> RContext;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::BaseNode BaseNode;
      typedef typename BVH::Node Node;
      typedef typename BVH::NodeMB NodeMB;
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef Vec3<vint<K>> Vec3viK;
      static const size_t stackSize = 
        1+(N-1)*BVH::maxDepth+   // standard depth
        1+(N-1)*BVH::maxDepth;   // transform feature


      struct __aligned(32) RayFiberContext {
        NodeRef node;
        size_t mask;
        StackItemMask* stackPtr;
        RayFiberContext *next;
#if !defined(__AVX512F__)
        size_t offset;
#endif
        
        __forceinline void init(NodeRef _node,
                                size_t  _mask,
                                StackItemMask* _stackPtr,
                                RayFiberContext *_next,
                                size_t _offset)
        {
          node        = _node;
          mask        = _mask;
          stackPtr    = _stackPtr;
          next        = _next;
#if !defined(__AVX512F__)
          offset      = _offset;
#endif
        }

        __forceinline RayFiberContext *swapContext(NodeRef &_node,
                                                   size_t  &_mask,
                                                   StackItemMask*&_stackPtr)
        {
          node = _node;
          mask = _mask;
          stackPtr = _stackPtr;          
          assert(next);
          _node     = next->node;
          _mask     = next->mask;
          _stackPtr = next->stackPtr;          
          return next;
        }

        __forceinline size_t getOffset()
        {
#if !defined(__AVX512F__)
          return offset;
#else
          return 0;
#endif

        }
                           
      };


      struct NearFarPreCompute
      {
#if defined(__AVX512F__)
        vint<K> permX,permY,permZ;
#endif
        size_t nearX,nearY,nearZ;
        size_t farX,farY,farZ;
        __forceinline NearFarPreCompute(const Vec3fa &dir)
        {
#if defined(__AVX512F__)
        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);
        permX = select(vfloat<K>(dir.x) >= 0.0f,id,id2);
        permY = select(vfloat<K>(dir.y) >= 0.0f,id,id2);
        permZ = select(vfloat<K>(dir.z) >= 0.0f,id,id2);
#endif
        nearX = (dir.x < 0.0f) ? 1*sizeof(vfloat<N>) : 0*sizeof(vfloat<N>);
        nearY = (dir.y < 0.0f) ? 3*sizeof(vfloat<N>) : 2*sizeof(vfloat<N>);
        nearZ = (dir.z < 0.0f) ? 5*sizeof(vfloat<N>) : 4*sizeof(vfloat<N>);
        farX  = nearX ^ sizeof(vfloat<N>);
        farY  = nearY ^ sizeof(vfloat<N>);
        farZ  = nearZ ^ sizeof(vfloat<N>);
        }
      };

      // =============================================================================================
      // =============================================================================================
      // =============================================================================================

      struct Packet {
        Vec3vfK rdir;
        Vec3vfK org_rdir;
        vfloat<K> min_dist;
        vfloat<K> max_dist;
      };

      struct Frusta {
        Vec3fa min_rdir; 
        Vec3fa max_rdir; 
        Vec3fa min_org_rdir; 
        Vec3fa max_org_rdir; 
        float min_dist;
        float max_dist;
      };

      __forceinline static size_t initPacketsAndFrusta(RayK<K> **input_packets, const size_t numOctantRays, Packet *const packet, Frusta &frusta)
      {
        const size_t numPackets = (numOctantRays+K-1)/K;

        Vec3vfK   tmp_min_rdir(pos_inf); 
        Vec3vfK   tmp_max_rdir(neg_inf);
        Vec3vfK   tmp_min_org(pos_inf); 
        Vec3vfK   tmp_max_org(neg_inf);
        vfloat<K> tmp_min_dist(pos_inf);
        vfloat<K> tmp_max_dist(neg_inf);

        size_t m_active = 0;
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
          tmp_min_org  = min(tmp_min_org ,select(m_valid,org ,Vec3vfK(pos_inf)));
          tmp_max_org  = max(tmp_max_org ,select(m_valid,org ,Vec3vfK(neg_inf)));
        }

        m_active &= (numOctantRays == 64) ? (size_t)-1 : (((size_t)1 << numOctantRays)-1);

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

        const Vec3fa frusta_min_rdir = select(ge_mask(reduced_min_rdir,Vec3fa(zero)),reduced_min_rdir,reduced_max_rdir);
        const Vec3fa frusta_max_rdir = select(ge_mask(reduced_min_rdir,Vec3fa(zero)),reduced_max_rdir,reduced_min_rdir);

        const Vec3fa frusta_min_org_rdir = frusta_min_rdir*select(ge_mask(reduced_min_rdir,Vec3fa(zero)),reduced_max_origin,reduced_min_origin);
        const Vec3fa frusta_max_org_rdir = frusta_max_rdir*select(ge_mask(reduced_min_rdir,Vec3fa(zero)),reduced_min_origin,reduced_max_origin);

        frusta.min_rdir     = frusta_min_rdir;
        frusta.max_rdir     = frusta_max_rdir;
        frusta.min_org_rdir = frusta_min_org_rdir;
        frusta.max_org_rdir = frusta_max_org_rdir;
        frusta.min_dist     = frusta_min_dist;
        frusta.max_dist     = frusta_max_dist;

        return m_active;
      }

      __forceinline static size_t intersectNodePacket(const Packet *const packet,
                                                      const vfloat<K> &minX,
                                                      const vfloat<K> &minY,
                                                      const vfloat<K> &minZ,
                                                      const vfloat<K> &maxX,
                                                      const vfloat<K> &maxY,
                                                      const vfloat<K> &maxZ,
                                                      const size_t m_active)
      {
        assert(m_active);
        const size_t startPacketID = __bsf(m_active) / K;
        const size_t endPacketID   = __bsr(m_active) / K;
        size_t m_trav_active = 0;
        for (size_t i=startPacketID;i<=endPacketID;i++)
        {
          //STAT3(normal.trav_nodes,1,1,1);                          
          const Packet &p = packet[i]; 
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
        return m_trav_active;
      }

      __forceinline static size_t traverseCoherentStream(const size_t m_trav_active,
                                                         Packet *const packet,
                                                         const Node* __restrict__ const node,
                                                         const NearFarPreCompute &pc,
                                                         const Frusta &frusta,
                                                         size_t *const maskK,
                                                         vfloat<K> &dist)
      {
        /* interval-based culling test */
        const vfloat<K> bminX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearX));
        const vfloat<K> bminY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearY));
        const vfloat<K> bminZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.nearZ));
        const vfloat<K> bmaxX = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farX));
        const vfloat<K> bmaxY = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farY));
        const vfloat<K> bmaxZ = vfloat<K>(*(vfloat<N>*)((const char*)&node->lower_x+pc.farZ));

        const vfloat<K> fminX = msub(bminX, vfloat<K>(frusta.min_rdir.x), vfloat<K>(frusta.min_org_rdir.x));
        const vfloat<K> fminY = msub(bminY, vfloat<K>(frusta.min_rdir.y), vfloat<K>(frusta.min_org_rdir.y));
        const vfloat<K> fminZ = msub(bminZ, vfloat<K>(frusta.min_rdir.z), vfloat<K>(frusta.min_org_rdir.z));
        const vfloat<K> fmaxX = msub(bmaxX, vfloat<K>(frusta.max_rdir.x), vfloat<K>(frusta.max_org_rdir.x));
        const vfloat<K> fmaxY = msub(bmaxY, vfloat<K>(frusta.max_rdir.y), vfloat<K>(frusta.max_org_rdir.y));
        const vfloat<K> fmaxZ = msub(bmaxZ, vfloat<K>(frusta.max_rdir.z), vfloat<K>(frusta.max_org_rdir.z));
        const vfloat<K> fmin  = maxi(maxi(fminX,fminY),maxi(fminZ,frusta.min_dist)); 
        const vfloat<K> fmax  = mini(mini(fmaxX,fmaxY),mini(fmaxZ,frusta.max_dist)); 
        const vbool<K> vmask_node_hit = fmin <= fmax;  
        //STAT3(normal.trav_nodes,1,1,1);                          

        size_t m_node_hit = movemask(vmask_node_hit) & (((size_t)1 << N)-1);
        // ==================
        const size_t first_index    = __bsf(m_trav_active);
        const size_t first_packetID = first_index / K;
        const size_t first_rayID    = first_index % K;

        Packet &p = packet[first_packetID]; 
        //STAT3(normal.trav_nodes,1,1,1);                          

        const vfloat<K> rminX = msub(bminX, vfloat<K>(p.rdir.x[first_rayID]), vfloat<K>(p.org_rdir.x[first_rayID]));
        const vfloat<K> rminY = msub(bminY, vfloat<K>(p.rdir.y[first_rayID]), vfloat<K>(p.org_rdir.y[first_rayID]));
        const vfloat<K> rminZ = msub(bminZ, vfloat<K>(p.rdir.z[first_rayID]), vfloat<K>(p.org_rdir.z[first_rayID]));
        const vfloat<K> rmaxX = msub(bmaxX, vfloat<K>(p.rdir.x[first_rayID]), vfloat<K>(p.org_rdir.x[first_rayID]));
        const vfloat<K> rmaxY = msub(bmaxY, vfloat<K>(p.rdir.y[first_rayID]), vfloat<K>(p.org_rdir.y[first_rayID]));
        const vfloat<K> rmaxZ = msub(bmaxZ, vfloat<K>(p.rdir.z[first_rayID]), vfloat<K>(p.org_rdir.z[first_rayID]));
        const vfloat<K> rmin  = maxi(maxi(rminX,rminY),maxi(rminZ,p.min_dist[first_rayID])); 
        const vfloat<K> rmax  = mini(mini(rmaxX,rmaxY),mini(rmaxZ,p.max_dist[first_rayID])); 

        const vbool<K> vmask_first_hit = rmin <= rmax;  

        size_t m_first_hit = movemask(vmask_first_hit) & (((size_t)1 << N)-1);

        // ==================
          
        dist = select(vmask_first_hit,rmin,fmin);            
        //dist = fmin;            

        size_t m_node = m_node_hit ^ (m_first_hit /*  & m_leaf */);
        while(unlikely(m_node)) 
        {
          const size_t b   = __bscf(m_node); 
          const vfloat<K> minX = vfloat<K>(bminX[b]);
          const vfloat<K> minY = vfloat<K>(bminY[b]);
          const vfloat<K> minZ = vfloat<K>(bminZ[b]);
          const vfloat<K> maxX = vfloat<K>(bmaxX[b]);
          const vfloat<K> maxY = vfloat<K>(bmaxY[b]);
          const vfloat<K> maxZ = vfloat<K>(bmaxZ[b]);
          const size_t m_current = m_trav_active & intersectNodePacket(packet,minX,minY,minZ,maxX,maxY,maxZ,m_trav_active);
          m_node_hit ^= m_current ? (size_t)0 : ((size_t)1 << b);
          maskK[b] = m_current;
        }
        return m_node_hit;
      }
      

      // =============================================================================================
      // =============================================================================================
      // =============================================================================================


      static const size_t stackSizeChunk  = N*BVH::maxDepth+1;
      static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;

      static void intersect_coherent_soa(BVH* bvh, RayK<K> **input_rays, size_t numValidStreams, const RTCIntersectContext* context);

      static void occluded_coherent_soa(BVH* bvh, RayK<K> **input_rays, size_t numValidStreams, const RTCIntersectContext* context);
      
    public:
      static void intersect(BVH* bvh, Ray **ray, size_t numRays, const RTCIntersectContext* context);
      static void occluded (BVH* bvh, Ray **ray, size_t numRays, const RTCIntersectContext* context);
    };


  }
}
