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

#pragma once

#include "bvh.h"
#include "../../common/ray.h"
#include "../../common/stack_item.h"

namespace embree
{
  namespace isa 
  {

    /*! An item on the stack holds the node ID and distance of that node. */
    template<typename T>
      struct __aligned(16) StackItemMaskT
    {
      /*! assert that the xchg function works */
      static_assert(sizeof(T) <= 8, "sizeof(T) <= 8 failed");

      /*! use SSE instructions to swap stack items */
      __forceinline static void xchg(StackItemMaskT& a, StackItemMaskT& b) 
      { 
        const vfloat4 sse_a = vfloat4::load((float*)&a); 
        const vfloat4 sse_b = vfloat4::load((float*)&b);
        vfloat4::store(&a,sse_b);
        vfloat4::store(&b,sse_a);
      }

      /*! Sort 2 stack items. */
      __forceinline friend void sort(StackItemMaskT& s1, StackItemMaskT& s2) {
        if (s2.dist < s1.dist) xchg(s2,s1);
      }
    
      /*! Sort 3 stack items. */
      __forceinline friend void sort(StackItemMaskT& s1, StackItemMaskT& s2, StackItemMaskT& s3)
      {
        if (s2.dist < s1.dist) xchg(s2,s1);
        if (s3.dist < s2.dist) xchg(s3,s2);
        if (s2.dist < s1.dist) xchg(s2,s1);
      }
    
      /*! Sort 4 stack items. */
      __forceinline friend void sort(StackItemMaskT& s1, StackItemMaskT& s2, StackItemMaskT& s3, StackItemMaskT& s4)
      {
        if (s2.dist < s1.dist) xchg(s2,s1);
        if (s4.dist < s3.dist) xchg(s4,s3);
        if (s3.dist < s1.dist) xchg(s3,s1);
        if (s4.dist < s2.dist) xchg(s4,s2);
        if (s3.dist < s2.dist) xchg(s3,s2);
      }

      /*! Sort N stack items. */
      __forceinline friend void sort(StackItemMaskT* begin, StackItemMaskT* end)
      {
        for (StackItemMaskT* i = begin+1; i != end; ++i)
        {
          const vfloat4 item = vfloat4::load((float*)i);
          const unsigned int dist = i->dist;
          StackItemMaskT* j = i;

          while ((j != begin) && ((j-1)->dist < dist))
          {
            vfloat4::store(j, vfloat4::load((float*)(j-1)));
            --j;
          }

          vfloat4::store(j, item);
        }
      }
    
    public:
      unsigned int dist;        
      unsigned int mask;
      T ptr; 
    };



    template<int K>
      class LoopTraversalPreCompute
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;
      typedef Vec3<vfloat<K>> Vec3vfK;
      typedef Vec3<vint<K>> Vec3viK;

    public:

      vint<K> permX;
      vint<K> permY;
      vint<K> permZ;

      size_t nearX,nearY,nearZ;
      size_t farX, farY, farZ;

      LoopTraversalPreCompute(const Vec3vfK &rdir, 
                              const Vec3vfK &org_rdir, 
                              const Vec3viK &nearXYZ, 
                              const size_t first) 
      {
#if defined(__AVX2__)
        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<K/2>(id,id);
 
        permX = select(vfloat<K>(rdir.x[first]) >= 0.0f,id,id2);
        permY = select(vfloat<K>(rdir.y[first]) >= 0.0f,id,id2);
        permZ = select(vfloat<K>(rdir.z[first]) >= 0.0f,id,id2);
        nearX = nearXYZ.x[first];
        nearY = nearXYZ.y[first];
        nearZ = nearXYZ.z[first];
        farX  = nearX ^ sizeof(vfloat<8>);
        farY  = nearY ^ sizeof(vfloat<8>);
        farZ  = nearZ ^ sizeof(vfloat<8>);
#endif
      }
    };

#if defined(__AVX__)
    template<int types, int K>
      class BVHNNodeTraverserKHit
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:


      // FIXME: optimize sequence
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   unsigned int &m_trav_active,
                                                   size_t mask,
                                                   const vfloat<K>& tNear,
                                                   const vint<K>& tMask,
                                                   StackItemMaskT<NodeRef>*& stackPtr,
                                                   StackItemMaskT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
#if 0
        vint<K> _tMask = tMask;
        const vbool<K> vmask = vbool<K>((unsigned int)mask);
        vint<K> cmask = vint<K>::compact(vmask,_tMask);
        m_trav_active = toScalar(cmask);
#endif
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);         
        cur.prefetch(types);
        m_trav_active = tMask[r]; //firstMask;
        assert(cur != BVH::emptyNode);
        if (unlikely(mask == 0)) return;

        unsigned int *active = (unsigned int*)&tMask;

        vint<K> dist = (asInt(tNear) & 0xfffffff8) | vint<K>( step );

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur; 
        const unsigned int d0 = dist[r];
        const int m0 = active[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); 
        c1.prefetch(types); 
        const unsigned int d1 = dist[r];
        const int m1 = active[r];

        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr->dist = d1; stackPtr++; cur = c0; m_trav_active = m0; return; }
          else         { stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr->dist = d0; stackPtr++; cur = c1; m_trav_active = m1; return; }
        }

        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr->dist = d0; stackPtr++;
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr->dist = d1; stackPtr++;

        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        NodeRef c = node->child(r); c.prefetch(types); 
        unsigned int d = dist[r]; 
        unsigned int m = active[r];
        stackPtr->ptr = c; stackPtr->mask = m; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; 
          unsigned int mm = stackPtr[-1].mask;
          stackPtr--;
          m_trav_active = mm;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); 
        c.prefetch(types); 
        m = active[r];
        d = dist[r]; 
        stackPtr->ptr = c; stackPtr->mask = m; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; 
          unsigned int mm = stackPtr[-1].mask;
          stackPtr--;
          m_trav_active = mm;
          return;
        }

        /*! fallback case if more than 4 children are hit */
        StackItemMaskT<NodeRef>* stackFirst = stackPtr-4;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          c = node->child(r); 
          c.prefetch(types); 
          d = dist[r]; 
          m = active[r];
          stackPtr->ptr  = c; 
          stackPtr->mask = m;
          stackPtr->dist = d; 
          stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        sort(stackFirst,stackPtr);
        cur = (NodeRef) stackPtr[-1].ptr; 
        unsigned int mm = stackPtr[-1].mask;
        stackPtr--;          
        m_trav_active = mm;
      }




      static __forceinline int traverseAnyHit(NodeRef& cur,
                                              size_t mask,
                                              const vfloat<K>& tNear,
                                              const vint<K>& tMask,
                                              StackItemMaskT<NodeRef>*& stackPtr,
                                              StackItemMaskT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);         
        cur.prefetch(types);
        
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return tMask[r];
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur; // node->child(r); 
        const unsigned int d0 = ((unsigned int*)&tNear)[r];
        const int m0 = tMask[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); 
        c1.prefetch(types); 
        const unsigned int d1 = ((unsigned int*)&tNear)[r];
        const int m1 = tMask[r];

        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr++; cur = c0; return m0; }
          else         { stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr++; cur = c1; return m1; }
        }

        /*! Here starts the slow path for 3+ hit children. */
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c0; stackPtr->mask = m0; stackPtr++;
        assert(stackPtr < stackEnd);
        stackPtr->ptr = c1; stackPtr->mask = m1; stackPtr++;

        StackItemMaskT<NodeRef>* stackFirst = stackPtr-2;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          NodeRef c = node->child(r); 
          c.prefetch(types); 
          int m = tMask[r];
          stackPtr->ptr  = c; 
          stackPtr->mask = m;
          stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        cur = (NodeRef) stackPtr[-1].ptr; 
        unsigned int mm = stackPtr[-1].mask;
        stackPtr--;
        return mm;
      }

    };
#endif

      /*! BVH hybrid packet intersector. Switches between packet and single ray traversal (optional). */
      template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single = true>
        class BVHNIntersectorKHybrid2
        {
          /* shortcuts for frequently used types */
          typedef typename PrimitiveIntersectorK::Precalculations Precalculations;
          typedef typename PrimitiveIntersectorK::Primitive Primitive;
          typedef BVHN<N> BVH;
          typedef typename BVH::NodeRef NodeRef;
          typedef typename BVH::BaseNode BaseNode;
          typedef typename BVH::Node Node;
          typedef typename BVH::NodeMB NodeMB;
          typedef Vec3<vfloat<K>> Vec3vfK;
          typedef Vec3<vint<K>> Vec3viK;

          static const size_t stackSizeChunk  = N*BVH::maxDepth+1;
          static const size_t stackSizeSingle = 1+(N-1)*BVH::maxDepth;




          static __forceinline vbool<K> loopIntersect( const NodeRef &cur,
                                                       const unsigned int &m_trav_active,
                                                       const LoopTraversalPreCompute<K> &prl,
                                                       const Vec3vfK& rdir,
                                                       const Vec3vfK& org_rdir, 
                                                       const vfloat<K> &ray_tnear,
                                                       const vfloat<K> &ray_tfar,
                                                       vfloat<K> &dist,
                                                       vint<K>   &maskK,
                                                       const vfloat<K> &inf,
                                                       const vint<K>& shift_one)
          {
              const Node* __restrict__ const node = cur.node();
              STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                                      
#if defined(__AVX512F__)
              const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),prl.permX);
              const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),prl.permY);
              const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),prl.permZ);

              dist = inf;
              maskK =  vint<K>( zero );
              size_t bits = m_trav_active;
              do
              {            
                STAT3(normal.trav_nodes,1,1,1);                          
                const size_t i = __bscf(bits);
                const vfloat<K> tNearFarX= msub(bminmaxX, rdir.x[i], org_rdir.x[i]);
                const vfloat<K> tNearFarY = msub(bminmaxY, rdir.y[i], org_rdir.y[i]);
                const vfloat<K> tNearFarZ = msub(bminmaxZ, rdir.z[i], org_rdir.z[i]);
                const vint<K> bitmask     = shift_one[i]; 
                //const vint<K> bitmask  = one << vint<K>(i);
                const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray_tnear[i]));
                const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat<K>(ray_tfar[i]));
                const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));              
                dist   = select(vmask,min(tNear,dist),dist);
                maskK = mask_or(vmask,maskK,maskK,bitmask); 
              } while(bits);
              const vbool<K> vmask   = lt(vbool<K>(0xff),dist,inf);
              return vmask;
#else
              const vfloat<K> bminX = vfloat<K>(*(vfloat8*)((const char*)&node->lower_x+prl.nearX));
              const vfloat<K> bminY = vfloat<K>(*(vfloat8*)((const char*)&node->lower_x+prl.nearY));
              const vfloat<K> bminZ = vfloat<K>(*(vfloat8*)((const char*)&node->lower_x+prl.nearZ));
              const vfloat<K> bmaxX = vfloat<K>(*(vfloat8*)((const char*)&node->lower_x+prl.farX));
              const vfloat<K> bmaxY = vfloat<K>(*(vfloat8*)((const char*)&node->lower_x+prl.farY));
              const vfloat<K> bmaxZ = vfloat<K>(*(vfloat8*)((const char*)&node->lower_x+prl.farZ));

              dist = inf;
              maskK =  vint<K>( zero );
              size_t bits = m_trav_active;
              do
              {            
                STAT3(normal.trav_nodes,1,1,1);                          
                const size_t i = __bscf(bits);
                const vfloat<K> tNearX = msub(bminX, rdir.x[i], org_rdir.x[i]); // optimize loading of 'i
                const vfloat<K> tNearY = msub(bminY, rdir.y[i], org_rdir.y[i]);
                const vfloat<K> tNearZ = msub(bminZ, rdir.z[i], org_rdir.z[i]);
                const vfloat<K> tFarX  = msub(bmaxX, rdir.x[i], org_rdir.x[i]);
                const vfloat<K> tFarY  = msub(bmaxY, rdir.y[i], org_rdir.y[i]);
                const vfloat<K> tFarZ  = msub(bmaxZ, rdir.z[i], org_rdir.z[i]);
                const vint<K> bitmask  = vint<K>((int)1 << i);
                const vfloat<K> tNear  = max(tNearX,tNearY,tNearZ,vfloat<K>(ray_tnear[i]));
                const vfloat<K> tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat<K>(ray_tfar[i]));
                const vbool<K> vmask   = tNear <= tFar;
                dist   = select(vmask,min(tNear,dist),dist);
                maskK = select(vmask,maskK | bitmask,maskK); 
              } while(bits);              
              return dist < inf;
#endif
          }
          
        public:
          static void intersect(vint<K>* valid, BVH* bvh, RayK<K>& ray);
          static void occluded (vint<K>* valid, BVH* bvh, RayK<K>& ray);
        };

    }


#if 0
          if (likely(__popcnt(m_trav_active) == 1))
          {            
            const size_t rayID = __bsf(m_trav_active);
            const vfloat<K> rdir_x = rdir.x[rayID];
            const vfloat<K> rdir_y = rdir.y[rayID];
            const vfloat<K> rdir_z = rdir.z[rayID];
            const vfloat<K> org_rdir_x = org_rdir.x[rayID];
            const vfloat<K> org_rdir_y = org_rdir.y[rayID];
            const vfloat<K> org_rdir_z = org_rdir.z[rayID];
            const vfloat<K> tnear      = ray_tnear[rayID];
            const vfloat<K> tfar       = ray_tfar[rayID];
            const vint<K> mask16       = one << vint<K>(rayID); 
            while (likely(!cur.isLeaf()))
            {
              STAT3(normal.trav_nodes,1,1,1);                          
              const Node* __restrict__ const node = cur.node();
              STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                         
              const vfloat<K> bminmaxX = permute(vfloat<K>::load((float*)&node->lower_x),permX);
              const vfloat<K> bminmaxY = permute(vfloat<K>::load((float*)&node->lower_y),permY);
              const vfloat<K> bminmaxZ = permute(vfloat<K>::load((float*)&node->lower_z),permZ);
              const vfloat<K> tNearFarX = msub(bminmaxX, rdir_x, org_rdir_x);
              const vfloat<K> tNearFarY = msub(bminmaxY, rdir_y, org_rdir_y);
              const vfloat<K> tNearFarZ = msub(bminmaxZ, rdir_z, org_rdir_z);
              const vfloat<K> tNear     = max(tNearFarX,tNearFarY,tNearFarZ,tnear);
              const vfloat<K> tFar      = min(tNearFarX,tNearFarY,tNearFarZ,tfar);
              const vbool<K> vmask      = le(tNear,align_shift_right<8>(tFar,tFar));
              if (unlikely(none(vmask))) goto pop;

              BVHNNodeTraverserKHit<types,K>::traverseClosestHit(cur,m_trav_active,vmask,tNear,mask16,stackPtr,stackEnd);
            }          
          }
          else
#endif

  }
