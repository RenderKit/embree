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

#include "bvh_intersector_hybrid2.h"
#include "bvh_intersector_single.h"
#include "bvh_intersector_node.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglei.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglepairsv.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/trianglepairs_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle4i_intersector_pluecker.h"
#include "../geometry/trianglepairs_intersector_moeller.h"
#include "../geometry/quad_intersector_moeller.h"
#include "../geometry/quadi_intersector_moeller.h"
#include "../geometry/quadi_intersector_pluecker.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/subdivpatch1cached.h"
#include "../geometry/object_intersector.h"

#define SWITCH_DURING_DOWN_TRAVERSAL 1
#define FORCE_SINGLE_MODE 0

#define DBG(x) 
//PRINT(x)

namespace embree
{
  namespace isa
  {
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid2<N,K,types,robust,PrimitiveIntersectorK,single>::intersect(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* verify correct input */
      vbool<K> valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      Vec3vfK &ray_org = ray.org;
      Vec3vfK &ray_dir = ray.dir;
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      const vfloat16 ray_tnear = select(valid0,ray.tnear,vfloat<K>(pos_inf));
            vfloat16 ray_tfar  = select(valid0,ray.tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
      nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
      nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

      vbool<K> m_active = ray_tnear <= ray_tfar;
      while(m_active)
      {
        DBG(m_active);
        size_t first = __bsf(m_active);
        vbool<K> m_samesign = \
          (nearXYZ.x[first] == nearXYZ.x) &
          (nearXYZ.y[first] == nearXYZ.y) &
          (nearXYZ.z[first] == nearXYZ.z);
        assert(m_samesign);

        m_active &=~m_samesign;
        DBG(m_samesign);

#if defined(__AVX512F__) && 1
        StackItemMaskT<NodeRef>  stack[stackSizeSingle];  //!< stack of nodes 
        StackItemMaskT<NodeRef>* stackPtr = stack + 1;    //!< current stack pointer
        StackItemMaskT<NodeRef>* stackEnd = stack + stackSizeSingle;
        stack[0].ptr  = bvh->root;
        stack[0].mask = m_samesign;
        stack[0].dist = neg_inf;

        const size_t nearX = nearXYZ.x[first];
        const size_t nearY = nearXYZ.y[first];
        const size_t nearZ = nearXYZ.z[first];
#if 1
        const vint<K> id( step );
        const vint<K> id2 = align_shift_right<8>(id,id);
 
        const vint<K> permX = select(vfloat<K>(rdir.x[first]) >= 0.0f,id,id2);
        const vint<K> permY = select(vfloat<K>(rdir.y[first]) >= 0.0f,id,id2);
        const vint<K> permZ = select(vfloat<K>(rdir.z[first]) >= 0.0f,id,id2);

#else

        const size_t farX  = nearX ^ sizeof(vfloat<8>);
        const size_t farY  = nearY ^ sizeof(vfloat<8>);
        const size_t farZ  = nearZ ^ sizeof(vfloat<8>);

#endif


        while (1) pop:
        {
          /*! pop next node */
          if (unlikely(stackPtr == stack)) break;
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
          vbool16 m_trav_active = stackPtr->mask;
          DBG("pop");
          DBG(cur);
          DBG(m_trav_active);
	  DBG( lt(m_trav_active,vfloat16(*(float*)&stackPtr->dist),ray.tfar));

          /*! if popped node is too far, pop next one */
          if (unlikely(none(lt(m_trav_active,vfloat16(*(float*)&stackPtr->dist),ray.tfar)))) continue;

          while (likely(!cur.isLeaf()))
          {
            DBG("TRAVERSAL");
            DBG(cur);
            const Node* __restrict__ const node = cur.node();
            //STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

#if 0
            const vfloat16 bminX = vfloat16(*(vfloat8*)((const char*)&node->lower_x+nearX));
            const vfloat16 bminY = vfloat16(*(vfloat8*)((const char*)&node->lower_x+nearY));
            const vfloat16 bminZ = vfloat16(*(vfloat8*)((const char*)&node->lower_x+nearZ));
            const vfloat16 bmaxX = vfloat16(*(vfloat8*)((const char*)&node->lower_x+farX));
            const vfloat16 bmaxY = vfloat16(*(vfloat8*)((const char*)&node->lower_x+farY));
            const vfloat16 bmaxZ = vfloat16(*(vfloat8*)((const char*)&node->lower_x+farZ));
#else

            const vfloat16 bminmaxX = permute(vfloat16::load((float*)&node->lower_x),permX);
            const vfloat16 bminmaxY = permute(vfloat16::load((float*)&node->lower_y),permY);
            const vfloat16 bminmaxZ = permute(vfloat16::load((float*)&node->lower_z),permZ);


#endif
            vfloat16 dist(inf);
            size_t bits = movemask(m_trav_active);
            vint16 mask16( zero );
            do
            {                    
              STAT3(normal.trav_nodes,1,1,1);                          
              size_t i = __bscf(bits);
#if 0            
              const vfloat16 tNearX = msub(bminX, rdir.x[i], org_rdir.x[i]); // optimize loading of 'i
              const vfloat16 tNearY = msub(bminY, rdir.y[i], org_rdir.y[i]);
              const vfloat16 tNearZ = msub(bminZ, rdir.z[i], org_rdir.z[i]);
              const vfloat16 tFarX  = msub(bmaxX, rdir.x[i], org_rdir.x[i]);
              const vfloat16 tFarY  = msub(bmaxY, rdir.y[i], org_rdir.y[i]);
              const vfloat16 tFarZ  = msub(bmaxZ, rdir.z[i], org_rdir.z[i]);      
              const vint16 bitmask( (unsigned int)1 << i );
              const vfloat16 tNear  = max(tNearX,tNearY,tNearZ,vfloat16(ray_tnear[i]));
              const vfloat16 tFar   = min(tFarX ,tFarY ,tFarZ ,vfloat16(ray_tfar[i]));
              const vbool16 vmask   = le(tNear,tFar);
#else
              const vfloat16 tNearFarX = msub(bminmaxX, rdir.x[i], org_rdir.x[i]);
              const vfloat16 tNearFarY = msub(bminmaxY, rdir.y[i], org_rdir.y[i]);
              const vfloat16 tNearFarZ = msub(bminmaxZ, rdir.z[i], org_rdir.z[i]);
              const vint16 bitmask( (unsigned int)1 << i );
              const vfloat16 tNear     = max(tNearFarX,tNearFarY,tNearFarZ,vfloat16(ray_tnear[i]));
              const vfloat16 tFar      = min(tNearFarX,tNearFarY,tNearFarZ,vfloat16(ray_tfar[i]));
              const vbool16 vmask      = le(tNear,align_shift_right<8>(tFar,tFar));              
#endif

              dist   = select(vmask,min(tNear,dist),dist);
              mask16 = select(vmask,mask16 | bitmask,mask16); // optimize
            } while(bits);
            
            const vbool16 vmask   = lt(vbool16(0xff),dist,inf);
            DBG(dist);
            DBG(mask16);
            DBG(vmask);
            if (unlikely(none(vmask))) goto pop;

            DBG("SORT");

            m_trav_active = BVHNNodeTraverserKHit<types>::traverseClosestHit(cur, vmask,dist,mask16,stackPtr,stackEnd);              
            DBG(m_trav_active);
          }          

          DBG("INTERSECTION");

          /*! this is a leaf node */
          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          STAT3(normal.trav_hit_boxes[__popcnt(m_trav_active)],1,1,1);                          

          size_t lazy_node = 0;
          size_t bits = movemask(m_trav_active);
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) 
          {                                              
            PrimitiveIntersectorK::intersect(pre, ray, i, prim, num, bvh->scene, lazy_node);
          }
          ray_tfar = min(ray_tfar,ray.tfar);

        } // traversal + intersection

        
#else
        size_t bits = movemask(m_samesign);
        for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
          NodeRef cur = bvh->root;      
          BVHNIntersectorKSingle<N,K,types,robust,PrimitiveIntersectorK>::intersect1(bvh, cur, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
        }
#endif
      }
      DBG(ray);
      //exit(0);
      AVX_ZERO_UPPER();
    }

    // ===================================================================================================================================================================
    // ===================================================================================================================================================================
    // ===================================================================================================================================================================


    
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid2<N,K,types,robust,PrimitiveIntersectorK,single>::occluded(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* verify correct input */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      vbool<K> terminated = !valid;
      Vec3vfK ray_org = ray.org, ray_dir = ray.dir;
      vfloat<K> ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      if (single)
      {
        nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
        nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
        nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));
      }

      vbool<K> m_active = ray_tnear <= ray_tfar;

      size_t bits = movemask(m_active);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
        NodeRef cur = bvh->root;      
        if (BVHNIntersectorKSingle<N,K,types,robust,PrimitiveIntersectorK>::occluded1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
          set(terminated, i);
        if (all(terminated)) break;
      }
      vint<K>::store(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }


    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector16 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH8Quad4vIntersector16HybridMoeller2, BVHNIntersectorKHybrid2<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH8Quad4vIntersector16HybridMoellerNoFilter2, BVHNIntersectorKHybrid2<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA false> > >);


    DEFINE_INTERSECTOR16(BVH8Triangle4Intersector16HybridMoeller2, BVHNIntersectorKHybrid2<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH8Triangle4Intersector16HybridMoellerNoFilter2, BVHNIntersectorKHybrid2<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA false> > >);

#endif
  }
}


/*
            const vfloat16 lowerX = vfloat16(*(vfloat8*)&node->lower_x);
            const vfloat16 upperX = vfloat16(*(vfloat8*)&node->upper_x);

            const vfloat16 lowerY = vfloat16(*(vfloat8*)&node->lower_y);
            const vfloat16 upperY = vfloat16(*(vfloat8*)&node->upper_y);

            const vfloat16 lowerZ = vfloat16(*(vfloat8*)&node->lower_z);
            const vfloat16 upperZ = vfloat16(*(vfloat8*)&node->upper_z);

            const vfloat16 t_lowerX = msub(lowerX, vray.rdir.x, vray.org_rdir.x);
            const vfloat16 t_upperX = msub(upperX, vray.rdir.x, vray.org_rdir.x);
            const vfloat16 t_lowerY = msub(lowerY, vray.rdir.y, vray.org_rdir.y);
            const vfloat16 t_upperY = msub(upperY, vray.rdir.y, vray.org_rdir.y);
            const vfloat16 t_lowerZ = msub(lowerZ, vray.rdir.z, vray.org_rdir.z);
            const vfloat16 t_upperZ = msub(upperZ, vray.rdir.z, vray.org_rdir.z);

            const vbool16 m_node = lowerX != vfloat<K>(pos_inf);

            const vfloat16 tNearX = min(t_lowerX,t_upperX);
            const vfloat16 tFarX  = max(t_lowerX,t_upperX);
            const vfloat16 tNearY = min(t_lowerY,t_upperY);
            const vfloat16 tFarY  = max(t_lowerY,t_upperY);
            const vfloat16 tNearZ = min(t_lowerZ,t_upperZ);
            const vfloat16 tFarZ  = max(t_lowerZ,t_upperZ);

            const vfloat16 tNearX = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearX)), vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tNearY = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearY)), vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tNearZ = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.nearZ)), vray.rdir.z, vray.org_rdir.z);
            const vfloat16 tFarX  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farX )), vray.rdir.x, vray.org_rdir.x);
            const vfloat16 tFarY  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farY )), vray.rdir.y, vray.org_rdir.y);
            const vfloat16 tFarZ  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+vray.farZ )), vray.rdir.z, vray.org_rdir.z);
            const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near);
            const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);

            const vbool16 vmask = tNear <= tFar;

	org_rdir = org*rdir;


	while (true)
	{
	  stackPtr--;
	  NodeRef cur = NodeRef(stackPtr->ptr);
	  
          assert(*(float*)&stackPtr->dist < ray.tfar[rayID0]);

          while (true)
          {
            if (unlikely(cur0.isLeaf())) break;
            {
              STAT3(normal.trav_nodes,1,1,1);
              const typename BVH8::Node* node = cur0.node();

              const vfloat16 nodeX = vfloat16::load((float*)((const char*)&node->lower_x));
              const vfloat16 nodeY = vfloat16::load((float*)((const char*)&node->lower_y));
              const vfloat16 nodeZ = vfloat16::load((float*)((const char*)&node->lower_z));
              const vbool16 m_node = nodeX != vfloat<K>(pos_inf);
              const vfloat16 tNearFarX = msub(nodeX, rdir0.x, org_rdir0.x);
              const vfloat16 tNearFarY = msub(nodeY, rdir0.y, org_rdir0.y);
              const vfloat16 tNearFarZ = msub(nodeZ, rdir0.z, org_rdir0.z);
              const vfloat16 tFarNearX = align_shift_right<8>(tNearFarX,tNearFarX);
              const vfloat16 tFarNearY = align_shift_right<8>(tNearFarY,tNearFarY);
              const vfloat16 tFarNearZ = align_shift_right<8>(tNearFarZ,tNearFarZ);
              const vfloat16 tNearX = min(tNearFarX,tFarNearX);
              const vfloat16 tFarX  = max(tNearFarX,tFarNearX);
              const vfloat16 tNearY = min(tNearFarY,tFarNearY);
              const vfloat16 tFarY  = max(tNearFarY,tFarNearY);
              const vfloat16 tNearZ = min(tNearFarZ,tFarNearZ);
              const vfloat16 tFarZ  = max(tNearFarZ,tFarNearZ);
              const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near);
              const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);
              const vbool16 vmask = le(m_node,tNear,tFar);
              size_t mask = movemask(vmask);
              stackPtr0--;
              cur0 = NodeRef(stackPtr->ptr);

              if (unlikely(none(vmask))) continue;
              stackPtr0++;

              vfloat8 tNear8((__m256)tNear);
              BVHNNodeTraverser1<N,types>::traverseClosestHit(cur0,node,mask,tNear8,stackPtr0,stackEnd0);
            }

          }

          if (unlikely(cur == BVH::invalidNode)) {
            break;
          }
        
          assert(cur != BVH::emptyNode);
	  STAT3(normal.trav_leaves, 1, 1, 1);
	  size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          const float old_tfar = ray.tfar[rayID0];

          PrimitiveIntersectorK::intersect(pre, ray, rayID0, prim, num, bvh->scene, lazy_node);

          ray_far = select(mask8,vfloat<K>(ray.tfar[rayID0] ),vfloat<K>(neg_inf));
          if (unlikely(ray.tfar[rayID0] < old_tfar))
          {
            StackItemT<NodeRef>* left = stack + 1;
            for (StackItemT<NodeRef>* right = stack+1; right<stackPtr; right++) 
            {
              if (*(float*)&right->dist >= ray.tfar[rayID0]) continue;
              *left = *right; 
              left++;
            }
            stackPtr = left;
          }
	  //ray_far = ray.tfar[rayID0];

          if (unlikely(lazy_node)) {
            stackPtr->ptr = lazy_node;
            stackPtr->dist = neg_inf;
            stackPtr++;
          }        
        }
 
 */


//#define DBG_PRINT(x) PRINT(x)
#define DBG_PRINT(x) 

#if 0 // defined(__AVX512F__)
    // EXPERIMENTAL CODE WILL BE INTEGRATED AND REMOVED SOON
    template<int types, typename PrimitiveIntersectorK>
    void BVHNIntersectorKHybrid<8,16,types,false,PrimitiveIntersectorK,true>::intersect(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* verify correct input */
      vbool<K> valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));
      
      /* load ray */
      Vec3vfK ray_org16 = ray.org;
      Vec3vfK ray_dir16 = ray.dir;
      vfloat<K> ray_tnear16 = ray.tnear, ray_tfar16  = ray.tfar;
      const Vec3vfK rdir16 = rcp_safe(ray_dir16);
      const Vec3vfK org16(ray_org16), org_rdir16 = org16 * rdir16;
      ray_tnear16 = select(valid0,ray_tnear16,vfloat<K>(pos_inf));
      ray_tfar16  = select(valid0,ray_tfar16 ,vfloat<K>(neg_inf));

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir16.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
      nearXYZ.y = select(rdir16.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
      nearXYZ.z = select(rdir16.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

      Precalculations pre(valid0,ray);


      StackItemT<NodeRef> stack0[stackSizeSingle];  //!< stack of nodes 
      StackItemT<NodeRef>* stackEnd0 = stack0 + stackSizeSingle;

      stack0[0].ptr  = BVH::invalidNode;
      stack0[0].dist = pos_inf;


      const vint16 identity( step );
      const vint16 identity_half =  align_shift_right<8>(identity,identity);
      

#if 0
        asm nop;

      const vbool16 m_lower8(0xff);

      size_t m_active = movemask(valid0);

      while(m_active)
      {
        size_t m_active_traversal = 1;
        size_t rayID0 = __bsf(m_active);
        m_active      = __btc(m_active,rayID0);       

        //BVHNIntersectorKSingle<8,16,types,false,PrimitiveIntersectorK>::intersect1(bvh, cur, rayID0, pre, ray, ray_org16, ray_dir16, rdir16, ray_tnear16, ray_tfar16, nearXYZ);

        Vec3vfK rdir0 = Vec3vfK(rdir16.x[rayID0],rdir16.y[rayID0],rdir16.z[rayID0]);
        Vec3vfK org_rdir0 = Vec3vfK(org_rdir16.x[rayID0],org_rdir16.y[rayID0],org_rdir16.z[rayID0]);
        vfloat<K> ray_near0(ray_tnear16[rayID0]);
        vfloat<K> ray_far0(ray_tfar16[rayID0]);
        const vfloat<K> pinf(pos_inf);

        const vint16 permX = select(rdir0.x >= 0.0f,identity,identity_half);
        const vint16 permY = select(rdir0.y >= 0.0f,identity,identity_half);
        const vint16 permZ = select(rdir0.z >= 0.0f,identity,identity_half);

#if 1
        const vfloat16 upperSign(asFloat(select(m_lower8,vint16(zero),vint16(0x80000000))));
        rdir0.x = rdir0.x ^ upperSign;
        rdir0.y = rdir0.y ^ upperSign;
        rdir0.z = rdir0.z ^ upperSign;
        org_rdir0.x = org_rdir0.x ^ upperSign; 
        org_rdir0.y = org_rdir0.y ^ upperSign;
        org_rdir0.z = org_rdir0.z ^ upperSign;
        const vfloat16 sign(asFloat(vint16(0x80000000)));
#else
        const size_t flip = sizeof(vfloat<N>);
        const size_t nearX = nearXYZ.x[rayID0];
        const size_t nearY = nearXYZ.y[rayID0];
        const size_t nearZ = nearXYZ.z[rayID0];
        const size_t farX  = nearX ^ flip;
        const size_t farY  = nearY ^ flip;
        const size_t farZ  = nearZ ^ flip;
#endif

        StackItemT<NodeRef>* stackPtr0 = stack0 + 1;        //!< current stack pointer 0


        NodeRef cur0 = bvh->root;

	while (true)
	{
          const vfloat<K> ray_far8 (select(m_lower8,ray_far0,vfloat<K>(neg_inf)));
          const vfloat<K> ray_near_far = select(m_lower8,ray_near0,-ray_far0);

          /* down traversal */
          while(true)
          {                        
            if (unlikely(cur0.isLeaf())) break;
            STAT3(normal.trav_nodes,1,1,1);
              /* intersect node */
            const typename BVH8::Node* node = cur0.node();

#if 1
            const vfloat16 nodeX = permute(vfloat16::load((float*)&node->lower_x),permX);
            const vfloat16 nodeY = permute(vfloat16::load((float*)&node->lower_y),permY);
            const vfloat16 nodeZ = permute(vfloat16::load((float*)&node->lower_z),permZ);
            const vfloat16 tNearFarX = msub(nodeX, rdir0.x, org_rdir0.x);
            const vfloat16 tNearFarY = msub(nodeY, rdir0.y, org_rdir0.y);
            const vfloat16 tNearFarZ = msub(nodeZ, rdir0.z, org_rdir0.z);

            stackPtr0--;
            assert(stackPtr0 >= stack0);
            cur0 = NodeRef(stackPtr0->ptr);

            const vfloat16 tNearFar = max(tNearFarX,tNearFarY,tNearFarZ,ray_near_far);
#if 0
            const vfloat16 tNear = tNearFar;
            const vfloat16 tFar  = align_shift_right<8>(tNearFar,tNearFar) ^ sign;
            const vbool16 vmask = le(m_lower8,tNear,tFar);
#else
            const vfloat16 tNear = tNearFar ^ sign;
            const vfloat16 tFar  = align_shift_right<8>(tNearFar,tNearFar);
            const vbool16 vmask = ge(m_lower8,tNear,tFar);

#endif

#endif

#if 0
            const vfloat16 tNearX = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+nearX)), rdir0.x, org_rdir0.x);
            const vfloat16 tNearY = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+nearY)), rdir0.y, org_rdir0.y);
            const vfloat16 tNearZ = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+nearZ)), rdir0.z, org_rdir0.z);
            const vfloat16 tFarX  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+farX )), rdir0.x, org_rdir0.x);
            const vfloat16 tFarY  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+farY )), rdir0.y, org_rdir0.y);
            const vfloat16 tFarZ  = msub(vfloat16(*(vfloat8*)((const char*)&node->lower_x+farZ )), rdir0.z, org_rdir0.z);

            stackPtr0--;
            assert(stackPtr0 >= stack0);
            cur0 = NodeRef(stackPtr0->ptr);

            const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near0);
            const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far8);

            const vbool16 vmask = tNear <= tFar;

#endif 

#if 0
            const vfloat16 nodeX = vfloat16::load((float*)((const char*)&node->lower_x));
            //const vbool16 m_node = ne(m_lower8,nodeX,pinf);
            const vbool16 m_node = ne(nodeX,pinf);

            const vfloat16 nodeY = vfloat16::load((float*)((const char*)&node->lower_y));
            const vfloat16 nodeZ = vfloat16::load((float*)((const char*)&node->lower_z));
            const vfloat16 tNearFarX = msub(nodeX, rdir0.x, org_rdir0.x);
            const vfloat16 tNearFarY = msub(nodeY, rdir0.y, org_rdir0.y);
            const vfloat16 tNearFarZ = msub(nodeZ, rdir0.z, org_rdir0.z);
            
            stackPtr0--;
            assert(stackPtr0 >= stack0);
            cur0 = NodeRef(stackPtr0->ptr);

            const vfloat16 tFarNearX = align_shift_right<8>(tNearFarX,tNearFarX);
            const vfloat16 tFarNearY = align_shift_right<8>(tNearFarY,tNearFarY);
            const vfloat16 tFarNearZ = align_shift_right<8>(tNearFarZ,tNearFarZ);
            const vfloat16 tNearX = min(tNearFarX,tFarNearX);
            const vfloat16 tFarX  = max(tNearFarX,tFarNearX);
            const vfloat16 tNearY = min(tNearFarY,tFarNearY);
            const vfloat16 tFarY  = max(tNearFarY,tFarNearY);
            const vfloat16 tNearZ = min(tNearFarZ,tFarNearZ);
            const vfloat16 tFarZ  = max(tNearFarZ,tFarNearZ);
            const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near0);
            const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far8);
            const vbool16 vmask = le(m_node,tNear,tFar);
            DBG_PRINT(vmask);
#endif
            size_t mask = movemask(vmask);

            /*! if no child is hit, pop next node */
            if (unlikely(any(vmask)))
            {
              stackPtr0++;
              /* select next child and push other children */
              //vfloat8 tNear8((__m256)tNear);
              traverseClosestHit(cur0,node,mask,tNear,stackPtr0,stackEnd0);              
            }
          }
          if (unlikely(cur0 == BVH::invalidNode)) break;


          assert(cur != BVH::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur0.leaf(num);

          size_t lazy_node = 0;
          PrimitiveIntersectorK::intersect(pre, ray, rayID0, prim, num, bvh->scene, lazy_node);

          // perform stack0 compaction
          if (unlikely(any(ray.tfar[rayID0] < ray_far0)))
          {
            DBG_PRINT("COMPACT STACK rayID0");
            
            StackItemT<NodeRef>* left = stack0 + 1;
            for (StackItemT<NodeRef>* right = stack0+1; right<stackPtr0; right++) 
            {
              DBG_PRINT(right->dist);
              DBG_PRINT(right->ptr);
              if (*(float*)&right->dist >= ray.tfar[rayID0]) continue;
              *left = *right; 
              left++;
                DBG_PRINT(left->dist);
                DBG_PRINT(left->ptr);
            }
            stackPtr0 = left;
            DBG_PRINT(stackPtr0-stack0);
          }

          ray_far0 = ray.tfar[rayID0];
          stackPtr0--;
          assert(stackPtr0 >= stack0);
          cur0 = stackPtr0->ptr;          
        }
      }
#else

      StackItemT<NodeRef> stack1[stackSizeSingle];  //!< stack of nodes 
      StackItemT<NodeRef>* stackEnd1 = stack1 + stackSizeSingle;
      stack1[0].ptr  = BVH::invalidNode;
      stack1[0].dist = pos_inf;

      /*! load the ray into SIMD registers */
      const vbool16 m_lower8(0xff);

      size_t m_active = movemask(valid0);
      while(m_active)
      {
        size_t m_active_traversal = 1;
        size_t rayID0 = __bsf(m_active);
        m_active      = __btc(m_active,rayID0);       
        size_t rayID1 = (size_t)-1;
        if (likely(m_active))
        {
          m_active_traversal |= 2;
          rayID1 = __bsf(m_active);
          m_active      = __btc(m_active,rayID1);                 
        }

        DBG_PRINT(rayID0);
        DBG_PRINT(rayID1);
        

        StackItemT<NodeRef>* stackPtr0 = stack0 + 1;        //!< current stack pointer 0
        StackItemT<NodeRef>* stackPtr1 = stack1 + 1;        //!< current stack pointer 1

        Vec3vfK rdir0 = Vec3vfK(rdir16.x[rayID0],rdir16.y[rayID0],rdir16.z[rayID0]);
        Vec3vfK rdir1 = Vec3vfK(rdir16.x[rayID1],rdir16.y[rayID1],rdir16.z[rayID1]);
        Vec3vfK org_rdir0 = Vec3vfK(org_rdir16.x[rayID0],org_rdir16.y[rayID0],org_rdir16.z[rayID0]);
        Vec3vfK org_rdir1 = Vec3vfK(org_rdir16.x[rayID1],org_rdir16.y[rayID1],org_rdir16.z[rayID1]);
        //vfloat<K> ray_far0(ray_tfar16[rayID0]);
        //vfloat<K> ray_far1(ray_tfar16[rayID1]);

#if 0

        const vint16 permX0 = select(rdir0.x >= 0.0f,identity,identity_half);
        const vint16 permY0 = select(rdir0.y >= 0.0f,identity,identity_half);
        const vint16 permZ0 = select(rdir0.z >= 0.0f,identity,identity_half);

        const vint16 permX1 = select(rdir1.x >= 0.0f,identity,identity_half);
        const vint16 permY1 = select(rdir1.y >= 0.0f,identity,identity_half);
        const vint16 permZ1 = select(rdir1.z >= 0.0f,identity,identity_half);

        const vfloat16 upperSign(asFloat(select(m_lower8,vint16(zero),vint16(0x80000000))));

        rdir0.x = rdir0.x ^ upperSign;
        rdir0.y = rdir0.y ^ upperSign;
        rdir0.z = rdir0.z ^ upperSign;
        org_rdir0.x = org_rdir0.x ^ upperSign; 
        org_rdir0.y = org_rdir0.y ^ upperSign;
        org_rdir0.z = org_rdir0.z ^ upperSign;

        rdir1.x = rdir1.x ^ upperSign;
        rdir1.y = rdir1.y ^ upperSign;
        rdir1.z = rdir1.z ^ upperSign;
        org_rdir1.x = org_rdir1.x ^ upperSign; 
        org_rdir1.y = org_rdir1.y ^ upperSign;
        org_rdir1.z = org_rdir1.z ^ upperSign;

        const vfloat16 sign(asFloat(vint16(0x80000000)));

#endif
        asm nop;

        NodeRef cur0 = bvh->root;
        NodeRef cur1 = bvh->root;
        const vfloat<K> pinf(pos_inf);

	while (true)
	{
          NodeRef cur;

          const vfloat<K> ray_far0 = ray.tfar[rayID0];
          const vfloat<K> ray_far1 = ray.tfar[rayID1];
          const vfloat<K> ray_near0(ray_tnear16[rayID0]);
          const vfloat<K> ray_near1(ray_tnear16[rayID1]);
          // const vfloat<K> ray_near_far0 = select(m_lower8,ray_near0,ray_far0^sign);
          // const vfloat<K> ray_near_far1 = select(m_lower8,ray_near1,ray_far1^sign);

          /* down traversal */
          while(true)
          {                        
            DBG_PRINT("TRAVERSAL");

            /*! stop if we found a leaf node for ray0 */
            if ( likely(m_active_traversal & 1) )
            {
              DBG_PRINT("TRAVERSAL0");
              DBG_PRINT(cur0);

              cur = cur0;
              if (unlikely(cur0.isLeaf())) break;
              STAT3(normal.trav_nodes,1,1,1);
              /* intersect node */
              const typename BVH8::Node* node = cur0.node();

              const vfloat16 nodeX = vfloat16::load((float*)((const char*)&node->lower_x));
              const vbool16 m_node = ne(m_lower8,nodeX,pinf);
              const vfloat16 nodeY = vfloat16::load((float*)((const char*)&node->lower_y));
              const vfloat16 nodeZ = vfloat16::load((float*)((const char*)&node->lower_z));
              const vfloat16 tNearFarX = msub(nodeX, rdir0.x, org_rdir0.x);
              const vfloat16 tNearFarY = msub(nodeY, rdir0.y, org_rdir0.y);
              const vfloat16 tNearFarZ = msub(nodeZ, rdir0.z, org_rdir0.z);

              stackPtr0--;
              assert(stackPtr1 >= stack1);
              cur0 = NodeRef(stackPtr0->ptr);

              const vfloat16 tFarNearX = align_shift_right<8>(tNearFarX,tNearFarX);
              const vfloat16 tFarNearY = align_shift_right<8>(tNearFarY,tNearFarY);
              const vfloat16 tFarNearZ = align_shift_right<8>(tNearFarZ,tNearFarZ);
              const vfloat16 tNearX = min(tNearFarX,tFarNearX);
              const vfloat16 tFarX  = max(tNearFarX,tFarNearX);
              const vfloat16 tNearY = min(tNearFarY,tFarNearY);
              const vfloat16 tFarY  = max(tNearFarY,tFarNearY);
              const vfloat16 tNearZ = min(tNearFarZ,tFarNearZ);
              const vfloat16 tFarZ  = max(tNearFarZ,tFarNearZ);
              const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near0);
              const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far0);
              const vbool16 vmask = le(m_node,tNear,tFar);
              DBG_PRINT(vmask);

              size_t mask = movemask(vmask);
              /*! if no child is hit, pop next node */
              if (unlikely(any(vmask)))
              {
                stackPtr0++;
                /* select next child and push other children */
                //vfloat8 tNear8((__m256)tNear);
                traverseClosestHit(cur0,node,mask,tNear,stackPtr0,stackEnd0);              
              }
              DBG_PRINT(cur0);

              //cur0.prefetch();
            }

            /*! stop if we found a leaf node for ray1 */
            if ( likely(m_active_traversal & 2) )
            {
              DBG_PRINT("TRAVERSAL1");
              DBG_PRINT(cur1);

              cur = cur1;
              if (unlikely(cur1.isLeaf())) break;
              STAT3(normal.trav_nodes,1,1,1);
              /* intersect node */
              const typename BVH8::Node* node = cur1.node();

              const vfloat16 nodeX = vfloat16::load((float*)((const char*)&node->lower_x));
              const vbool16 m_node = ne(m_lower8,nodeX,pinf);
              const vfloat16 nodeY = vfloat16::load((float*)((const char*)&node->lower_y));
              const vfloat16 nodeZ = vfloat16::load((float*)((const char*)&node->lower_z));
              const vfloat16 tNearFarX = msub(nodeX, rdir1.x, org_rdir1.x);
              const vfloat16 tNearFarY = msub(nodeY, rdir1.y, org_rdir1.y);
              const vfloat16 tNearFarZ = msub(nodeZ, rdir1.z, org_rdir1.z);

              stackPtr1--;
              assert(stackPtr1 >= stack1);
              cur1 = NodeRef(stackPtr1->ptr);

              const vfloat16 tFarNearX = align_shift_right<8>(tNearFarX,tNearFarX);
              const vfloat16 tFarNearY = align_shift_right<8>(tNearFarY,tNearFarY);
              const vfloat16 tFarNearZ = align_shift_right<8>(tNearFarZ,tNearFarZ);
              const vfloat16 tNearX = min(tNearFarX,tFarNearX);
              const vfloat16 tFarX  = max(tNearFarX,tFarNearX);
              const vfloat16 tNearY = min(tNearFarY,tFarNearY);
              const vfloat16 tFarY  = max(tNearFarY,tFarNearY);
              const vfloat16 tNearZ = min(tNearFarZ,tFarNearZ);
              const vfloat16 tFarZ  = max(tNearFarZ,tFarNearZ);
              const vfloat16 tNear = max(tNearX,tNearY,tNearZ,ray_near1);
              const vfloat16 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far1);
              const vbool16 vmask = le(m_node,tNear,tFar);
              DBG_PRINT(vmask);
              size_t mask = movemask(vmask);

              /*! if no child is hit, pop next node */
              if (unlikely(any(vmask)))
              {
                stackPtr1++;
                /* select next child and push other children */
                //vfloat8 tNear8((__m256)tNear);
                traverseClosestHit(cur1,node,mask,tNear,stackPtr1,stackEnd1);              
              }
              DBG_PRINT(cur1);
              //cur1.prefetch();
            }           
          }

          /* leaf code */

	  /*! sentinal to indicate stack is empty */          
          if (unlikely(cur == BVH::invalidNode)) 
          {
            DBG_PRINT("TRAVERSAL FINISHED");
            DBG_PRINT(m_active_traversal);

            if (cur0 == BVH::invalidNode) 
            {
              if (m_active)
              {
                rayID0        = __bsf(m_active);
                m_active      = __btc(m_active,rayID0);       
                rdir0         = Vec3vfK(rdir16.x[rayID0],rdir16.y[rayID0],rdir16.z[rayID0]);
                org_rdir0     = Vec3vfK(org_rdir16.x[rayID0],org_rdir16.y[rayID0],org_rdir16.z[rayID0]);
                //ray_near0     = ray_tnear16[rayID0];
                cur0          = bvh->root;
                stackPtr0     = stack0 + 1;
              }                
              else
                m_active_traversal &= ~1;
            }
            if (cur1 == BVH::invalidNode) 
            {
              if (m_active)
              {
                rayID1        = __bsf(m_active);
                m_active      = __btc(m_active,rayID1);       
                rdir1         = Vec3vfK(rdir16.x[rayID1],rdir16.y[rayID1],rdir16.z[rayID1]);
                org_rdir1     = Vec3vfK(org_rdir16.x[rayID1],org_rdir16.y[rayID1],org_rdir16.z[rayID1]);
                //ray_near1     = ray_tnear16[rayID1];
                cur1          = bvh->root;
                stackPtr1     = stack1 + 1;               
              } 
              else
                m_active_traversal &= ~2;
            }

            DBG_PRINT(m_active_traversal);

            if (unlikely(m_active_traversal == 0))
              break;
          }
          else
          {
            DBG_PRINT("LEAF");
            DBG_PRINT(cur);

            assert(cur != BVH::emptyNode);
            STAT3(normal.trav_leaves, 1, 1, 1);
            size_t num; Primitive* prim = (Primitive*)cur.leaf(num);
            size_t rayID = cur == cur0 ? rayID0 : rayID1;

            DBG_PRINT(rayID);

            DBG_PRINT(ray.tfar[rayID0]);
            DBG_PRINT(ray.tfar[rayID1]);

            DBG_PRINT("INTERSECT");
            size_t lazy_node = 0;
            PrimitiveIntersectorK::intersect(pre, ray, rayID, prim, num, bvh->scene, lazy_node);

            DBG_PRINT(ray.tfar[rayID0]);
            DBG_PRINT(ray.tfar[rayID1]);

            // perform stack0 compaction
            if (unlikely(any(ray.tfar[rayID0] < ray_far0)))
            {
              DBG_PRINT("COMPACT STACK rayID0");

              StackItemT<NodeRef>* left = stack0 + 1;
              for (StackItemT<NodeRef>* right = stack0+1; right<stackPtr0; right++) 
              {
                DBG_PRINT(right->dist);
                DBG_PRINT(right->ptr);
                if (*(float*)&right->dist >= ray.tfar[rayID0]) continue;
                *left = *right; 
                left++;
                DBG_PRINT(left->dist);
                DBG_PRINT(left->ptr);
              }
              stackPtr0 = left;
              DBG_PRINT(stackPtr0-stack0);
            }

            // perform stack1 compaction
            if (unlikely(any(ray.tfar[rayID1] < ray_far1)))
            {
              DBG_PRINT("COMPACT STACK rayID1");

              StackItemT<NodeRef>* left = stack1 + 1;
              for (StackItemT<NodeRef>* right = stack1+1; right<stackPtr1; right++) 
              {
                if (*(float*)&right->dist >= ray.tfar[rayID1]) continue;
                *left = *right; 
                left++;
              }
              stackPtr1 = left;
              DBG_PRINT(stackPtr1-stack1);
            }

            {
              DBG_PRINT("STACK 0");
              size_t i;
              i = 0;
              for (StackItemT<NodeRef>* right = stack0; right<stackPtr0; right++,i++) 
              {
                DBG_PRINT(i);
                DBG_PRINT(right->dist);
                DBG_PRINT(right->ptr);
              }
              i = 0;
              DBG_PRINT("STACK 1");
              for (StackItemT<NodeRef>* right = stack1; right<stackPtr1; right++,i++) 
              {
                DBG_PRINT(i);
                DBG_PRINT(right->dist);
                DBG_PRINT(right->ptr);
              }

            }
            
            DBG_PRINT(ray_far0);
            DBG_PRINT(ray_far1);

            if (cur == cur0)
            {
              stackPtr0--;
              assert(stackPtr0 >= stack0);
              cur0 = stackPtr0->ptr;
              DBG_PRINT("STACK_POP cur0");
              DBG_PRINT(cur0);
            }
            else
            {
              stackPtr1--;
              assert(stackPtr1 >= stack1);
              cur1 = stackPtr1->ptr;
              DBG_PRINT("STACK_POP cur1");
              DBG_PRINT(cur1);
            }

          } /* leaf */
          
        } /* traversal */
        // exit(0);
      } /* ray packet */
#endif

      AVX_ZERO_UPPER();
    }


    template<int types, typename PrimitiveIntersectorK>
    void BVHNIntersectorKHybrid<8,16,types,false,PrimitiveIntersectorK,true>::occluded(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* verify correct input */
      vbool<K> valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));
      
      /* load ray */
      Vec3vfK ray_org16 = ray.org;
      Vec3vfK ray_dir16 = ray.dir;
      vfloat<K> ray_tnear16 = ray.tnear, ray_tfar16  = ray.tfar;
      const Vec3vfK rdir16 = rcp_safe(ray_dir16);
      const Vec3vfK org16(ray_org16), org_rdir16 = org16 * rdir16;
      ray_tnear16 = select(valid0,ray_tnear16,vfloat<K>(pos_inf));
      ray_tfar16  = select(valid0,ray_tfar16 ,vfloat<K>(neg_inf));

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir16.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
      nearXYZ.y = select(rdir16.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
      nearXYZ.z = select(rdir16.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

      Precalculations pre(valid0,ray);

      vbool<K> terminated = !valid0;

      size_t m_active = movemask(valid0);
      while(m_active)
      {
        size_t m_active_traversal = 1;
        size_t rayID0 = __bsf(m_active);
        m_active      = __btc(m_active,rayID0);       

        NodeRef cur = bvh->root;
        if (BVHNIntersectorKSingle<8,16,types,false,PrimitiveIntersectorK>::occluded1(bvh, cur, rayID0, pre, ray, ray_org16, ray_dir16, rdir16, ray_tnear16, ray_tfar16, nearXYZ))
          set(terminated, rayID0);

      }
      vint<K>::store(valid0 & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
#endif

    // ===================================================================================================================================================================
    // ===================================================================================================================================================================
    // ===================================================================================================================================================================
