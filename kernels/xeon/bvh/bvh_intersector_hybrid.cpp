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

#include "bvh_intersector_hybrid.h"
#include "bvh_intersector_single.h"
#include "bvh_intersector_node.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglei.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle4i_intersector_pluecker.h"
#include "../geometry/quadv_intersector_moeller.h"
#include "../geometry/quadi_intersector_moeller.h"
#include "../geometry/quadi_intersector_pluecker.h"

#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/subdivpatch1cached.h"
#include "../geometry/object_intersector.h"

#define SWITCH_DURING_DOWN_TRAVERSAL 1
#define FORCE_SINGLE_MODE 0

namespace embree
{
  namespace isa
  {
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::intersect(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* filter out invalid rays */
      vbool<K> valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif

      /* verify correct input */
      assert(all(valid0,ray.valid()));
      assert(all(valid0,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));
      
      /* load ray */
      Vec3vfK ray_org = ray.org;
      Vec3vfK ray_dir = ray.dir;
      vfloat<K> ray_tnear = max(ray.tnear,0.0f);
      vfloat<K> ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      if (single)
      {
        nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
        nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
        nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));
      }

      /* allocate stack and push root node */
      vfloat<K> stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      vfloat<K>* __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* cull node if behind closest hit point */
        vfloat<K> curDist = *sptr_near;
        const vbool<K> active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;
        
        /* switch to single ray traversal */
#if (!defined(__WIN32__) || defined(__X86_64__)) && defined(__SSE4_2__)
#if FORCE_SINGLE_MODE == 0
        if (single)
#endif
        {
          size_t bits = movemask(active);
#if FORCE_SINGLE_MODE == 0
          if (unlikely(__popcnt(bits) <= switchThreshold)) 
#endif
          {
            for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
              BVHNIntersectorKSingle<N,K,types,robust,PrimitiveIntersectorK>::intersect1(bvh, cur, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
            }
            ray_tfar = min(ray_tfar,ray.tfar);
            continue;
          }
        }
#endif
        while (likely(!cur.isLeaf()))
        {
          /* process nodes */
          const vbool<K> valid_node = ray_tfar > curDist;
          STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
          const NodeRef nodeRef = cur;
          const BaseNode* __restrict__ const node = nodeRef.baseNode(types);

          /* set cur to invalid */
          cur = BVH::emptyNode;
          curDist = pos_inf;

          for (unsigned i=0; i<N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH::emptyNode)) break;
            vfloat<K> lnearP;
            vbool<K> lhit;
            BVHNNodeIntersectorK<N,K,types,robust>::intersect(nodeRef,i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP,lhit);

            /* if we hit the child we choose to continue with that child if it
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              assert(child != BVH::emptyNode);
              const vfloat<K> childDist = select(lhit,lnearP,inf);

              /* push cur node onto stack and continue with hit child */
              if (any(childDist < curDist))
              {
                if (likely(cur != BVH::emptyNode)) {
                  *sptr_node = cur; sptr_node++;
                  *sptr_near = curDist; sptr_near++;
                }
                curDist = childDist;
                cur = child;
              }

              /* push hit child onto stack */
              else {
                *sptr_node = child; sptr_node++;
                *sptr_near = childDist; sptr_near++;
              }
            }
          }
          if (unlikely(cur == BVH::emptyNode))
            goto pop;

#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          if (single)
          {
            // seems to be the best place for testing utilization
            if (unlikely(popcnt(ray_tfar > curDist) <= switchThreshold))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
          }
#endif
	}
        
        /* return if stack is empty */
        if (unlikely(cur == BVH::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
        assert(cur != BVH::emptyNode);
        const vbool<K> valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),K);
        size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
        PrimitiveIntersectorK::intersect(valid_leaf,pre,ray,prim,items,bvh->scene,lazy_node);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }

      AVX_ZERO_UPPER();
    }

    // ===================================================================================================================================================================
    // ===================================================================================================================================================================
    // ===================================================================================================================================================================


    
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK, bool single>
    void BVHNIntersectorKHybrid<N,K,types,robust,PrimitiveIntersectorK,single>::occluded(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* filter out invalid rays */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      vbool<K> terminated = !valid;
      Vec3vfK ray_org = ray.org, ray_dir = ray.dir;
      vfloat<K> ray_tnear = max(ray.tnear,0.0f);
      vfloat<K> ray_tfar  = max(ray.tfar ,0.0f);
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

      /* allocate stack and push root node */
      vfloat<K> stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      vfloat<K>* __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        /* cull node if behind closest hit point */
        vfloat<K> curDist = *sptr_near;
        const vbool<K> active = curDist < ray_tfar;
        if (unlikely(none(active))) 
          continue;
        
        /* switch to single ray traversal */
#if (!defined(__WIN32__) || defined(__X86_64__)) && defined(__SSE4_2__)
        if (single)
        {
          size_t bits = movemask(active);
          if (unlikely(__popcnt(bits) <= switchThreshold)) {
            for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
              if (BVHNIntersectorKSingle<N,K,types,robust,PrimitiveIntersectorK>::occluded1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
                set(terminated, i);
            }
            if (all(terminated)) break;
            ray_tfar = select(terminated,vfloat<K>(neg_inf),ray_tfar);
            continue;
          }
        }
#endif
                
        while (likely(!cur.isLeaf()))
        {
          /* process nodes */
          const vbool<K> valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),K);
          const NodeRef nodeRef = cur;
          const BaseNode* __restrict__ const node = nodeRef.baseNode(types);

          /* set cur to invalid */
          cur = BVH::emptyNode;
          curDist = pos_inf;

          for (unsigned i=0; i<N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH::emptyNode)) break;
            vfloat<K> lnearP;
            vbool<K> lhit;
            BVHNNodeIntersectorK<N,K,types,robust>::intersect(nodeRef,i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP,lhit);

            /* if we hit the child we choose to continue with that child if it
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              assert(child != BVH::emptyNode);
              const vfloat<K> childDist = select(lhit,lnearP,inf);

              /* push cur node onto stack and continue with hit child */
              if (any(childDist < curDist))
              {
                if (likely(cur != BVH::emptyNode)) {
                  *sptr_node = cur; sptr_node++;
                  *sptr_near = curDist; sptr_near++;
                }
                curDist = childDist;
                cur = child;
              }

              /* push hit child onto stack */
              else {
                *sptr_node = child; sptr_node++;
                *sptr_near = childDist; sptr_near++;
              }
            }
          }
          if (unlikely(cur == BVH::emptyNode))
            goto pop;

#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          if (single)
          {
            // seems to be the best place for testing utilization
            if (unlikely(popcnt(ray_tfar > curDist) <= switchThreshold))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
          }
#endif
	}
        
        /* return if stack is empty */
        if (unlikely(cur == BVH::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        
        /* intersect leaf */
        assert(cur != BVH::emptyNode);
        const vbool<K> valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),K);
        size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
        terminated |= PrimitiveIntersectorK::occluded(!terminated,pre,ray,prim,items,bvh->scene,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated,vfloat<K>(neg_inf),ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      vint<K>::store(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector4 Definitions
    ////////////////////////////////////////////////////////////////////////////////
  
    DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 4 COMMA false> > >);
#if defined(__AVX__)
    DEFINE_INTERSECTOR4(BVH4Triangle8Intersector4HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA  4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle8Intersector4HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 4 COMMA false> > >);
#endif
    DEFINE_INTERSECTOR4(BVH4Triangle4vIntersector4HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersectorK_1<4 COMMA TriangleMvIntersectorKPluecker<4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle4iIntersector4HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersectorK_1<4 COMMA Triangle4iIntersectorKPluecker<4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle4vMBIntersector4HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 4 COMMA true> > >);

    DEFINE_INTERSECTOR4(BVH4Quad4vIntersector4HybridMoeller        ,BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 4 COMMA true > > >);
    DEFINE_INTERSECTOR4(BVH4Quad4vIntersector4HybridMoellerNoFilter,BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 4 COMMA false> > >);
    DEFINE_INTERSECTOR4(BVH4Quad4iIntersector4HybridPluecker       ,BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiIntersectorKPluecker<4 COMMA 4 COMMA true > > >);
    DEFINE_INTERSECTOR4(BVH4Quad4iMBIntersector4HybridPluecker     ,BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 4 COMMA true > > >);
   
    DEFINE_INTERSECTOR4(BVH4Subdivpatch1CachedIntersector4, BVHNIntersectorKHybrid<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector4>);
    DEFINE_INTERSECTOR4(BVH4VirtualIntersector4Chunk, BVHNIntersectorKChunk<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK<4 COMMA ObjectIntersector4> >);
    DEFINE_INTERSECTOR4(BVH4VirtualMBIntersector4Chunk, BVHNIntersectorKChunk<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK<4 COMMA ObjectIntersector4> >);


    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector8 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA true> > >);

    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA false> > >);

    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4vIntersector8HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA ArrayIntersectorK_1<8 COMMA TriangleMvIntersectorKPluecker<4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4iIntersector8HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA ArrayIntersectorK_1<8 COMMA Triangle4iIntersectorKPluecker<4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4vMBIntersector8HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA true> > >);

    DEFINE_INTERSECTOR8(BVH4Quad4vIntersector8HybridMoeller        ,BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 8 COMMA true > > >);
    DEFINE_INTERSECTOR8(BVH4Quad4vIntersector8HybridMoellerNoFilter,BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH4Quad4iIntersector8HybridPluecker       ,BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiIntersectorKPluecker<4 COMMA 8 COMMA true > > >);
    DEFINE_INTERSECTOR8(BVH4Quad4iMBIntersector8HybridPluecker     ,BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 8 COMMA true> > >);
   
    DEFINE_INTERSECTOR8(BVH4Subdivpatch1CachedIntersector8, BVHNIntersectorKHybrid<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector8>);

    DEFINE_INTERSECTOR8(BVH4VirtualIntersector8Chunk, BVHNIntersectorKChunk<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK<8 COMMA ObjectIntersector8> >);
    DEFINE_INTERSECTOR8(BVH4VirtualMBIntersector8Chunk, BVHNIntersectorKChunk<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK<8 COMMA ObjectIntersector8> >);

#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector16 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle8Intersector16HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle8Intersector16HybridMoellerNoFilter, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4vIntersector16HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA ArrayIntersectorK_1<16 COMMA TriangleMvIntersectorKPluecker<4 COMMA 16 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4iIntersector16HybridPluecker, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA ArrayIntersectorK_1<16 COMMA Triangle4iIntersectorKPluecker<4 COMMA 16 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4vMBIntersector16HybridMoeller, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA true> > >);

    DEFINE_INTERSECTOR16(BVH4Quad4vIntersector16HybridMoeller        ,BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true > > >);
    DEFINE_INTERSECTOR16(BVH4Quad4vIntersector16HybridMoellerNoFilter,BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH4Quad4iIntersector16HybridPluecker       ,BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiIntersectorKPluecker<4 COMMA 16 COMMA true > > >);
    DEFINE_INTERSECTOR16(BVH4Quad4iMBIntersector16HybridPluecker     ,BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 16 COMMA true> > >);
   
    DEFINE_INTERSECTOR16(BVH4Subdivpatch1CachedIntersector16, BVHNIntersectorKHybrid<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector16>);

    DEFINE_INTERSECTOR16(BVH4VirtualIntersector16Chunk, BVHNIntersectorKChunk<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK<16 COMMA ObjectIntersector16> >);
    DEFINE_INTERSECTOR16(BVH4VirtualMBIntersector16Chunk, BVHNIntersectorKChunk<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK<16 COMMA ObjectIntersector16> >);

#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector4 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR4(BVH8Triangle4Intersector4HybridMoeller, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH8Triangle4Intersector4HybridMoellerNoFilter, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 4 COMMA false> > >);

    DEFINE_INTERSECTOR4(BVH8Triangle8Intersector4HybridMoeller, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH8Triangle8Intersector4HybridMoellerNoFilter, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 4 COMMA false> > >);

    DEFINE_INTERSECTOR4(BVH8Triangle4vMBIntersector4HybridMoeller, BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 4 COMMA true> > >);

    DEFINE_INTERSECTOR4(BVH8Quad4vIntersector4HybridMoeller         ,BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH8Quad4vIntersector4HybridMoellerNoFilter ,BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 4 COMMA false> > >);
    DEFINE_INTERSECTOR4(BVH8Quad4iIntersector4HybridPluecker        ,BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiIntersectorKPluecker<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH8Quad4iIntersector4HybridPlueckerNoFilter,BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiIntersectorKPluecker<4 COMMA 4 COMMA false> > >);
    DEFINE_INTERSECTOR4(BVH8Quad4iMBIntersector4HybridPluecker      ,BVHNIntersectorKHybrid<8 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 4 COMMA true> > >);
   
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector8 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8HybridMoeller,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8HybridMoellerNoFilter,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA false> > >);

    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8HybridMoeller,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA  8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8HybridMoellerNoFilter,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 8 COMMA false> > >);

    DEFINE_INTERSECTOR8(BVH8Triangle4vMBIntersector8HybridMoeller, BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 4 COMMA 8 COMMA true> > >);

    DEFINE_INTERSECTOR8(BVH8Quad4vIntersector8HybridMoeller         ,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Quad4vIntersector8HybridMoellerNoFilter ,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH8Quad4iIntersector8HybridPluecker        ,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiIntersectorKPluecker<4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Quad4iIntersector8HybridPlueckerNoFilter,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiIntersectorKPluecker<4 COMMA 8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH8Quad4iMBIntersector8HybridPluecker      ,BVHNIntersectorKHybrid<8 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 8 COMMA true> > >);

#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector16 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH8Triangle4Intersector16HybridMoeller,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH8Triangle4Intersector16HybridMoellerNoFilter,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA false> > >);

     DEFINE_INTERSECTOR16(BVH8Triangle8Intersector16HybridMoeller,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 16 COMMA true> > >);
     DEFINE_INTERSECTOR16(BVH8Triangle8Intersector16HybridMoellerNoFilter,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA 16 COMMA false> > >);

    DEFINE_INTERSECTOR16(BVH8Triangle4vMBIntersector16HybridMoeller, BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 16 COMMA 16 COMMA true> > >);

    DEFINE_INTERSECTOR16(BVH8Quad4vIntersector16HybridMoeller         ,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true > > >);
    DEFINE_INTERSECTOR16(BVH8Quad4vIntersector16HybridMoellerNoFilter ,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMvIntersectorKMoellerTrumbore<4 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH8Quad4iIntersector16HybridPluecker        ,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiIntersectorKPluecker<4 COMMA 16 COMMA true > > >);
    DEFINE_INTERSECTOR16(BVH8Quad4iIntersector16HybridPlueckerNoFilter,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiIntersectorKPluecker<4 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH8Quad4iMBIntersector16HybridPluecker      ,BVHNIntersectorKHybrid<8 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA QuadMiMBIntersectorKPluecker<4 COMMA 16 COMMA true > > >);

#endif
  }
}

