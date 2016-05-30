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

#include "bvh_intersector1.h"
#include "bvh_intersector_node.h"
#include "bvh_traverser1.h"

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

namespace embree
{ 
  namespace isa
  {
    int getISA() { 
      return VerifyMultiTargetLinking::getISA(); 
    }
  
    template<int N, int types, bool robust, typename PrimitiveIntersector1>
    void BVHNIntersector1<N,types,robust,PrimitiveIntersector1>::intersect(const BVH* __restrict__ bvh, Ray& __restrict__ ray, const RTCIntersectContext* context)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray,bvh);

      /*! stack state */
      StackItemT<NodeRef> stack[stackSize];           //!< stack of nodes 
      StackItemT<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemT<NodeRef>* stackEnd = stack+stackSize;
      stack[0].ptr  = bvh->root;
      stack[0].dist = neg_inf;

      /* filter out invalid rays */
#if defined(EMBREE_IGNORE_INVALID_RAYS)
      if (!ray.valid()) return;
#endif

      /* verify correct input */
      assert(ray.valid());
      assert(ray.tnear >= 0.0f);
      assert(!(types & BVH_MB) || (ray.time >= 0.0f && ray.time <= 1.0f));

      /*! load the ray into SIMD registers */
      size_t leafType = 0;
      const unsigned int* geomID_to_instID = nullptr;
      TravRay<N,Nx> vray(ray.org,ray.dir);
      vfloat<Nx> ray_near = max(ray.tnear,0.0f);
      vfloat<Nx> ray_far  = max(ray.tfar ,0.0f);

      /*! initialize the node traverser */
      BVHNNodeTraverser1<N,Nx,types> nodeTraverser(vray);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);

        /*! if popped node is too far, pop next one */
        if (unlikely(*(float*)&stackPtr->dist > ray.tfar))
          continue;

        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf node */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);

          /* intersect node */
          size_t mask = 0;
          vfloat<Nx> tNear;
          bool nodeIntersected = BVHNNodeIntersector1<N,Nx,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time,tNear,mask);
          if (unlikely(!nodeIntersected)) break;

          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;

          /* select next child and push other children */
          nodeTraverser.traverseClosestHit(cur,mask,tNear,stackPtr,stackEnd);
        }

        /* ray transformation support */
        if (unlikely(nodeTraverser.traverseTransform(cur,ray,vray,leafType,geomID_to_instID,stackPtr,stackEnd)))
          goto pop;
        
        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        size_t lazy_node = 0;
        PrimitiveIntersector1::intersect(pre,ray,context,leafType,prim,num,bvh->scene,geomID_to_instID,lazy_node);
        ray_far = ray.tfar;

        /*! push lazy node onto stack */
        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = neg_inf;
          stackPtr++;
        }

        // perform stack compaction
        /*StackItemT<NodeRef>* left=stack;
        for (StackItemT<NodeRef>* right=stack; right<stackPtr; right++) 
        {
          if (*(float*)&right->dist >= ray.tfar) continue;
          *left = *right; left++;
        }
        stackPtr = left;*/
      }
      AVX_ZERO_UPPER();
    }
    
    template<int N, int types, bool robust, typename PrimitiveIntersector1>
    void BVHNIntersector1<N,types,robust,PrimitiveIntersector1>::occluded(const BVH* __restrict__ bvh, Ray& __restrict__ ray, const RTCIntersectContext* context)
    {
      /*! early out for already occluded rays */
      if (unlikely(ray.geomID == 0))
        return;

      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray,bvh);

      /*! stack state */
      NodeRef stack[stackSize];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSize;
      stack[0] = bvh->root;
      
      /* filter out invalid rays */
#if defined(EMBREE_IGNORE_INVALID_RAYS)
      if (!ray.valid()) return;
#endif

      /* verify correct input */
      assert(ray.valid());
      assert(ray.tnear >= 0.0f);
      assert(!(types & BVH_MB) || (ray.time >= 0.0f && ray.time <= 1.0f));

      /*! load the ray into SIMD registers */
      size_t leafType = 0;
      const unsigned int* geomID_to_instID = nullptr;
      TravRay<N,Nx> vray(ray.org,ray.dir);
      vfloat<Nx> ray_near = max(ray.tnear,0.0f);
      vfloat<Nx> ray_far  = max(ray.tfar ,0.0f);

      /*! initialize the node traverser */
      BVHNNodeTraverser1<N,Nx,types> nodeTraverser(vray);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = (NodeRef) *stackPtr;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf node */
          if (unlikely(cur.isLeaf())) break;
          STAT3(shadow.trav_nodes,1,1,1);

          /* intersect node */
          size_t mask = 0;
          vfloat<Nx> tNear;
          bool nodeIntersected = BVHNNodeIntersector1<N,Nx,types,robust>::intersect(cur,vray,ray_near,ray_far,ray.time,tNear,mask);
          if (unlikely(!nodeIntersected)) break;

          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;

          /* select next child and push other children */
          nodeTraverser.traverseAnyHit(cur,mask,tNear,stackPtr,stackEnd);
        }
        
        /* ray transformation support */
        if (unlikely(nodeTraverser.traverseTransform(cur,ray,vray,leafType,geomID_to_instID,stackPtr,stackEnd)))
          goto pop;

        /*! this is a leaf node */
        assert(cur != BVH::emptyNode);
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        size_t lazy_node = 0;
        if (PrimitiveIntersector1::occluded(pre,ray,context,leafType,prim,num,bvh->scene,geomID_to_instID,lazy_node)) {
          ray.geomID = 0;
          break;
        }
        
        /*! push lazy node onto stack */
        if (unlikely(lazy_node)) {
          *stackPtr = (NodeRef)lazy_node;
          stackPtr++;
        }
      }
      AVX_ZERO_UPPER();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector1 Definitions
    ////////////////////////////////////////////////////////////////////////////////

    IF_ENABLED_LINES(DEFINE_INTERSECTOR1(BVH4Line4iIntersector1,BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<LineMiIntersector1<4 COMMA 4 COMMA true> > >));
    IF_ENABLED_LINES(DEFINE_INTERSECTOR1(BVH4Line4iMBIntersector1,BVHNIntersector1<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<LineMiMBIntersector1<4 COMMA 4 COMMA true> > >));

    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH4Bezier1vIntersector1,BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH4Bezier1iIntersector1,BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH4Bezier1vIntersector1_OBB,BVHNIntersector1<4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH4Bezier1iIntersector1_OBB,BVHNIntersector1<4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH4Bezier1iMBIntersector1_OBB,BVHNIntersector1<4 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >));
  
#if 1
    typedef Select2Intersector1<
      TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA true>,
      TriangleMvMBIntersector1MoellerTrumbore<4 COMMA 4 COMMA true> > Intersector1_Triangle4Moeller_Triangle4vMBMoeller;
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4XfmTriangle4Intersector1Moeller,BVHNIntersector1<4 COMMA BVH_TN_AN1_AN2 COMMA false COMMA Intersector1_Triangle4Moeller_Triangle4vMBMoeller>));
#else
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4XfmTriangle4Intersector1Moeller,BVHNIntersector1<4 COMMA BVH_TN_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA true> > >));
#endif

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4Triangle4Intersector1Moeller,BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4Triangle4vIntersector1Pluecker,BVHNIntersector1<4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<TriangleMvIntersector1Pluecker<4 COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4Triangle4iIntersector1Pluecker,BVHNIntersector1<4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<Triangle4iIntersector1Pluecker<4 COMMA 4 COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH4Triangle4vMBIntersector1Moeller,BVHNIntersector1<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<4 COMMA 4 COMMA true> > >));

    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR1(BVH4Subdivpatch1CachedIntersector1,BVHNIntersector1<4 COMMA BVH_AN1 COMMA true COMMA SubdivPatch1CachedIntersector1>));
    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR1(BVH4GridAOSIntersector1,BVHNIntersector1<4 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersector1>));

    IF_ENABLED_USER(DEFINE_INTERSECTOR1(BVH4VirtualIntersector1,BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<ObjectIntersector1> >));
    IF_ENABLED_USER(DEFINE_INTERSECTOR1(BVH4VirtualMBIntersector1,BVHNIntersector1<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<ObjectIntersector1> >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH4Quad4vIntersector1Moeller,BVHNIntersector1<4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH4Quad4iIntersector1Pluecker,BVHNIntersector1<4 COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH4Quad4iMBIntersector1Pluecker,BVHNIntersector1<4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<QuadMiMBIntersector1Pluecker<4 COMMA true> > >));

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(QBVH4Triangle4iIntersector1Pluecker,BVHNIntersector1<4 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<Triangle4iIntersector1Pluecker<4 COMMA 4 COMMA true> > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(QBVH4Quad4iIntersector1Pluecker,BVHNIntersector1<4 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));
    
    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector1 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4Intersector1Moeller,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >));
    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(BVH8Triangle4vMBIntersector1Moeller,BVHNIntersector1<8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<SIMD_MODE(4) COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4vIntersector1Moeller,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<QuadMvIntersector1MoellerTrumbore<4 COMMA true> > >));

    IF_ENABLED_TRIS(DEFINE_INTERSECTOR1(QBVH8Triangle4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<Triangle4iIntersector1Pluecker<4 COMMA 4 COMMA true> > >));

    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(QBVH8Quad4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_QN1 COMMA false COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));

    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH8Bezier1vIntersector1_OBB,BVHNIntersector1<8 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH8Bezier1iIntersector1_OBB,BVHNIntersector1<8 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >));
    IF_ENABLED_HAIR(DEFINE_INTERSECTOR1(BVH8Bezier1iMBIntersector1_OBB,BVHNIntersector1<8 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >));
    IF_ENABLED_LINES(DEFINE_INTERSECTOR1(BVH8Line4iIntersector1,BVHNIntersector1<8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersector1<LineMiIntersector1<4 COMMA 4 COMMA true> > >));
    IF_ENABLED_LINES(DEFINE_INTERSECTOR1(BVH8Line4iMBIntersector1,BVHNIntersector1<8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<LineMiMBIntersector1<4 COMMA 4 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4iIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true COMMA ArrayIntersector1<QuadMiIntersector1Pluecker<4 COMMA true> > >));
    IF_ENABLED_QUADS(DEFINE_INTERSECTOR1(BVH8Quad4iMBIntersector1Pluecker,BVHNIntersector1<8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersector1<QuadMiMBIntersector1Pluecker<4 COMMA true> > >));

    IF_ENABLED_SUBDIV(DEFINE_INTERSECTOR1(BVH8GridAOSIntersector1,BVHNIntersector1<8 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersector1>));

#endif
  }
}
