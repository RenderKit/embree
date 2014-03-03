// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "bvh4hair_intersector1.h"
#include "geometry/bezier1_intersector1.h"
#include "geometry/bezier1i_intersector1.h"

#if BVH4HAIR_WIDTH == 8
#define load4f load8f
#endif

namespace embree
{ 
#if BVH4HAIR_NAVIGATION
  extern BVH4Hair::NodeRef naviNode;
  extern ssize_t naviDepth;
#endif

  namespace isa
  {
    template<typename PrimitiveIntersector>
    __forceinline size_t BVH4HairIntersector1<PrimitiveIntersector>::intersectBox(const BVH4Hair::UnalignedNode* node, Ray& ray, 
                                                                                  const simd3f& ray_org, const simd3f& ray_dir, simdf& tNear, simdf& tFar)
    {
#if BVH4HAIR_SHARED_XFM
      const Vec3fa dir = xfmVector(node->space,ray.dir);
      const Vec3fa rdir = rcp_safe(dir);
      const Vec3fa org = xfmPoint(node->space,ray.org);
      const simd3f vorg  = simd3f(org);
      const simd3f vrdir = simd3f(rdir);
      const simd3f lower(node->lower_x,node->lower_y,node->lower_z);
      const simd3f upper(node->upper_x,node->upper_y,node->upper_z);
      const simd3f tLowerXYZ = (lower - vorg) * vrdir;
      const simd3f tUpperXYZ = (upper - vorg) * vrdir;
#else
      const simd3f dir = xfmVector(node->naabb,ray_dir);
      const simd3f rdir = rcp_safe(dir);
      const simd3f org = xfmPoint(node->naabb,ray_org);
      const simd3f tLowerXYZ = - org * rdir;     // (Vec3fa(zero) - org) * rdir;
      const simd3f tUpperXYZ = rdir + tLowerXYZ; // (Vec3fa(one ) - org) * rdir;
#endif

#if ((BVH4HAIR_WIDTH == 4) && defined(__SSE4_1__) || (BVH4HAIR_WIDTH == 8) && defined(__AVX2__))
      const simdf tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const simdf tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const simdf tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const simdf tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const simdf tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const simdf tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
      tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
      const simdb vmask = tNear <= tFar;
      return movemask(vmask);
#else
      const simdf tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const simdf tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const simdf tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const simdf tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const simdf tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const simdf tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      const simdb vmask = tNear <= tFar;
      return movemask(vmask);
#endif
    }

    template<typename PrimitiveIntersector>
    void BVH4HairIntersector1<PrimitiveIntersector>::intersect(const BVH4Hair* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      const Precalculations pre(ray);
      //const LinearSpace3fa pre(rcp(frame(ray.dir)));
      //const LinearSpace3fa pre(frame(ray.dir).transposed()); // FIXME: works only with normalized ray direction

      /*! stack state */
      StackItem stack[stackSize];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      StackItem* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      NAVI(stack[0].ref = naviNode);
      stack[0].tNear = ray.tnear;
      stack[0].tFar = ray.tfar;
      NAVI(stack[0].depth = 0);
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(simdf) : 1*sizeof(simdf);
      const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(simdf) : 3*sizeof(simdf);
      const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(simdf) : 5*sizeof(simdf);
      
      /*! load the ray into SIMD registers */
      const simd3f org(ray.org.x,ray.org.y,ray.org.z);
      const simd3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const simd3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const simd3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        /*for (size_t i=1; i<stackPtr-&stack[0]; i++)
          if (stack[i-1].tNear < stack[i+0].tNear)
            StackItem::swap2(stack[i-1],stack[i+0]);*/
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ref);
        simdf tNear = stackPtr->tNear;
        simdf tFar = min(stackPtr->tFar,ray.tfar);
        NAVI(size_t depth = stackPtr->depth);
        
        /*! if popped node is too far, pop next one */
#if BVH4HAIR_WIDTH == 4
        if (unlikely(_mm_cvtss_f32(tNear) > _mm_cvtss_f32(tFar)))
          continue;
#else
        if (unlikely(tNear[0] > tFar[0]))
          continue;
#endif

        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          size_t mask;
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! process nodes with aligned bounds */
          if (likely(cur.isAlignedNode()))
          {
            /*! single ray intersection with 4 boxes */
            const AlignedNode* node = cur.alignedNode();
            const size_t farX  = nearX ^ sizeof(simdf), farY  = nearY ^ sizeof(simdf), farZ  = nearZ ^ sizeof(simdf);
#if defined (__AVX2__)
            const simdf tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
            const simdf tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
            const simdf tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
            const simdf tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
            const simdf tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
            const simdf tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
            const simdf tNearX = (load4f((const char*)&node->lower_x+nearX) - org.x) * rdir.x;
            const simdf tNearY = (load4f((const char*)&node->lower_x+nearY) - org.y) * rdir.y;
            const simdf tNearZ = (load4f((const char*)&node->lower_x+nearZ) - org.z) * rdir.z;
            const simdf tFarX  = (load4f((const char*)&node->lower_x+farX ) - org.x) * rdir.x;
            const simdf tFarY  = (load4f((const char*)&node->lower_x+farY ) - org.y) * rdir.y;
            const simdf tFarZ  = (load4f((const char*)&node->lower_x+farZ ) - org.z) * rdir.z;
#endif
            
#if ((BVH4HAIR_WIDTH == 4) && defined(__SSE4_1__) || (BVH4HAIR_WIDTH == 8) && defined(__AVX2__))
            tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
            tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
            const simdb vmask = cast(tNear) > cast(tFar);
            mask = movemask(vmask)^((1<<BVH4Hair::N)-1);
#else
            tNear = max(tNearX,tNearY,tNearZ,tNear);
            tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
            const simdb vmask = tNear <= tFar;
            mask = movemask(vmask);
#endif
          }

          /*! process nodes with unaligned bounds */
          else {
            const UnalignedNode* node = cur.unalignedNode();
            mask = intersectBox(node,ray,org,dir,tNear,tFar);
          }

#if BVH4HAIR_NAVIGATION
          if (depth == naviDepth)
          {
            simdb valid = tNear <= tFar;
            if (any(valid)) {
              const int c = select_min(valid,tNear);
              const Node* node = cur.node();
              ray.tfar = tNear[c];
              ray.u = ray.v = 0.0f;
              ray.Ng = Vec3fa(zero);
              ray.geomID = 0;
              ray.primID = 132*(int) node->child(c);
            }
            goto pop;
          }
#endif

          /*! if no child is hit, pop next node */
          const Node* node = cur.node();
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          NodeRef c0 = node->child(r);
          c0.prefetch();

          if (likely(mask == 0)) {
            cur = c0;  tNear = tNear[r]; tFar = tFar[r]; NAVI(depth++);
            assert(cur != BVH4Hair::emptyNode);
            continue;
          }

          /*! two children are hit, push far child, and continue with closer child */
          const float n0 = tNear[r]; const float f0 = tFar[r]; 
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const float n1 = tNear[r]; const float f1 = tFar[r];
          assert(c0 != BVH4Hair::emptyNode);
          assert(c1 != BVH4Hair::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd); 
            if (n0 < n1) { 
              stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; NAVI(stackPtr->depth = depth+1); stackPtr++; 
              cur = c0; tNear = n0; tFar = f0; NAVI(depth++);
              continue; 
            }
            else { 
              stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; NAVI(stackPtr->depth = depth+1); stackPtr++; 
              cur = c1; tNear = n1; tFar = f1; NAVI(depth++);
              continue; 
            }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; NAVI(stackPtr->depth = depth+1); stackPtr++;
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; NAVI(stackPtr->depth = depth+1); stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          NodeRef c = node->child(r); c.prefetch(); float n2 = tNear[r]; float f2 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n2; stackPtr->tFar = f2; NAVI(stackPtr->depth = depth+1); stackPtr++;
          assert(c != BVH4Hair::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; NAVI(depth = stackPtr[-1].depth); stackPtr--;
            continue;
          }

#if BVH4HAIR_WIDTH == 8

          while (mask) {
            r = __bscf(mask);
            c = node->child(r); c.prefetch(); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; NAVI(stackPtr->depth = depth+1); stackPtr++;
          }
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; NAVI(depth = stackPtr[-1].depth); stackPtr--;
#else
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; NAVI(stackPtr->depth = depth+1); stackPtr++;
          assert(c != BVH4Hair::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; NAVI(depth = stackPtr[-1].depth); stackPtr--;
#endif
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        PrimitiveIntersector::intersect(pre,ray,prim,num,bvh->scene);
      }
      AVX_ZERO_UPPER();
    }

    template<typename PrimitiveIntersector>
    void BVH4HairIntersector1<PrimitiveIntersector>::occluded(const BVH4Hair* bvh, Ray& ray) 
    {
      /*! perform per ray precalculations required by the primitive intersector */
      const Precalculations pre(ray);
      //const LinearSpace3fa pre(frame(ray.dir).transposed()); // FIXME: works only for normalized ray.dir

      /*! stack state */
      StackItem stack[stackSize];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      StackItem* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      stack[0].tNear = ray.tnear;
      stack[0].tFar = ray.tfar;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(simdf) : 1*sizeof(simdf);
      const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(simdf) : 3*sizeof(simdf);
      const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(simdf) : 5*sizeof(simdf);
      
      /*! load the ray into SIMD registers */
      const simd3f org(ray.org.x,ray.org.y,ray.org.z);
      const simd3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const simd3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const simd3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ref);
        simdf tNear = stackPtr->tNear;
        simdf tFar = min(stackPtr->tFar,ray.tfar);
        
        /*! if popped node is too far, pop next one */
#if BVH4HAIR_WIDTH == 4
        if (unlikely(_mm_cvtss_f32(tNear) > _mm_cvtss_f32(tFar)))
          continue;
#else
        if (unlikely(tNear[0] > tFar[0]))
          continue;
#endif

        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          size_t mask;
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! process nodes with aligned bounds */
          if (likely(cur.isAlignedNode()))
          {
            /*! single ray intersection with 4 boxes */
            const AlignedNode* node = cur.alignedNode();
            const size_t farX  = nearX ^ sizeof(simdf), farY  = nearY ^ sizeof(simdf), farZ  = nearZ ^ sizeof(simdf);
#if defined (__AVX2__)
            const simdf tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
            const simdf tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
            const simdf tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
            const simdf tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
            const simdf tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
            const simdf tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
            const simdf tNearX = (load4f((const char*)&node->lower_x+nearX) - org.x) * rdir.x;
            const simdf tNearY = (load4f((const char*)&node->lower_x+nearY) - org.y) * rdir.y;
            const simdf tNearZ = (load4f((const char*)&node->lower_x+nearZ) - org.z) * rdir.z;
            const simdf tFarX  = (load4f((const char*)&node->lower_x+farX ) - org.x) * rdir.x;
            const simdf tFarY  = (load4f((const char*)&node->lower_x+farY ) - org.y) * rdir.y;
            const simdf tFarZ  = (load4f((const char*)&node->lower_x+farZ ) - org.z) * rdir.z;
#endif
            
#if ((BVH4HAIR_WIDTH == 4) && defined(__SSE4_1__) || (BVH4HAIR_WIDTH == 8) && defined(__AVX2__))
            tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
            tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
            const simdb vmask = cast(tNear) > cast(tFar);
            mask = movemask(vmask)^((1<<BVH4Hair::N)-1);
#else
            tNear = max(tNearX,tNearY,tNearZ,tNear);
            tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
            const simdb vmask = tNear <= tFar;
            mask = movemask(vmask);
#endif
          }

          /*! process nodes with unaligned bounds */
          else {
            const UnalignedNode* node = cur.unalignedNode();
            mask = intersectBox(node,ray,org,dir,tNear,tFar);
          }

          /*! if no child is hit, pop next node */
          const Node* node = cur.node();
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          NodeRef c0 = node->child(r); c0.prefetch();

          if (likely(mask == 0)) {
            cur = c0; tNear = tNear[r]; tFar = tFar[r];
            assert(cur != BVH4Hair::emptyNode);
            continue;
          }
     
          /*! two children are hit, push far child, and continue with closer child */
           const float n0 = tNear[r]; const float f0 = tFar[r]; 
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const float n1 = tNear[r]; const float f1 = tFar[r];
          assert(c0 != BVH4Hair::emptyNode);
          assert(c1 != BVH4Hair::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd); 
            if (n0 < n1) { stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++; cur = c0; tNear = n0; tFar = f0; continue; }
            else         { stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++; cur = c1; tNear = n1; tFar = f1; continue; }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++;
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          NodeRef c = node->child(r); c.prefetch(); float n2 = tNear[r]; float f2 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n2; stackPtr->tFar = f2; stackPtr++;
          assert(c != BVH4Hair::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
            continue;
          }

#if BVH4HAIR_WIDTH == 8
          while (mask) {
            r = __bscf(mask);
            c = node->child(r); c.prefetch(); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; stackPtr++;
          }
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
#else
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; stackPtr++;
          assert(c != BVH4Hair::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
#endif
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        if (PrimitiveIntersector::occluded(pre,ray,prim,num,bvh->scene)) {
          ray.geomID = 0;
          break;
        }
      }
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR1(BVH4HairBezier1Intersector1,BVH4HairIntersector1<Bezier1Intersector1>);
    DEFINE_INTERSECTOR1(BVH4HairBezier1iIntersector1,BVH4HairIntersector1<Bezier1iIntersector1>);
  }
}
