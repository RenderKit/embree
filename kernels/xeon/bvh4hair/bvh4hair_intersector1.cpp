// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

namespace embree
{ 
  namespace isa
  {
    template<typename PrimitiveIntersector>
    __forceinline size_t BVH4HairIntersector1<PrimitiveIntersector>::intersectBox(const BVH4Hair::AlignedNode* node, 
                                                                                  const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, 
                                                                                  const size_t nearX, const size_t nearY, const size_t nearZ,
                                                                                  ssef& tNear, ssef& tFar)
    {
      const BBoxSSE3f bounds = node->getBounds(nearX,nearY,nearZ);

#if defined (__AVX2__)
      const ssef tNearX = msub(bounds.lower.x, rdir.x, org_rdir.x);
      const ssef tNearY = msub(bounds.lower.y, rdir.y, org_rdir.y);
      const ssef tNearZ = msub(bounds.lower.z, rdir.z, org_rdir.z);
      const ssef tFarX  = msub(bounds.upper.x, rdir.x, org_rdir.x);
      const ssef tFarY  = msub(bounds.upper.y, rdir.y, org_rdir.y);
      const ssef tFarZ  = msub(bounds.upper.z, rdir.z, org_rdir.z);
#else
      const ssef tNearX = (bounds.lower.x - org.x) * rdir.x;
      const ssef tNearY = (bounds.lower.y - org.y) * rdir.y;
      const ssef tNearZ = (bounds.lower.z - org.z) * rdir.z;
      const ssef tFarX  = (bounds.upper.x - org.x) * rdir.x;
      const ssef tFarY  = (bounds.upper.y - org.y) * rdir.y;
      const ssef tFarZ  = (bounds.upper.z - org.z) * rdir.z;
#endif
      
#if defined(__SSE4_1__)
      tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
      tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
      const sseb vmask = cast(tNear) > cast(tFar);
      return movemask(vmask)^((1<<BVH4Hair::N)-1);
#else
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#endif
    }

    template<typename PrimitiveIntersector>
    __forceinline size_t BVH4HairIntersector1<PrimitiveIntersector>::intersectBox(const BVH4Hair::AlignedNodeMB* node, 
                                                                                  const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const float time, 
                                                                                  const size_t nearX, const size_t nearY, const size_t nearZ,
                                                                                  ssef& tNear, ssef& tFar)
    {
      const BBoxSSE3f bounds = node->getBounds(time,nearX,nearY,nearZ);

#if defined (__AVX2__)
      const ssef tNearX = msub(bounds.lower.x, rdir.x, org_rdir.x);
      const ssef tNearY = msub(bounds.lower.y, rdir.y, org_rdir.y);
      const ssef tNearZ = msub(bounds.lower.z, rdir.z, org_rdir.z);
      const ssef tFarX  = msub(bounds.upper.x, rdir.x, org_rdir.x);
      const ssef tFarY  = msub(bounds.upper.y, rdir.y, org_rdir.y);
      const ssef tFarZ  = msub(bounds.upper.z, rdir.z, org_rdir.z);
#else
      const ssef tNearX = (bounds.lower.x - org.x) * rdir.x;
      const ssef tNearY = (bounds.lower.y - org.y) * rdir.y;
      const ssef tNearZ = (bounds.lower.z - org.z) * rdir.z;
      const ssef tFarX  = (bounds.upper.x - org.x) * rdir.x;
      const ssef tFarY  = (bounds.upper.y - org.y) * rdir.y;
      const ssef tFarZ  = (bounds.upper.z - org.z) * rdir.z;
#endif
      
#if defined(__SSE4_1__)
      tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
      tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
      const sseb vmask = cast(tNear) > cast(tFar);
      return movemask(vmask)^((1<<BVH4Hair::N)-1);
#else
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#endif
    }

#if BVH4HAIR_COMPRESS_UNALIGNED_NODES
    template<typename PrimitiveIntersector>
    __forceinline size_t BVH4HairIntersector1<PrimitiveIntersector>::intersectBox(const BVH4Hair::CompressedUnalignedNode* node, Ray& ray, 
                                                                                  const sse3f& ray_org, const sse3f& ray_dir, 
                                                                                  ssef& tNear, ssef& tFar)
    {
      const LinearSpace3fa xfm = node->getXfm();
      //const Vec3fa dir = xfmVector(xfm,ray.dir);
      const Vec3fa dir = madd(xfm.vx,(Vec3fa)ray_dir.x,madd(xfm.vy,(Vec3fa)ray_dir.y,xfm.vz*(Vec3fa)ray_dir.z));
      //const sse3f rdir = Vec3fa(one)/dir; 
      const sse3f rdir = rcp_safe(dir); 
      //const Vec3fa org = xfmPoint(xfm,ray.org);
      const Vec3fa org = madd(xfm.vx,(Vec3fa)ray_org.x,madd(xfm.vy,(Vec3fa)ray_org.y,xfm.vz*(Vec3fa)ray_org.z));
      const sse3f vorg  = sse3f(org);
      const sse3f vrdir = sse3f(rdir);
      const BBoxSSE3f bounds = node->getBounds();
      const sse3f tLowerXYZ = (bounds.lower - vorg) * vrdir;
      const sse3f tUpperXYZ = (bounds.upper - vorg) * vrdir;

#if defined(__SSE4_1__)
      const ssef tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
      tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#else
      const ssef tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#endif
    }
#endif

    template<typename PrimitiveIntersector>
    __forceinline size_t BVH4HairIntersector1<PrimitiveIntersector>::intersectBox(const BVH4Hair::UncompressedUnalignedNode* node, Ray& ray, 
                                                                                  const sse3f& ray_org, const sse3f& ray_dir, 
                                                                                  ssef& tNear, ssef& tFar)
    {
      const AffineSpaceSSE3f xfm = node->naabb;
      const sse3f dir = xfmVector(xfm,ray_dir);
      //const sse3f nrdir = sse3f(ssef(-1.0f))/dir;
      const sse3f nrdir = sse3f(ssef(-1.0f))*rcp_safe(dir);
      const sse3f org = xfmPoint(xfm,ray_org);
      const sse3f tLowerXYZ = org * nrdir;     // (Vec3fa(zero) - org) * rdir;
      const sse3f tUpperXYZ = tLowerXYZ - nrdir; // (Vec3fa(one ) - org) * rdir;
      
#if defined(__SSE4_1__)
      const ssef tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
      tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#else
      const ssef tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#endif
    }

    template<typename PrimitiveIntersector>
    __forceinline size_t BVH4HairIntersector1<PrimitiveIntersector>::intersectBox(const BVH4Hair::UnalignedNodeMB* node, Ray& ray,
                                                                                  const sse3f& ray_org, const sse3f& ray_dir, 
                                                                                  ssef& tNear, ssef& tFar)
    {
      const ssef t0 = ssef(1.0f)-ray.time, t1 = ray.time;
      const LinearSpaceSSE3f xfm = t0*node->space0 + t1*node->space1;
      const sse3f lower = t0*t0*node->t0s0.lower + t0*t1*node->t1s0_t0s1.lower + t1*t1*node->t1s1.lower;
      const sse3f upper = t0*t0*node->t0s0.upper + t0*t1*node->t1s0_t0s1.upper + t1*t1*node->t1s1.upper;
      const BBoxSSE3f bounds(lower,upper);

      const sse3f dir = xfmVector(xfm,ray_dir);
      const sse3f rdir = rcp_safe(dir); 
      const sse3f org = xfmPoint(xfm,ray_org);
      const sse3f tLowerXYZ = (bounds.lower - org) * rdir;
      const sse3f tUpperXYZ = (bounds.upper - org) * rdir;

#if defined(__SSE4_1__)
      const ssef tNearX = mini(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = mini(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = mini(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = maxi(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = maxi(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = maxi(tLowerXYZ.z,tUpperXYZ.z);
      tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
      tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#else
      const ssef tNearX = min(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tNearY = min(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tNearZ = min(tLowerXYZ.z,tUpperXYZ.z);
      const ssef tFarX  = max(tLowerXYZ.x,tUpperXYZ.x);
      const ssef tFarY  = max(tLowerXYZ.y,tUpperXYZ.y);
      const ssef tFarZ  = max(tLowerXYZ.z,tUpperXYZ.z);
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      const sseb vmask = tNear <= tFar;
      return movemask(vmask);
#endif
    }

    template<typename PrimitiveIntersector>
    void BVH4HairIntersector1<PrimitiveIntersector>::intersect(const BVH4Hair* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      typename PrimitiveIntersector::Precalculations pre(ray);

      /*! stack state */
      StackItemNearFar stack[stackSize];  //!< stack of nodes 
      StackItemNearFar* stackPtr = stack+1;        //!< current stack pointer
      StackItemNearFar* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      stack[0].tNear = ray.tnear;
      stack[0].tFar = ray.tfar;
            
      /*! load the ray into SIMD registers */
      const sse3f org(ray.org.x,ray.org.y,ray.org.z);
      const sse3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);

      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0.0f ? 0*BVH4Hair::AlignedNode::stride : 1*BVH4Hair::AlignedNode::stride;
      const size_t nearY = ray_rdir.y >= 0.0f ? 0*BVH4Hair::AlignedNode::stride : 1*BVH4Hair::AlignedNode::stride;
      const size_t nearZ = ray_rdir.z >= 0.0f ? 0*BVH4Hair::AlignedNode::stride : 1*BVH4Hair::AlignedNode::stride;

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        /*for (size_t i=1; i<stackPtr-&stack[0]; i++)
          if (stack[i-1].tNear < stack[i+0].tNear)
            StackItemNearFar::swap2(stack[i-1],stack[i+0]);*/
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ref);
        ssef tNear = stackPtr->tNear;
        ssef tFar = min(stackPtr->tFar,ray.tfar);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(_mm_cvtss_f32(tNear) > _mm_cvtss_f32(tFar)))
          continue;

        /* downtraversal loop */
        while (true)
        {
          /*! process nodes with aligned bounds */
          size_t mask;
          if (likely(cur.isAlignedNode()))
            mask = intersectBox(cur.alignedNode(),org,rdir,org_rdir,nearX,nearY,nearZ,tNear,tFar);

          /*! process nodes with unaligned bounds */
          else if (unlikely(cur.isUnalignedNode()))
            mask = intersectBox(cur.unalignedNode(),ray,org,dir,tNear,tFar);

          /*! process nodes with aligned bounds and motion blur */
          else if (unlikely(cur.isAlignedNodeMB()))
            mask = intersectBox(cur.alignedNodeMB(),org,rdir,org_rdir,ray.time,nearX,nearY,nearZ,tNear,tFar);

          /*! process nodes with unaligned bounds and motion blur */
          else if (unlikely(cur.isUnalignedNodeMB()))
            mask = intersectBox(cur.unalignedNodeMB(),ray,org,dir,tNear,tFar);

          /*! otherwise this is a leaf */
          else break;

          /*! if no child is hit, pop next node */
          STAT3(normal.trav_nodes,1,1,1);
          const Node* node = cur.node();
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          NodeRef c0 = node->child(r);
          c0.prefetch();

          if (likely(mask == 0)) {
            cur = c0;  tNear = tNear[r]; tFar = tFar[r];
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
              stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++; 
              cur = c0; tNear = n0; tFar = f0;
              continue; 
            }
            else { 
              stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++; 
              cur = c1; tNear = n1; tFar = f1;
              continue; 
            }
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

          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; stackPtr++;
          assert(c != BVH4Hair::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
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
      typename PrimitiveIntersector::Precalculations pre(ray);

      /*! stack state */
      StackItemNearFar stack[stackSize];  //!< stack of nodes 
      StackItemNearFar* stackPtr = stack+1;        //!< current stack pointer
      StackItemNearFar* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      stack[0].tNear = ray.tnear;
      stack[0].tFar = ray.tfar;
            
      /*! load the ray into SIMD registers */
      const sse3f org(ray.org.x,ray.org.y,ray.org.z);
      const sse3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);

      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0.0f ? 0*BVH4Hair::AlignedNode::stride : 1*BVH4Hair::AlignedNode::stride;
      const size_t nearY = ray_rdir.y >= 0.0f ? 0*BVH4Hair::AlignedNode::stride : 1*BVH4Hair::AlignedNode::stride;
      const size_t nearZ = ray_rdir.z >= 0.0f ? 0*BVH4Hair::AlignedNode::stride : 1*BVH4Hair::AlignedNode::stride;

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ref);
        ssef tNear = stackPtr->tNear;
        ssef tFar = min(stackPtr->tFar,ray.tfar);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(_mm_cvtss_f32(tNear) > _mm_cvtss_f32(tFar)))
          continue;

        /* downtraversal loop */
        while (true)
        {
          /*! process nodes with aligned bounds */
          size_t mask;
          if (likely(cur.isAlignedNode()))
            mask = intersectBox(cur.alignedNode(),org,rdir,org_rdir,nearX,nearY,nearZ,tNear,tFar);

          /*! process nodes with unaligned bounds */
          else if (unlikely(cur.isUnalignedNode()))
            mask = intersectBox(cur.unalignedNode(),ray,org,dir,tNear,tFar);

          /*! process nodes with aligned bounds and motion blur */
          else if (unlikely(cur.isAlignedNodeMB()))
            mask = intersectBox(cur.alignedNodeMB(),org,rdir,org_rdir,ray.time,nearX,nearY,nearZ,tNear,tFar);

          /*! process nodes with unaligned bounds and motion blur */
          else if (unlikely(cur.isUnalignedNodeMB()))
            mask = intersectBox(cur.unalignedNodeMB(),ray,org,dir,tNear,tFar);

          /*! otherwise this is a leaf */
          else break;

          /*! if no child is hit, pop next node */
          STAT3(shadow.trav_nodes,1,1,1);
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

          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; stackPtr++;
          assert(c != BVH4Hair::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
        }
        
        /*! this is a leaf node */
        STAT3(shadow.trav_leaves,1,1,1);
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

    //DEFINE_INTERSECTOR1(BVH4HairBezier1MBIntersector1,BVH4HairIntersector1<Bezier1Intersector1MB>);
    DEFINE_INTERSECTOR1(BVH4HairBezier1iMBIntersector1,BVH4HairIntersector1<Bezier1iIntersector1MB>);
  }
}
