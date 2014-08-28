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

#pragma once

#include "bvh4.h"
#include "common/ray8.h"
#include "common/stack_item.h"

namespace embree
{
  namespace isa 
  {
    /*! Single ray traversal for packets. */
    template<int types, typename PrimitiveIntersector8>
    class BVH4Intersector8Single 
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector8::Precalculations Precalculations;
      typedef typename PrimitiveIntersector8::Primitive Primitive;
      typedef typename BVH4::NodeRef NodeRef;
      typedef typename BVH4::Node Node;
      typedef StackItemT<NodeRef> StackItem;
      static const size_t stackSizeSingle = 1+3*BVH4::maxDepth;
      static const size_t stackSizeChunk = 4*BVH4::maxDepth+1;
      
    public:

      static __forceinline size_t intersectBox(const BVH4::UnalignedNode* node, 
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

    static __forceinline size_t intersectBox(const BVH4::UnalignedNodeMB* node, 
					     const sse3f& ray_org, const sse3f& ray_dir, float time,  
				      ssef& tNear, ssef& tFar)
    {
      const ssef t0 = ssef(1.0f)-time, t1 = time;
#if 0
      const AffineSpaceSSE3f xfm = t0*node->space0 + t1*node->space1;
      //const AffineSpaceSSE3f xfm = frame(normalize(ssef(0.5f)*node->space0.row2() + ssef(0.5f)*node->space1.row2())).transposed();
      //const LinearSpaceSSE3f xfm = frame(normalize(t0*node->space0.l.row2() + t1*node->space1.l.row2())).transposed();
      //const sse3f p = t0*node->space0.p + t1*node->space1.p;
      //const sse3f lower = t0*t0*node->t0s0.lower + t0*t1*node->t1s0_t0s1.lower + t1*t1*node->t1s1.lower;
      //const sse3f upper = t0*t0*node->t0s0.upper + t0*t1*node->t1s0_t0s1.upper + t1*t1*node->t1s1.upper;
      const sse3f lower = node->t1s0_t0s1.lower;
      const sse3f upper = node->t1s0_t0s1.upper;
#else
      //const AffineSpaceSSE3f xfm = t0*node->space0 + t1*node->space1;
      //const LinearSpaceSSE3f xfm = t0*node->space0.l + t1*node->space1.l;
      //const LinearSpaceSSE3f xfm = frame(normalize(t0*node->space0.l.row2() + t1*node->space1.l.row2())).transposed();
      //const sse3f p = t0*node->space0.p + t1*node->space1.p;
      const LinearSpaceSSE3f xfm = node->space0.l;
      const sse3f lower = t0*node->t0s0.lower + t1*node->t1s1.lower;
      const sse3f upper = t0*node->t0s0.upper + t1*node->t1s1.upper;
#endif
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
      
      static __forceinline void intersect1(const BVH4* bvh, NodeRef root, const size_t k, Precalculations& pre, 
					   Ray8& ray, const avx3f &ray_org, const avx3f &ray_dir, const avx3f &ray_rdir, const avxf &ray_tnear, const avxf &ray_tfar, const avx3i& nearXYZ)
    {
      /*! stack state */
      StackItemInt32<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
      StackItemInt32<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
      StackItemInt32<NodeRef>* stackEnd = stack + stackSizeSingle;
      stack[0].ptr = root;
      stack[0].dist = neg_inf;
      
      /*! load the ray into SIMD registers */
      const sse3f org(ray_org.x[k], ray_org.y[k], ray_org.z[k]);
      const sse3f dir(ray_dir.x[k], ray_dir.y[k], ray_dir.z[k]);
      const sse3f rdir(ray_rdir.x[k], ray_rdir.y[k], ray_rdir.z[k]);
      const sse3f org_rdir(org*rdir);
      ssef ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = nearXYZ.x[k];
      const size_t nearY = nearXYZ.y[k];
      const size_t nearZ = nearXYZ.z[k];
      
      /* pop loop */
      while (true) pop:
      {
	/*! pop next node */
	if (unlikely(stackPtr == stack)) break;
	stackPtr--;
	NodeRef cur = NodeRef(stackPtr->ptr);
	
	/*! if popped node is too far, pop next one */
	if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
	  continue;
	
	/* downtraversal loop */
	while (true)
	{
	  size_t mask; 
	  const Node* node;
	  ssef tNear, tFar;

	  /* process standard nodes */
          if (likely((types & 0x1) & cur.isNode())) 
	  {
	    STAT3(normal.trav_nodes,1,1,1);
          
	    /*! single ray intersection with 4 boxes */
	    node = cur.node();
	    const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
#if defined (__AVX2__)
	    const ssef tNearX = msub(load4f((const char*)node+nearX), rdir.x, org_rdir.x);
	    const ssef tNearY = msub(load4f((const char*)node+nearY), rdir.y, org_rdir.y);
	    const ssef tNearZ = msub(load4f((const char*)node+nearZ), rdir.z, org_rdir.z);
	    const ssef tFarX  = msub(load4f((const char*)node+farX ), rdir.x, org_rdir.x);
	    const ssef tFarY  = msub(load4f((const char*)node+farY ), rdir.y, org_rdir.y);
	    const ssef tFarZ  = msub(load4f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
	    const ssef tNearX = (load4f((const char*)node + nearX) - org.x) * rdir.x;
	    const ssef tNearY = (load4f((const char*)node + nearY) - org.y) * rdir.y;
	    const ssef tNearZ = (load4f((const char*)node + nearZ) - org.z) * rdir.z;
	    const ssef tFarX = (load4f((const char*)node + farX) - org.x) * rdir.x;
	    const ssef tFarY = (load4f((const char*)node + farY) - org.y) * rdir.y;
	    const ssef tFarZ = (load4f((const char*)node + farZ) - org.z) * rdir.z;
#endif
	    
#if defined(__SSE4_1__)
	    tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
	    tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
	    const sseb vmask = cast(tNear) > cast(tFar);
	    mask = movemask(vmask)^0xf;
#else
	    tNear = max(tNearX,tNearY,tNearZ,ray_near);
	    tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);
	    const sseb vmask = tNear <= tFar;
	    mask = movemask(vmask);
#endif
	  } 

	  /* process motion blur nodes */
	  else if (likely((types & 0x10) & cur.isNodeMB()))
	  {
	    STAT3(normal.trav_nodes,1,1,1);

	    /*! single ray intersection with 4 boxes */
	    const BVH4::NodeMB* nodeMB = cur.nodeMB(); node = (const BVH4::Node*) &nodeMB->lower_dx;
	    const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
	    const ssef* pNearX = (const ssef*)((const char*)nodeMB+nearX);
	    const ssef* pNearY = (const ssef*)((const char*)nodeMB+nearY);
	    const ssef* pNearZ = (const ssef*)((const char*)nodeMB+nearZ);
	    const ssef tNearX = (ssef(pNearX[0]) - org.x + ray.time[k]*pNearX[6]) * rdir.x;
	    const ssef tNearY = (ssef(pNearY[0]) - org.y + ray.time[k]*pNearY[6]) * rdir.y;
	    const ssef tNearZ = (ssef(pNearZ[0]) - org.z + ray.time[k]*pNearZ[6]) * rdir.z;
	    tNear = max(tNearX,tNearY,tNearZ,ray_near);
	    const ssef* pFarX = (const ssef*)((const char*)nodeMB+farX);
	    const ssef* pFarY = (const ssef*)((const char*)nodeMB+farY);
	    const ssef* pFarZ = (const ssef*)((const char*)nodeMB+farZ);
	    const ssef tFarX = (ssef(pFarX[0]) - org.x + ray.time[k]*pFarX[6]) * rdir.x;
	    const ssef tFarY = (ssef(pFarY[0]) - org.y + ray.time[k]*pFarY[6]) * rdir.y;
	    const ssef tFarZ = (ssef(pFarZ[0]) - org.z + ray.time[k]*pFarZ[6]) * rdir.z;
	    tFar = min(tFarX,tFarY,tFarZ,ray_far);
	    mask = movemask(tNear <= tFar);
	  }

	  /*! process nodes with unaligned bounds */
          else if (unlikely((types & 0x100) & cur.isUnalignedNode())) {
	    const BVH4::UnalignedNode* nodeU = cur.unalignedNode(); node = (const BVH4::Node*) &nodeU->naabb.l.vz.x; // FIXME: HACK
	    tNear = ray_near; tFar = ray_far;
            mask = intersectBox(nodeU,org,dir,tNear,tFar);
	  }

          /*! process nodes with unaligned bounds and motion blur */
          else if (unlikely((types & 0x1000) & cur.isUnalignedNodeMB())) {
	    const BVH4::UnalignedNodeMB* nodeMB = cur.unalignedNodeMB(); node = (const BVH4::Node*) &nodeMB->t1s1; // FIXME: HACK
	    tNear = ray_near; tFar = ray_far;
            mask = intersectBox(nodeMB,org,dir,ray.time[k],tNear,tFar);
	  }

	  /*! stop if we found a leaf */
	  else
	    break;
	  
	  /*! if no child is hit, pop next node */
	  if (unlikely(mask == 0))
	    goto pop;
	  
	  /*! one child is hit, continue with that child */
	  size_t r = __bscf(mask);
	  if (likely(mask == 0)) {
	    cur = node->child(r);
	    assert(cur != BVH4::emptyNode);
	    continue;
	  }
	  
	  /*! two children are hit, push far child, and continue with closer child */
	  NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
	  r = __bscf(mask);
	  NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
	  assert(c0 != BVH4::emptyNode);
	  assert(c1 != BVH4::emptyNode);
	  
	  if (likely(mask == 0)) {
	    assert(stackPtr < stackEnd);
	    if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
	    else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
	  }
	  
	  /*! Here starts the slow path for 3 or 4 hit children. We push
	   *  all nodes onto the stack to sort them there. */
	  assert(stackPtr < stackEnd);
	  stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
	  assert(stackPtr < stackEnd);
	  stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
	  
	  /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
	  assert(stackPtr < stackEnd);
	  r = __bscf(mask);
	  NodeRef c = node->child(r); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  
	  assert(c0 != BVH4::emptyNode);
	  if (likely(mask == 0)) {
	    sort(stackPtr[-1], stackPtr[-2], stackPtr[-3]);
	    cur = (NodeRef)stackPtr[-1].ptr; stackPtr--;
	    continue;
	  }
	  
	  /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
	  assert(stackPtr < stackEnd);
	  r = __bscf(mask);
	  c = node->child(r); d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  assert(c != BVH4::emptyNode);
	  sort(stackPtr[-1], stackPtr[-2], stackPtr[-3], stackPtr[-4]);
	  cur = (NodeRef)stackPtr[-1].ptr; stackPtr--;
	}
	
	/*! this is a leaf node */
	STAT3(normal.trav_leaves, 1, 1, 1);
	size_t num; Primitive* prim = (Primitive*)cur.leaf(num);
	PrimitiveIntersector8::intersect(pre, ray, k, prim, num, bvh->geometry);
	ray_far = ray.tfar[k];
      }
    }
    
      static __forceinline bool occluded1(const BVH4* bvh, NodeRef root, const size_t k, Precalculations& pre, 
					  Ray8& ray,const avx3f &ray_org, const avx3f &ray_dir, const avx3f &ray_rdir, const avxf &ray_tnear, const avxf &ray_tfar, const avx3i& nearXYZ)
    {
      /*! stack state */
      NodeRef stack[stackSizeSingle];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSizeSingle;
      stack[0]  = root;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = nearXYZ.x[k];
      const size_t nearY = nearXYZ.y[k];
      const size_t nearZ = nearXYZ.z[k];
      
      /*! load the ray into SIMD registers */
      const sse3f org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const sse3f dir(ray_dir.x[k], ray_dir.y[k], ray_dir.z[k]);
      const sse3f rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const sse3f norg = -org, org_rdir(org*rdir);
      const ssef ray_near(ray_tnear[k]), ray_far(ray_tfar[k]); 
      
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
	  size_t mask; 
	  const Node* node;
	  ssef tNear, tFar;

	  /* process standard nodes */
          if (likely((types & 0x1) & cur.isNode())) 
	  {
	    STAT3(normal.trav_nodes,1,1,1);
          
	    /*! single ray intersection with 4 boxes */
	    node = cur.node();
	    const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
#if defined (__AVX2__)
	    const ssef tNearX = msub(load4f((const char*)node+nearX), rdir.x, org_rdir.x);
	    const ssef tNearY = msub(load4f((const char*)node+nearY), rdir.y, org_rdir.y);
	    const ssef tNearZ = msub(load4f((const char*)node+nearZ), rdir.z, org_rdir.z);
	    const ssef tFarX  = msub(load4f((const char*)node+farX ), rdir.x, org_rdir.x);
	    const ssef tFarY  = msub(load4f((const char*)node+farY ), rdir.y, org_rdir.y);
	    const ssef tFarZ  = msub(load4f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
	    const ssef tNearX = (load4f((const char*)node + nearX) - org.x) * rdir.x;
	    const ssef tNearY = (load4f((const char*)node + nearY) - org.y) * rdir.y;
	    const ssef tNearZ = (load4f((const char*)node + nearZ) - org.z) * rdir.z;
	    const ssef tFarX = (load4f((const char*)node + farX) - org.x) * rdir.x;
	    const ssef tFarY = (load4f((const char*)node + farY) - org.y) * rdir.y;
	    const ssef tFarZ = (load4f((const char*)node + farZ) - org.z) * rdir.z;
#endif
	    
#if defined(__SSE4_1__)
	    tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
	    tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
	    const sseb vmask = cast(tNear) > cast(tFar);
	    mask = movemask(vmask)^0xf;
#else
	    tNear = max(tNearX,tNearY,tNearZ,ray_near);
	    tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);
	    const sseb vmask = tNear <= tFar;
	    mask = movemask(vmask);
#endif
	  } 

	  /* process motion blur nodes */
	  else if (likely((types & 0x10) & cur.isNodeMB()))
	  {
	    STAT3(normal.trav_nodes,1,1,1);

	    /*! single ray intersection with 4 boxes */
	    const BVH4::NodeMB* nodeMB = cur.nodeMB(); node = (const BVH4::Node*) &nodeMB->lower_dx;
	    const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
	    const ssef* pNearX = (const ssef*)((const char*)nodeMB+nearX);
	    const ssef* pNearY = (const ssef*)((const char*)nodeMB+nearY);
	    const ssef* pNearZ = (const ssef*)((const char*)nodeMB+nearZ);
	    const ssef tNearX = (norg.x + ssef(pNearX[0]) + ray.time[k]*pNearX[6]) * rdir.x;
	    const ssef tNearY = (norg.y + ssef(pNearY[0]) + ray.time[k]*pNearY[6]) * rdir.y;
	    const ssef tNearZ = (norg.z + ssef(pNearZ[0]) + ray.time[k]*pNearZ[6]) * rdir.z;
	    tNear = max(tNearX,tNearY,tNearZ,ray_near);
	    const ssef* pFarX = (const ssef*)((const char*)nodeMB+farX);
	    const ssef* pFarY = (const ssef*)((const char*)nodeMB+farY);
	    const ssef* pFarZ = (const ssef*)((const char*)nodeMB+farZ);
	    const ssef tFarX = (norg.x + ssef(pFarX[0]) + ray.time[k]*pFarX[6]) * rdir.x;
	    const ssef tFarY = (norg.y + ssef(pFarY[0]) + ray.time[k]*pFarY[6]) * rdir.y;
	    const ssef tFarZ = (norg.z + ssef(pFarZ[0]) + ray.time[k]*pFarZ[6]) * rdir.z;
	    tFar = min(tFarX,tFarY,tFarZ,ray_far);
	    mask = movemask(tNear <= tFar);
	  }

	  /*! process nodes with unaligned bounds */
          else if (unlikely((types & 0x100) & cur.isUnalignedNode())) {
	    const BVH4::UnalignedNode* nodeU = cur.unalignedNode(); node = (const BVH4::Node*) &nodeU->naabb.l.vz.x; // FIXME: HACK
	    tNear = ray_near; tFar = ray_far;
            mask = intersectBox(nodeU,org,dir,tNear,tFar);
	  }

          /*! process nodes with unaligned bounds and motion blur */
          else if (unlikely((types & 0x1000) & cur.isUnalignedNodeMB())) {
	    const BVH4::UnalignedNodeMB* nodeMB = cur.unalignedNodeMB(); node = (const BVH4::Node*) &nodeMB->t1s1; // FIXME: HACK
	    tNear = ray_near; tFar = ray_far;
            mask = intersectBox(nodeMB,org,dir,ray.time[k],tNear,tFar);
	  }

	  /*! stop if we found a leaf */
	  else
	    break;
	  
	  /*! if no child is hit, pop next node */
	  if (unlikely(mask == 0))
	    goto pop;
          
	  /*! one child is hit, continue with that child */
	  size_t r = __bscf(mask);
	  if (likely(mask == 0)) {
	    cur = node->child(r);
	    assert(cur != BVH4::emptyNode);
	    continue;
	  }
          
	  /*! two children are hit, push far child, and continue with closer child */
	  NodeRef c0 = node->child(r); c0.prefetch(types); unsigned int d0 = ((unsigned int*)&tNear)[r];
	  r = __bscf(mask);
	  NodeRef c1 = node->child(r); c1.prefetch(types); unsigned int d1 = ((unsigned int*)&tNear)[r];
	  assert(c0 != BVH4::emptyNode);
	  assert(c1 != BVH4::emptyNode);
	  if (likely(mask == 0)) {
	    assert(stackPtr < stackEnd);
	    if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; continue; }
	    else         { *stackPtr = c0; stackPtr++; cur = c1; continue; }
	  }
	  assert(stackPtr < stackEnd);
	  stackPtr[0] = c0; 
	  assert(stackPtr < stackEnd);
	  stackPtr[1] = c1; 
	  
	  stackPtr+=2;
	  
	  /*! three children are hit */
	  r = __bscf(mask);
	  cur = node->child(r); 
	  assert(cur != BVH4::emptyNode);
	  if (likely(mask == 0)) continue;
	  
	  assert(stackPtr < stackEnd);
	  *stackPtr = cur; stackPtr++;
          
	  /*! four children are hit */
	  cur = node->child(3);
	  assert(cur != BVH4::emptyNode);
	}
        
	/*! this is a leaf node */
	STAT3(shadow.trav_leaves,1,1,1);
	size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
	if (PrimitiveIntersector8::occluded(pre,ray,k,prim,num,bvh->geometry)) {
	  ray.geomID[k] = 0;
	  return true;
	}
      }
      return false;
    }

      static void intersect(avxb* valid, BVH4* bvh, Ray8& ray);
      static void occluded (avxb* valid, BVH4* bvh, Ray8& ray);
    };
  }
}
