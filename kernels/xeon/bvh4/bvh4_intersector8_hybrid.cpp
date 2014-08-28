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

#include "bvh4_intersector8_hybrid.h"

#include "geometry/triangle4_intersector8_moeller.h"
#include "geometry/triangle8_intersector8_moeller.h"
#include "geometry/triangle4v_intersector8_pluecker.h"

#define SWITCH_THRESHOLD 5

#define SWITCH_DURING_DOWN_TRAVERSAL 1

namespace embree
{
  namespace isa
  {
    template<int types, typename PrimitiveIntersector8>
    __forceinline void BVH4Intersector8Hybrid<types,PrimitiveIntersector8>::intersect1(const BVH4* bvh, NodeRef root, const size_t k, Precalculations& pre, 
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
          if (likely((types & 0x1) && cur.isNode())) 
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
	  else if (likely((types & 0x10) && cur.isNodeMB()))
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
    
    template<int types, typename PrimitiveIntersector8>
    __forceinline bool BVH4Intersector8Hybrid<types,PrimitiveIntersector8>::occluded1(const BVH4* bvh, NodeRef root, const size_t k, Precalculations& pre, 
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
          if (likely((types & 0x1) && cur.isNode())) 
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
	  else if (likely((types & 0x10) && cur.isNodeMB()))
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
    
    /* ray/box intersection */
    __forceinline avxb intersectBox(const Ray8& ray, const avxf& ray_tfar, const avx3f& rdir, const BVH4::NodeMB* node, const int i, avxf& dist) 
    {
      const avxf lower_x = avxf(node->lower_x[i]) + ray.time * avxf(node->lower_dx[i]);
      const avxf lower_y = avxf(node->lower_y[i]) + ray.time * avxf(node->lower_dy[i]);
      const avxf lower_z = avxf(node->lower_z[i]) + ray.time * avxf(node->lower_dz[i]);
      const avxf upper_x = avxf(node->upper_x[i]) + ray.time * avxf(node->upper_dx[i]);
      const avxf upper_y = avxf(node->upper_y[i]) + ray.time * avxf(node->upper_dy[i]);
      const avxf upper_z = avxf(node->upper_z[i]) + ray.time * avxf(node->upper_dz[i]);
      
      const avxf dminx = (lower_x - ray.org.x) * rdir.x;
      const avxf dminy = (lower_y - ray.org.y) * rdir.y;
      const avxf dminz = (lower_z - ray.org.z) * rdir.z;
      const avxf dmaxx = (upper_x - ray.org.x) * rdir.x;
      const avxf dmaxy = (upper_y - ray.org.y) * rdir.y;
      const avxf dmaxz = (upper_z - ray.org.z) * rdir.z;
      
      const avxf dlowerx = min(dminx,dmaxx);
      const avxf dlowery = min(dminy,dmaxy);
      const avxf dlowerz = min(dminz,dmaxz);
      
      const avxf dupperx = max(dminx,dmaxx);
      const avxf duppery = max(dminy,dmaxy);
      const avxf dupperz = max(dminz,dmaxz);
      
      const avxf near = max(dlowerx,dlowery,dlowerz,ray.tnear);
      const avxf far  = min(dupperx,duppery,dupperz,ray_tfar );
      dist = near;
      
      return near <= far;
    }

    template<int types, typename PrimitiveIntersector8>
    void BVH4Intersector8Hybrid<types,PrimitiveIntersector8>::intersect(avxb* valid_i, BVH4* bvh, Ray8& ray)
    {
      /* load ray */
      const avxb valid0 = *valid_i;
      avx3f ray_org = ray.org;
      avx3f ray_dir = ray.dir;
      avxf ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const avx3f rdir = rcp_safe(ray_dir);
      const avx3f org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,avxf(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,avxf(neg_inf));
      const avxf inf = avxf(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      avx3i nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,avxi(0*(int)sizeof(ssef)),avxi(1*(int)sizeof(ssef)));
      nearXYZ.y = select(rdir.y >= 0.0f,avxi(2*(int)sizeof(ssef)),avxi(3*(int)sizeof(ssef)));
      nearXYZ.z = select(rdir.z >= 0.0f,avxi(4*(int)sizeof(ssef)),avxi(5*(int)sizeof(ssef)));

      /* allocate stack and push root node */
      avxf    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      avxf*    __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef curNode = *sptr_node;
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* cull node if behind closest hit point */
        avxf curDist = *sptr_near;
        const avxb active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;
        
        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            intersect1(bvh, curNode, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
          }
          ray_tfar = min(ray_tfar,ray.tfar);
          continue;
        }
#endif

        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && curNode.isNode()))
          {
	    const avxb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const Node* __restrict__ const node = curNode.node();
	    
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
	      
#if defined(__AVX2__)
	      const avxf lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
	      const avxf lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
	      const avxf lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
	      const avxf lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
	      const avxf lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
	      const avxf lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
	      const avxf lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
	      const avxf lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
	      const avxb lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
	      const avxf lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
	      const avxf lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
	      const avxf lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
	      const avxf lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
	      const avxf lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
	      const avxf lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
	      const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
	      const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
	      const avxb lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const avxf childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }     
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = curNode;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && curNode.isNodeMB()))
	  {
	    const avxb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const BVH4::NodeMB* __restrict__ const node = curNode.nodeMB();
          
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;

	      avxf lnearP;
	      const avxb lhit = intersectBox(ray,ray_tfar,rdir,node,i,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const avxf childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = curNode;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  else 
	    break;
	}
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
        const avxb valid_leaf = ray_tfar > curDist;

        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Primitive* prim = (Primitive*) curNode.leaf(items);
        PrimitiveIntersector8::intersect(valid_leaf,pre,ray,prim,items,bvh->geometry);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
      }
      AVX_ZERO_UPPER();
    }

    
    template<int types, typename PrimitiveIntersector8>
    void BVH4Intersector8Hybrid<types,PrimitiveIntersector8>::occluded(avxb* valid_i, BVH4* bvh, Ray8& ray)
    {
      /* load ray */
      const avxb valid = *valid_i;
      avxb terminated = !valid;
      avx3f ray_org = ray.org, ray_dir = ray.dir;
      avxf ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const avx3f rdir = rcp_safe(ray_dir);
      const avx3f org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,avxf(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,avxf(neg_inf));
      const avxf inf = avxf(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      avx3i nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,avxi(0*(int)sizeof(ssef)),avxi(1*(int)sizeof(ssef)));
      nearXYZ.y = select(rdir.y >= 0.0f,avxi(2*(int)sizeof(ssef)),avxi(3*(int)sizeof(ssef)));
      nearXYZ.z = select(rdir.z >= 0.0f,avxi(4*(int)sizeof(ssef)),avxi(5*(int)sizeof(ssef)));

      /* allocate stack and push root node */
      avxf    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      avxf*    __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef curNode = *sptr_node;
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        /* cull node if behind closest hit point */
        avxf curDist = *sptr_near;
        const avxb active = curDist < ray_tfar;
        if (unlikely(none(active))) 
          continue;
        
        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            if (occluded1(bvh,curNode,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
              terminated[i] = -1;
          }
          if (all(terminated)) break;
          ray_tfar = select(terminated,avxf(neg_inf),ray_tfar);
          continue;
        }
#endif
                
        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && curNode.isNode()))
          {
	    const avxb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const Node* __restrict__ const node = curNode.node();
	    
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
	      
#if defined(__AVX2__)
	      const avxf lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
	      const avxf lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
	      const avxf lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
	      const avxf lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
	      const avxf lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
	      const avxf lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
	      const avxf lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
	      const avxf lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
	      const avxb lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
	      const avxf lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
	      const avxf lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
	      const avxf lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
	      const avxf lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
	      const avxf lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
	      const avxf lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
	      const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
	      const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
	      const avxb lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const avxf childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }     
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = curNode;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && curNode.isNodeMB()))
	  {
	    const avxb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const BVH4::NodeMB* __restrict__ const node = curNode.nodeMB();
          
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;

	      avxf lnearP;
	      const avxb lhit = intersectBox(ray,ray_tfar,rdir,node,i,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const avxf childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = curNode;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  else 
	    break;
	}
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        
        /* intersect leaf */
        const avxb valid_leaf = ray_tfar > curDist;

        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Primitive* prim = (Primitive*) curNode.leaf(items);
        terminated |= PrimitiveIntersector8::occluded(!terminated,pre,ray,prim,items,bvh->geometry);
        if (all(terminated)) break;
        ray_tfar = select(terminated,avxf(neg_inf),ray_tfar);
      }
      store8i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
    
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoeller, BVH4Intersector8Hybrid<0x1 COMMA Triangle4Intersector8MoellerTrumbore<true> >);
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoellerNoFilter, BVH4Intersector8Hybrid<0x1 COMMA Triangle4Intersector8MoellerTrumbore<false> >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8HybridMoeller, BVH4Intersector8Hybrid<0x1 COMMA Triangle8Intersector8MoellerTrumbore<true> >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8HybridMoellerNoFilter, BVH4Intersector8Hybrid<0x1 COMMA Triangle8Intersector8MoellerTrumbore<false> >);
    DEFINE_INTERSECTOR8(BVH4Triangle4vIntersector8HybridPluecker, BVH4Intersector8Hybrid<0x1 COMMA Triangle4vIntersector8Pluecker>);
  }
}
