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

#include "bvh4_intersector1.h"

#include "geometry/bezier1_intersector1.h"
#include "geometry/bezier1i_intersector1.h"
#include "geometry/triangle1_intersector1_moeller.h"
#include "geometry/triangle4_intersector1_moeller.h"
#if defined(__AVX__)
#include "geometry/triangle8_intersector1_moeller.h"
#endif
#include "geometry/triangle1v_intersector1_pluecker.h"
#include "geometry/triangle4v_intersector1_pluecker.h"
#include "geometry/triangle4i_intersector1.h"
#include "geometry/virtual_accel_intersector1.h"

namespace embree
{ 
  namespace isa
  {
    template<int types, typename PrimitiveIntersector>
    void BVH4Intersector1<types,PrimitiveIntersector>::intersect(const BVH4* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray);

      /*! stack state */
      StackItemInt32<NodeRef> stack[stackSize];  //!< stack of nodes 
      StackItemInt32<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemInt32<NodeRef>* stackEnd = stack+stackSize;
      stack[0].ptr = bvh->root;
      stack[0].dist = neg_inf;
            
      /*! load the ray into SIMD registers */
      const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const ssef  ray_near(ray.tnear);
      ssef ray_far(ray.tfar);

      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray_rdir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);

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
	    const ssef tNearX = (norg.x + load4f((const char*)node+nearX)) * rdir.x;
	    const ssef tNearY = (norg.y + load4f((const char*)node+nearY)) * rdir.y;
	    const ssef tNearZ = (norg.z + load4f((const char*)node+nearZ)) * rdir.z;
	    const ssef tFarX  = (norg.x + load4f((const char*)node+farX )) * rdir.x;
	    const ssef tFarY  = (norg.y + load4f((const char*)node+farY )) * rdir.y;
	    const ssef tFarZ  = (norg.z + load4f((const char*)node+farZ )) * rdir.z;
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
	    const BVH4::NodeMB* nodeMB = cur.nodeMB(); node = cur.node();
	    const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
	    const ssef* pNearX = (const ssef*)((const char*)nodeMB+nearX);
	    const ssef* pNearY = (const ssef*)((const char*)nodeMB+nearY);
	    const ssef* pNearZ = (const ssef*)((const char*)nodeMB+nearZ);
	    const ssef tNearX = (norg.x + ssef(pNearX[0]) + ray.time*pNearX[6]) * rdir.x;
	    const ssef tNearY = (norg.y + ssef(pNearY[0]) + ray.time*pNearY[6]) * rdir.y;
	    const ssef tNearZ = (norg.z + ssef(pNearZ[0]) + ray.time*pNearZ[6]) * rdir.z;
	    tNear = max(tNearX,tNearY,tNearZ,ray_near);
	    const ssef* pFarX = (const ssef*)((const char*)nodeMB+farX);
	    const ssef* pFarY = (const ssef*)((const char*)nodeMB+farY);
	    const ssef* pFarZ = (const ssef*)((const char*)nodeMB+farZ);
	    const ssef tFarX = (norg.x + ssef(pFarX[0]) + ray.time*pFarX[6]) * rdir.x;
	    const ssef tFarY = (norg.y + ssef(pFarY[0]) + ray.time*pFarY[6]) * rdir.y;
	    const ssef tFarZ = (norg.z + ssef(pFarZ[0]) + ray.time*pFarZ[6]) * rdir.z;
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
            cur = node->child(r); cur.prefetch();
            assert(cur != BVH4::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
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
          NodeRef c = node->child(r); c.prefetch(); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH4::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            continue;
          }
          
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH4::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        PrimitiveIntersector::intersect(pre,ray,prim,num,bvh->geometry);
        ray_far = ray.tfar;
      }
      AVX_ZERO_UPPER();
    }
    
    template<int types, typename PrimitiveIntersector>
    void BVH4Intersector1<types,PrimitiveIntersector>::occluded(const BVH4* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray);

      /*! stack state */
      NodeRef stack[stackSize];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSize;
      stack[0] = bvh->root;
      
      /*! load the ray into SIMD registers */
      const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const ssef  ray_near(ray.tnear);
      ssef ray_far(ray.tfar);

      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0 ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray_rdir.y >= 0 ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray_rdir.z >= 0 ? 4*sizeof(ssef) : 5*sizeof(ssef);      
      
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
	    const ssef tNearX = (norg.x + load4f((const char*)node+nearX)) * rdir.x;
	    const ssef tNearY = (norg.y + load4f((const char*)node+nearY)) * rdir.y;
	    const ssef tNearZ = (norg.z + load4f((const char*)node+nearZ)) * rdir.z;
	    const ssef tFarX  = (norg.x + load4f((const char*)node+farX )) * rdir.x;
	    const ssef tFarY  = (norg.y + load4f((const char*)node+farY )) * rdir.y;
	    const ssef tFarZ  = (norg.z + load4f((const char*)node+farZ )) * rdir.z;
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
	  else if ((types & 0x10) && likely(cur.isNodeMB()))
	  {
	    STAT3(normal.trav_nodes,1,1,1);

	    /*! single ray intersection with 4 boxes */
	    const BVH4::NodeMB* nodeMB = cur.nodeMB(); node = cur.node();
	    const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
	    const ssef* pNearX = (const ssef*)((const char*)nodeMB+nearX);
	    const ssef* pNearY = (const ssef*)((const char*)nodeMB+nearY);
	    const ssef* pNearZ = (const ssef*)((const char*)nodeMB+nearZ);
	    const ssef tNearX = (norg.x + ssef(pNearX[0]) + ray.time*pNearX[6]) * rdir.x;
	    const ssef tNearY = (norg.y + ssef(pNearY[0]) + ray.time*pNearY[6]) * rdir.y;
	    const ssef tNearZ = (norg.z + ssef(pNearZ[0]) + ray.time*pNearZ[6]) * rdir.z;
	    tNear = max(tNearX,tNearY,tNearZ,ray_near);
	    const ssef* pFarX = (const ssef*)((const char*)nodeMB+farX);
	    const ssef* pFarY = (const ssef*)((const char*)nodeMB+farY);
	    const ssef* pFarZ = (const ssef*)((const char*)nodeMB+farZ);
	    const ssef tFarX = (norg.x + ssef(pFarX[0]) + ray.time*pFarX[6]) * rdir.x;
	    const ssef tFarY = (norg.y + ssef(pFarY[0]) + ray.time*pFarY[6]) * rdir.y;
	    const ssef tFarZ = (norg.z + ssef(pFarZ[0]) + ray.time*pFarZ[6]) * rdir.z;
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
            cur = node->child(r); cur.prefetch(); 
            assert(cur != BVH4::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH4::emptyNode);
          assert(c1 != BVH4::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd);
            if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; continue; }
            else         { *stackPtr = c0; stackPtr++; cur = c1; continue; }
          }
          assert(stackPtr < stackEnd);
          *stackPtr = c0; stackPtr++;
          assert(stackPtr < stackEnd);
          *stackPtr = c1; stackPtr++;
          
          /*! three children are hit */
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch();
          assert(cur != BVH4::emptyNode);
          if (likely(mask == 0)) continue;
          assert(stackPtr < stackEnd);
          *stackPtr = cur; stackPtr++;
          
          /*! four children are hit */
          cur = node->child(3); cur.prefetch();
          assert(cur != BVH4::emptyNode);
        }
        
        /*! this is a leaf node */
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        if (PrimitiveIntersector::occluded(pre,ray,prim,num,bvh->geometry)) {
          ray.geomID = 0;
          break;
        }
      }
      AVX_ZERO_UPPER();
    }

#define COMMA ,
    DEFINE_INTERSECTOR1(BVH4Bezier1Intersector1,BVH4Intersector1<0x1 COMMA Bezier1Intersector1>);
    DEFINE_INTERSECTOR1(BVH4Bezier1iIntersector1,BVH4Intersector1<0x1 COMMA Bezier1iIntersector1>);
    DEFINE_INTERSECTOR1(BVH4Triangle1Intersector1Moeller,BVH4Intersector1<0x1 COMMA Triangle1Intersector1MoellerTrumbore>);
    DEFINE_INTERSECTOR1(BVH4Triangle4Intersector1Moeller,BVH4Intersector1<0x1 COMMA Triangle4Intersector1MoellerTrumbore>);
#if defined(__AVX__)
    DEFINE_INTERSECTOR1(BVH4Triangle8Intersector1Moeller,BVH4Intersector1<0x1 COMMA Triangle8Intersector1MoellerTrumbore>);
#endif
    DEFINE_INTERSECTOR1(BVH4Triangle1vIntersector1Pluecker,BVH4Intersector1<0x1 COMMA Triangle1vIntersector1Pluecker>);
    DEFINE_INTERSECTOR1(BVH4Triangle4vIntersector1Pluecker,BVH4Intersector1<0x1 COMMA Triangle4vIntersector1Pluecker>);
    DEFINE_INTERSECTOR1(BVH4Triangle4iIntersector1Pluecker,BVH4Intersector1<0x1 COMMA Triangle4iIntersector1Pluecker>);
    DEFINE_INTERSECTOR1(BVH4VirtualIntersector1,BVH4Intersector1<0x1 COMMA VirtualAccelIntersector1>);
  }
}
