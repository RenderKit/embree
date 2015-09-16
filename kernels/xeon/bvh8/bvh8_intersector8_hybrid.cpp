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

#include "bvh8_intersector8_hybrid.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle8v.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
//#include "../geometry/triangle_intersector_pluecker2.h"

#define DBG(x) 

#define SWITCH_THRESHOLD 7
//#define SWITCH_THRESHOLD 16
#define SWITCH_DURING_DOWN_TRAVERSAL 1


namespace embree
{
  namespace isa
  { 

    template<typename PrimitiveIntersector8>
    __forceinline void BVH8Intersector8Hybrid<PrimitiveIntersector8>::intersect1(const BVH8* bvh, NodeRef root, const size_t k, Precalculations& pre, Ray8& ray,const Vec3f8 &ray_org, const Vec3f8 &ray_dir, const Vec3f8 &ray_rdir, const float8 &ray_tnear, const float8 &ray_tfar, const Vec3i8& nearXYZ)
    {
      /*! stack state */
      StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
      StackItemT<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemT<NodeRef>* stackEnd = stack+stackSizeSingle;
      stack[0].ptr = root;
      stack[0].dist = neg_inf;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = nearXYZ.x[k];
      const size_t nearY = nearXYZ.y[k];
      const size_t nearZ = nearXYZ.z[k];

      /*! load the ray into SIMD registers */
      const Vec3f8 org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const Vec3f8 rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const Vec3f8 norg = -org, org_rdir(org*rdir);
      float8 rayNear(ray_tnear[k]), rayFar(ray_tfar[k]);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        STAT3(normal.trav_stack_pop,1,1,1);

        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
          continue;


        /* downtraversal loop */
        while (true)
        {

          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();
          const size_t farX  = nearX ^ sizeof(float8), farY  = nearY ^ sizeof(float8), farZ  = nearZ ^ sizeof(float8);
#if defined (__AVX2__)
          const float8 tNearX = msub(load8f((const char*)node+nearX), rdir.x, org_rdir.x);
          const float8 tNearY = msub(load8f((const char*)node+nearY), rdir.y, org_rdir.y);
          const float8 tNearZ = msub(load8f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const float8 tFarX  = msub(load8f((const char*)node+farX ), rdir.x, org_rdir.x);
          const float8 tFarY  = msub(load8f((const char*)node+farY ), rdir.y, org_rdir.y);
          const float8 tFarZ  = msub(load8f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
          const float8 tNearX = (norg.x + load8f((const char*)node+nearX)) * rdir.x;
          const float8 tNearY = (norg.y + load8f((const char*)node+nearY)) * rdir.y;
          const float8 tNearZ = (norg.z + load8f((const char*)node+nearZ)) * rdir.z;
          const float8 tFarX  = (norg.x + load8f((const char*)node+farX )) * rdir.x;
          const float8 tFarY  = (norg.y + load8f((const char*)node+farY )) * rdir.y;
          const float8 tFarZ  = (norg.z + load8f((const char*)node+farZ )) * rdir.z;
#endif

#if defined(__AVX2__)
          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const bool8 vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;
#else
          const float8 tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const float8 tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          const bool8 vmask = tNear <= tFar;
          size_t mask = movemask(vmask);
#endif

          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); cur.prefetch();
            assert(cur != BVH8::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH8::emptyNode);
          assert(c1 != BVH8::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd); 
            if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
            else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
          }

          /*! use 8-wide sorting network in the AVX2 */
#if defined(__AVX2__) && 0

          const bool8 mask8 = !vmask;
          const size_t hits = __popcnt(movemask(mask8));
          const int8 tNear_i = cast(tNear);
          const int8 dist    = select(mask8,(tNear_i & (~7)) | int8(step),int8( True ));
          const int8 order   = sortNetwork(dist) & 7;
          const unsigned int cur_index = extract<0>(extract<0>(order));
          cur = node->child(cur_index);
          cur.prefetch();

          for (size_t i=0;i<hits-1;i++) 
          {
            r = order[hits-1-i];
            assert( ((unsigned int)1 << r) & movemask(mask8));
            const NodeRef c = node->child(r); 
            assert(c != BVH8::emptyNode);
            c.prefetch(); 
            const unsigned int d = *(unsigned int*)&tNear[r]; 
            stackPtr->ptr = c; 
            stackPtr->dist = d; 
            stackPtr++;            
          }
#else

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
          assert(c != BVH8::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            continue;
          }
        
	  /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  if (likely(mask == 0)) {
	    sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
	    cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
	    continue;
	  }

	  /*! fallback case if more than 4 children are hit */
	  while (1)
	  {
	    r = __bscf(mask);
	    assert(stackPtr < stackEnd);
	    const NodeRef c = node->child(r); c.prefetch(); const unsigned int d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
	  cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
#endif
	}
        
        /*! this is a leaf node */
	assert(cur != BVH8::emptyNode);
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Triangle* prim = (Triangle*) cur.leaf(num);

        size_t lazy_node = 0;
        PrimitiveIntersector8::intersect(pre,ray,k,prim,num,bvh->scene,lazy_node);
        rayFar = ray.tfar[k];

        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = neg_inf;
          stackPtr++;
        }
      }
    }
   
    
    template<typename PrimitiveIntersector8>    
    void BVH8Intersector8Hybrid<PrimitiveIntersector8>::intersect(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
      /* load ray */
      bool8 valid0 = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      Vec3f8 ray_org = ray.org;
      Vec3f8 ray_dir = ray.dir;
      float8 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;

      const Vec3f8 rdir = rcp_safe(ray_dir);
      const Vec3f8 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float8)),int8(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float8)),int8(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float8)),int8(5*(int)sizeof(float8)));

      /* allocate stack and push root node */
      float8    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float8*    __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* cull node if behind closest hit point */
        float8 curDist = *sptr_near;
        const bool8 active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;

        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            intersect1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ);
          }
          ray_tfar = ray.tfar;
          continue;
        }
#endif
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(cur.isLeaf()))
            break;
          
          const bool8 valid_node = ray_tfar > curDist;
          STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
          const Node* __restrict__ const node = (Node*)cur.node();
          
          /* pop of next node */
          assert(sptr_node > stack_node);
          sptr_node--;
          sptr_near--;
          cur = *sptr_node;
          curDist = *sptr_near;
          
          for (unsigned i=0; i<BVH8::N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH8::emptyNode)) break;
            
#if defined(__AVX2__)
            const float8 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const float8 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const float8 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const float8 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const float8 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const float8 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const float8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
            const float8 lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
            const float8 lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
            const float8 lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
            const float8 lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
            const float8 lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
            const float8 lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
            const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              const float8 childDist = select(lhit,lnearP,inf);
              const NodeRef child = node->children[i];
              assert(child != BVH8::emptyNode);
              
              /* push cur node onto stack and continue with hit child */
              if (any(childDist < curDist))
              {
                *sptr_node = cur;
                *sptr_near = curDist; 
                curDist = childDist;
                cur = child;
		sptr_node++;
		sptr_near++;
              }              
              /* push hit child onto stack */
              else {
                *sptr_node = child;
                *sptr_near = childDist;               
		sptr_node++;
		sptr_near++;

              }
            }	      
          }

#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
        }
        
        /* return if stack is empty */
        if (unlikely(cur == BVH8::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
	assert(cur != BVH8::emptyNode);
        const bool8 valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Triangle* prim = (Triangle*) cur.leaf(items);
        
        size_t lazy_node = 0;
        PrimitiveIntersector8::intersect(valid_leaf,pre,ray,prim,items,bvh->scene,lazy_node);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      AVX_ZERO_UPPER();
    }
    


    template<typename PrimitiveIntersector8>
    __forceinline bool BVH8Intersector8Hybrid<PrimitiveIntersector8>::occluded1(const BVH8* bvh, NodeRef root, const size_t k, Precalculations& pre, Ray8& ray,const Vec3f8 &ray_org, const Vec3f8 &ray_dir, const Vec3f8 &ray_rdir, const float8 &ray_tnear, const float8 &ray_tfar, const Vec3i8& nearXYZ)
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
      const Vec3f8 org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const Vec3f8 rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const Vec3f8 norg = -org, org_rdir(org*rdir);
      const float8 rayNear(ray_tnear[k]), rayFar(ray_tfar[k]); 

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        STAT3(shadow.trav_stack_pop,1,1,1);

        stackPtr--;
        NodeRef cur = (NodeRef) *stackPtr;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(shadow.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();
          const size_t farX  = nearX ^ sizeof(float8), farY  = nearY ^ sizeof(float8), farZ  = nearZ ^ sizeof(float8);
#if defined (__AVX2__)
          const float8 tNearX = msub(load8f((const char*)node+nearX), rdir.x, org_rdir.x);
          const float8 tNearY = msub(load8f((const char*)node+nearY), rdir.y, org_rdir.y);
          const float8 tNearZ = msub(load8f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const float8 tFarX  = msub(load8f((const char*)node+farX ), rdir.x, org_rdir.x);
          const float8 tFarY  = msub(load8f((const char*)node+farY ), rdir.y, org_rdir.y);
          const float8 tFarZ  = msub(load8f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
          const float8 tNearX = (norg.x + load8f((const char*)node+nearX)) * rdir.x;
          const float8 tNearY = (norg.y + load8f((const char*)node+nearY)) * rdir.y;
          const float8 tNearZ = (norg.z + load8f((const char*)node+nearZ)) * rdir.z;
          const float8 tFarX  = (norg.x + load8f((const char*)node+farX )) * rdir.x;
          const float8 tFarY  = (norg.y + load8f((const char*)node+farY )) * rdir.y;
          const float8 tFarZ  = (norg.z + load8f((const char*)node+farZ )) * rdir.z;
#endif
          
#if defined(__AVX2__)
          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const bool8 vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;
#else
          const float8 tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const float8 tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          const bool8 vmask = tNear <= tFar;
          size_t mask = movemask(vmask);
#endif
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); cur.prefetch(); 
            assert(cur != BVH8::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH8::emptyNode);
          assert(c1 != BVH8::emptyNode);
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
          cur = node->child(r); cur.prefetch(); *stackPtr = cur; stackPtr++;
          if (likely(mask == 0)) {
            stackPtr--;
            continue;
          }

	  /*! process more than three children */
	  while(1)
	  {
	    r = __bscf(mask);
	    NodeRef c = node->child(r); c.prefetch(); *stackPtr = c; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
	  cur = (NodeRef) stackPtr[-1]; stackPtr--;
        }
        
        /*! this is a leaf node */
	assert(cur != BVH8::emptyNode);
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Triangle* prim = (Triangle*) cur.leaf(num);

        size_t lazy_node = 0;
        if (PrimitiveIntersector8::occluded(pre,ray,k,prim,num,bvh->scene,lazy_node)) {
          //ray.geomID = 0;
          //break;
	  return true;
        }

        if (unlikely(lazy_node)) {
          *stackPtr = lazy_node;
          stackPtr++;
        }
      }
      return false;
    }

     template<typename PrimitiveIntersector8>
    void BVH8Intersector8Hybrid<PrimitiveIntersector8>::occluded(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
      /* load ray */
      bool8 valid = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
      bool8 terminated = !valid;
      Vec3f8 ray_org = ray.org, ray_dir = ray.dir;
      float8 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f8 rdir = rcp_safe(ray_dir);
      const Vec3f8 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float8)),int8(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float8)),int8(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float8)),int8(5*(int)sizeof(float8)));

      /* allocate stack and push root node */
      float8    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float8*    __restrict__ sptr_near = stack_near + 2;

      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        /* cull node if behind closest hit point */
        float8 curDist = *sptr_near;
        const bool8 active = curDist < ray_tfar;
        if (unlikely(none(active))) 
          continue;
        
        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            if (occluded1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
              terminated[i] = -1;
          }
          if (all(terminated)) break;
          ray_tfar = select(terminated,float8(neg_inf),ray_tfar);
          continue;
        }
#endif

        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(cur.isLeaf()))
            break;
          
          const bool8 valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),8);
          const Node* __restrict__ const node = (Node*)cur.node();
          
          /* pop of next node */
          assert(sptr_node > stack_node);
          sptr_node--;
          sptr_near--;
          cur = *sptr_node;
          curDist = *sptr_near;
          
          for (unsigned i=0; i<BVH8::N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH8::emptyNode)) break;
            
#if defined(__AVX2__)
            const float8 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const float8 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const float8 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const float8 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const float8 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const float8 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const float8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
            const float8 lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
            const float8 lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
            const float8 lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
            const float8 lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
            const float8 lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
            const float8 lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
            const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              assert(child != BVH8::emptyNode);
              const float8 childDist = select(lhit,lnearP,inf);
              sptr_node++;
              sptr_near++;
              
              /* push cur node onto stack and continue with hit child */
              if (any(childDist < curDist))
              {
                *(sptr_node-1) = cur;
                *(sptr_near-1) = curDist; 
                curDist = childDist;
                cur = child;
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
            *sptr_node++ = cur;
            *sptr_near++ = curDist;
            goto pop;
          }
#endif
        }
        
        /* return if stack is empty */
        if (unlikely(cur == BVH8::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
	assert(cur != BVH8::emptyNode);
        const bool8 valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Triangle* prim = (Triangle*) cur.leaf(items);

        size_t lazy_node = 0;
        terminated |= PrimitiveIntersector8::occluded(!terminated,pre,ray,prim,items,bvh->scene,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated,float8(neg_inf),ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      store8i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8HybridMoeller,BVH8Intersector8Hybrid<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8HybridMoellerNoFilter,BVH8Intersector8Hybrid<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA false> > >);
    
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8HybridMoeller,BVH8Intersector8Hybrid<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8HybridMoellerNoFilter,BVH8Intersector8Hybrid<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA false> > >);

    //DEFINE_INTERSECTOR8(BVH8Triangle8vIntersector8HybridPluecker, BVH8Intersector8Hybrid<ArrayIntersector8_1<TriangleNvIntersectorMPluecker2<Ray8 COMMA Triangle8v COMMA true> > >);
    //DEFINE_INTERSECTOR8(BVH8Triangle8vIntersector8HybridPlueckerNoFilter, BVH8Intersector8Hybrid<ArrayIntersector8_1<TriangleNvIntersectorMPluecker2<Ray8 COMMA Triangle8v COMMA false> > >);
  }
}  

