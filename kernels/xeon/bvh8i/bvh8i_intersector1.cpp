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

#include "bvh8i_intersector1.h"
#include "../common/stack_item.h"
#include "bvh4i/bvh4i_builder_util.h"

#include "bvh8i.h"

#if defined(__AVX__)
#include "geometry/triangle8_intersector1_moeller.h"
#endif

namespace embree
{
  namespace isa
  {
// #if defined(__AVX2__)
// #if defined(__X86_64__)
//     __forceinline size_t bitscan(size_t mask) { return _tzcnt_u64(mask); }
// #else
//     __forceinline size_t bitscan(size_t mask) { return _tzcnt_u32(mask); }
// #endif
// #else
//     __forceinline size_t bitscan(size_t mask) { return __bsf(mask); }
// #endif
    
#define DBG(x) 
    
    template<typename TriangleIntersector>
    void BVH8iIntersector1<TriangleIntersector>::intersect(const BVH8i* bvh, Ray& ray)
    {
#if defined(__AVX__)
      /*! stack state */
      StackItem stack[1+3*BVH4i::maxDepth];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      stack[0].ptr  = bvh->root;
      stack[0].dist = neg_inf;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(avxf) : 1*sizeof(avxf);
      const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(avxf) : 3*sizeof(avxf);
      const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(avxf) : 5*sizeof(avxf);
      
      /*! load the ray into SIMD registers */
      const avx3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const avx3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const avx3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const avxf  rayNear(ray.tnear);
      avxf rayFar(ray.tfar);
      
      const void* nodePtr = bvh->nodePtr();
      const void* triPtr  = bvh->triPtr();

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(stackPtr->dist > ray.tfar))
          continue;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = (BVH8i::Node*)cur.node(nodePtr);
          const size_t farX  = nearX ^ sizeof(avxf), farY  = nearY ^ sizeof(avxf), farZ  = nearZ ^ sizeof(avxf);
          const avxf tNearX = msub(load8f((const char*)nodePtr+(size_t)cur+nearX), rdir.x, org_rdir.x);
          const avxf tNearY = msub(load8f((const char*)nodePtr+(size_t)cur+nearY), rdir.y, org_rdir.y);
          const avxf tNearZ = msub(load8f((const char*)nodePtr+(size_t)cur+nearZ), rdir.z, org_rdir.z);
          const avxf tFarX  = msub(load8f((const char*)nodePtr+(size_t)cur+farX ), rdir.x, org_rdir.x);
          const avxf tFarY  = msub(load8f((const char*)nodePtr+(size_t)cur+farY ), rdir.y, org_rdir.y);
          const avxf tFarZ  = msub(load8f((const char*)nodePtr+(size_t)cur+farZ ), rdir.z, org_rdir.z);
          
#if defined(__AVX2__)
          const avxf tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const avxf tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const avxb vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xf;
#else
          const avxf tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const avxf tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          const avxb vmask = tNear <= tFar;
          size_t mask = movemask(vmask);
#endif
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); const float d0 = tNear[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); const float d1 = tNear[r];
          if (likely(mask == 0)) {
            if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
            else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
          stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          r = __bscf(mask);
          NodeRef c = node->child(r); float d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            continue;
          }
          
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          r = __bscf(mask);
          c = node->child(r); d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  if (likely(mask == 0)) {
	    sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
	    cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
	    continue;
	  }
	  
	  while(1)
	    {
	      r = __bscf(mask);
	      c = node->child(r); d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	      if (unlikely(mask == 0)) break;
	    }
	    cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Triangle* tri = (Triangle*) cur.leaf(triPtr,num);
        TriangleIntersector::intersect(ray,tri,num,bvh->geometry);
        rayFar = ray.tfar;
      }
#endif
      AVX_ZERO_UPPER();
     
      // return;
      // const avxb init_mask(1,0,0,0,1,0,0,0); // = r_movemask(0x11);
      
      // DBG(PING);
      // struct StackEntry {
      //   float dist;
      //   unsigned int node;
      // };
      
      // __align(64) StackEntry stack[BVH8_MAX_STACK_DEPTH];
      
      // const BVH8i::BVH8iNode     *const __restrict__ bvh8  = (BVH8i::BVH8iNode*)bvh->qbvh;
      // const Triangle1 *const __restrict__ accel = (Triangle1*)bvh->accel;
      
      // stack[0].node = QBVH_TERMINAL_TOKEN;
      // stack[0].dist = pos_inf;
      // stack[1].node = bvh8[0].min_d[0];
      // stack[1].dist = ray.tnear;
      
      // const Vec3fa rdir = rcp_safe(ray.dir);
      // const Vec3fa org_rdir = ray.org * rdir;
      
      // const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(avxf) : 1*sizeof(avxf);
      // const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(avxf) : 3*sizeof(avxf);
      // const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(avxf) : 5*sizeof(avxf);
      
      // /* --  dummy node at index 0, removes branch in inner-loop -- */
      
      // const avxf org_aos((ssef)ray.org);
      // const avxf dir_aos((ssef)ray.dir);
      
      // const avxf rdir_x(rdir[0]);
      // const avxf rdir_y(rdir[1]);
      // const avxf rdir_z(rdir[2]);
      // const avxf org_rdir_x(org_rdir[0]);
      // const avxf org_rdir_y(org_rdir[1]);
      // const avxf org_rdir_z(org_rdir[2]);
      
      // size_t sindex = 2;
      
      // while(1)
      // {
      // 	sindex--;
      // 	unsigned int curNode = stack[sindex].node;
	
      // 	while(1)
      //   {
      //     DBG(DBG_PRINT(curNode));
          
      //     if (unlikely(qbvhLeaf(curNode) != 0)) break;
      //     STAT3(normal.trav_nodes,1,1,1);
          
      //     const BVH8i::BVH8iNode* __restrict__ const bptr = BVH8i::bvh8ChildPtrNoMask(bvh8,curNode);
          
      //     const avxf tNearX = load8f((float*)((const char*)bptr + (size_t)nearX)) * rdir_x - org_rdir_x;
      //     const avxf tNearY = load8f((float*)((const char*)bptr + (size_t)nearY)) * rdir_y - org_rdir_y;
      //     const avxf tNearZ = load8f((float*)((const char*)bptr + (size_t)nearZ)) * rdir_z - org_rdir_z;
          
      //     const avxf tFarX = load8f((float*)((const char*)bptr + ((size_t)nearX ^ sizeof(avxf)))) * rdir_x - org_rdir_x;
      //     const avxf tFarY = load8f((float*)((const char*)bptr + ((size_t)nearY ^ sizeof(avxf)))) * rdir_y - org_rdir_y;
      //     const avxf tFarZ = load8f((float*)((const char*)bptr + ((size_t)nearZ ^ sizeof(avxf)))) * rdir_z - org_rdir_z;
          
      //     const avxi near8 = max(max(cast(tNearX),cast(tNearY)),max(cast(tNearZ),cast(avxf(ray.tnear))));
      //     const avxi far8  = min(min(cast(tFarX) ,cast(tFarY )),min(cast(tFarZ ),cast(avxf(ray.tfar))));
          
      //     curNode = stack[sindex-1].node;
          
      //     sindex--;
          
      //     const avxb near_far = near8 > far8;
          
      //     DBG(DBG_PRINT(near_far));
          
      //     const unsigned int m_near_far = movemask(near_far) ^ 0xff;
      //     if (likely((m_near_far) == 0)) continue;
          
      //     size_t i_lr_hit = m_near_far;
      //     sindex++;
      //     const size_t pos_first = bitscan(i_lr_hit);
          
      //     curNode = bptr->min_d[pos_first];
          
      //     i_lr_hit &= i_lr_hit-1;
      //     if (likely(i_lr_hit == 0)) continue;
      //     unsigned int curNode_dist = ((unsigned int*)&near8)[pos_first];
	  
          
      //     while(1)
      //     {
      //       const size_t posN = bitscan(i_lr_hit);
            
      //       const unsigned int distN   = ((unsigned int*)&near8)[posN];
      //       const unsigned int nodeN   = bptr->min_d[posN];		  
            
      //       if (curNode_dist <= distN)
      //       {
      //         size_t start = sindex-1;
      //         while(1)
      //         {
      //           if (distN <= *(unsigned int*)&stack[start].dist)
      //           {
      //             *(unsigned int*)&stack[start+1].dist = distN;
      //             stack[start+1].node = nodeN;
      //             break;
      //           }
      //           else
      //           {
      //             *(long*)&stack[start+1] = *(long*)&stack[start];
      //           }
      //           assert(sindex > 0);
      //           start--;
      //         }
      //       }
      //       else
      //       {
      //         size_t start = sindex-1;
      //         while(1)
      //         {
      //           if (curNode_dist <= *(unsigned int*)&stack[start].dist)
      //           {
      //             *(unsigned int*)&stack[start+1].dist = curNode_dist;
      //             stack[start+1].node = curNode;
      //             break;
      //           }
      //           else
      //           {
      //             *(long*)&stack[start+1] = *(long*)&stack[start];
                  
      //           }
      //           assert(sindex > 0);
      //           start--;
      //         }
      //         curNode_dist = distN;
      //         curNode = nodeN;
      //       }
            
      //       sindex++;
            
            
      //       assert(sindex < BVH8_MAX_STACK_DEPTH);
            
      //       i_lr_hit &= i_lr_hit-1;
      //       if (likely(i_lr_hit == 0)) break;
      //     }
          
          
      //   }
        
      // 	if (unlikely(curNode == QBVH_TERMINAL_TOKEN)) { break; } // === stack empty ===
        
      // 	STAT3(normal.trav_leaves,1,1,1);
        
      // 	const size_t itemOffset    = qbvhItemOffset(curNode);
      // 	const Triangle1  *__restrict__ tptr    = (Triangle1*)((char*)accel + itemOffset);
      // 	const size_t items = qbvhItems(curNode);
        
      // 	assert(items > 0);
      // 	assert(items <= 4);
        
      // 	DBG(DBG_PRINT(itemOffset));
      // 	DBG(DBG_PRINT(items));
      // 	DBG(DBG_PRINT(org_aos));
      // 	DBG(DBG_PRINT(dir_aos));
        
      // 	const avxf zero = 0.0f;
      // 	size_t hit = 0;
      // 	for (size_t i=0;i<items;i+=2)
      //   {
      //     STAT3(normal.trav_prims,1,1,1);
          
      //     DBG(DBG_PRINT(tptr[i+0]));
      //     DBG(DBG_PRINT(tptr[i+1]));
          
      //     const avxf v0 = select(0xf,broadcast4f((__m128*)&tptr[i+0].v0),broadcast4f((__m128*)&tptr[i+1].v0));
      //     //const avxf v0 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v0),*(__m128*)&tptr[i+1].v0);
      //     const avxf v1 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v1),*(__m128*)&tptr[i+1].v1);
      //     const avxf v2 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v2),*(__m128*)&tptr[i+1].v2);
      //     const avxf Ng = insert<1>(broadcast4f((__m128*)&tptr[i+0].Ng),*(__m128*)&tptr[i+1].Ng);
          
      //     const avxf org = v0 - org_aos;
      //     const avxf R = cross(org,dir_aos);
          
      //     const avxf e1 = v1 - v0;
      //     const avxf e2 = v0 - v2;
          
      //     const avxf det = dot(dir_aos,Ng);
          
      //     const avxf sgnDet = signmsk(det);
      //     const avxf absDet = abs(det);
          
      //     const avxf U = dot(R,e2)^sgnDet;
      //     const avxf V = dot(R,e1)^sgnDet;
          
      //     const avxb m_aperture = (U >= 0.0f) & (V >= 0.0f) & (U+V <= absDet);
          
      //     if (unlikely(none(m_aperture))) continue;
          
      //     const avxf rcpDet = rcp(det);
      //     const avxf T = dot(org,Ng) * rcpDet;
          
      //     const avxb m_final = m_aperture & (T >= avxf(ray.tnear)) & (T < avxf(ray.tfar));
          
      //     if (unlikely(none(m_final))) continue;
          
      //     STAT3(normal.trav_prim_hits,1,1,1);
          
      //     const avxf t = select(m_final & init_mask,T /* rcp(absDet) */,inf);
	  
      //     const float t0 = _mm_cvtss_f32(extract<0>(t));
      //     const float t1 = _mm_cvtss_f32(extract<1>(t));
          
      //     DBG(DBG_PRINT(t0));
      //     DBG(DBG_PRINT(t1));
          
      //     //const avxf u = min(max(U*rcpDet,0.0f),1.0f);
      //     //const avxf v = min(max(V*rcpDet,0.0f),1.0f);
      //     const avxf u = abs(U*rcpDet);
      //     const avxf v = abs(V*rcpDet);
          
      //     hit = 1;
      //     if (t0 < t1) { 
      //       ray.geomID = tptr[i+0].geomID();
      //       ray.primID = tptr[i+0].primID();
      //       ray.tfar = fextract<0>(t);
      //       ray.u = fextract<0>(u);
      //       ray.v = fextract<0>(v);
      //       ray.Ng[0] = tptr[i+0].Ng.x; // fextract04<0>(Ng);
      //       ray.Ng[1] = tptr[i+0].Ng.y; //fextract04<0>(shuffle<1,1,1,1>(Ng));
      //       ray.Ng[2] = tptr[i+0].Ng.z; //fextract04<0>(shuffle<2,2,2,2>(Ng));
      //     } else {
      //       ray.geomID = tptr[i+1].geomID();
      //       ray.primID = tptr[i+1].primID();
      //       ray.tfar = fextract<1>(t);
      //       ray.u = fextract<1>(u);
      //       ray.v = fextract<1>(v);
      //       ray.Ng[0] = tptr[i+1].Ng.x; //fextract04<1>(Ng);
      //       ray.Ng[1] = tptr[i+1].Ng.y; //fextract04<1>(shuffle<1,1,1,1>(Ng));
      //       ray.Ng[2] = tptr[i+1].Ng.z; //fextract04<1>(shuffle<2,2,2,2>(Ng));
            
      //     };
      //   }
        
      // 	if (unlikely(hit != 0))
      //   {
      //     //const float max_dist = ray.tfar;
      //     const unsigned int max_dist = *(unsigned int*)&ray.tfar;
      //     size_t new_sindex = 1;
      //     for (size_t i=1;i!=sindex;i++)
      //       if (*(unsigned int*)&stack[i].dist <= max_dist) 			
      //         *(size_t*)&stack[new_sindex++] = *(size_t*)&stack[i];
      //     sindex = new_sindex;
      //   }
      // }
      
      // DBG(DBG_PRINT(ray));
      // DBG(exit(0));    
    }

    template<typename TriangleIntersector>    
    void BVH8iIntersector1<TriangleIntersector>::occluded(const BVH8i* bvh, Ray& ray)
    {
//       DBG(PING);
//       struct StackEntry {
//         unsigned int node;
//       };
      
//       __align(64) StackEntry stack[BVH8_MAX_STACK_DEPTH];
      
//       const BVH8i::BVH8iNode     *const __restrict__ bvh8  = (BVH8i::BVH8iNode*)bvh->qbvh;
//       const Triangle1 *const __restrict__ accel = (Triangle1*)bvh->accel;
      
//       stack[0].node = QBVH_TERMINAL_TOKEN;
//       stack[1].node = bvh8[0].min_d[0];
      
//       const Vec3fa rdir = rcp_safe(ray.dir);
//       const Vec3fa org_rdir = ray.org * rdir;
      
//       const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(avxf) : 1*sizeof(avxf);
//       const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(avxf) : 3*sizeof(avxf);
//       const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(avxf) : 5*sizeof(avxf);
      
//       /* --  dummy node at index 0, removes branch in inner-loop -- */
      
//       const avxf org_aos((ssef)ray.org);
//       const avxf dir_aos((ssef)ray.dir);
      
//       const avxf rdir_x(rdir[0]);
//       const avxf rdir_y(rdir[1]);
//       const avxf rdir_z(rdir[2]);
//       const avxf org_rdir_x(org_rdir[0]);
//       const avxf org_rdir_y(org_rdir[1]);
//       const avxf org_rdir_z(org_rdir[2]);
      
//       size_t sindex = 2;
      
//       while(1)
//       {
// 	sindex--;
// 	unsigned int curNode = stack[sindex].node;
	
// 	while(1)
//         {
//           DBG(DBG_PRINT(curNode));
          
//           if (unlikely(qbvhLeaf(curNode) != 0)) break;
//           STAT3(shadow.trav_nodes,1,1,1);
          
//           const BVH8i::BVH8iNode* __restrict__ const bptr = BVH8i::bvh8ChildPtrNoMask(bvh8,curNode);
          
//           const avxf tNearX = load8f((float*)((const char*)bptr + (size_t)nearX)) * rdir_x - org_rdir_x;
//           const avxf tNearY = load8f((float*)((const char*)bptr + (size_t)nearY)) * rdir_y - org_rdir_y;
//           const avxf tNearZ = load8f((float*)((const char*)bptr + (size_t)nearZ)) * rdir_z - org_rdir_z;
          
//           const avxf tFarX = load8f((float*)((const char*)bptr + ((size_t)nearX ^ sizeof(avxf)))) * rdir_x - org_rdir_x;
//           const avxf tFarY = load8f((float*)((const char*)bptr + ((size_t)nearY ^ sizeof(avxf)))) * rdir_y - org_rdir_y;
//           const avxf tFarZ = load8f((float*)((const char*)bptr + ((size_t)nearZ ^ sizeof(avxf)))) * rdir_z - org_rdir_z;
          
// #if 0
//           const avxf near8 = max(max(tNearX,tNearY),max(tNearZ,ray.tnear));
//           const avxf far8  = min(min(tFarX ,tFarY ),min( tFarZ,ray.tfar));
// #else
//           const avxi near8 = max(max(cast(tNearX),cast(tNearY)),max(cast(tNearZ),cast(avxf(ray.tnear))));
//           const avxi far8  = min(min(cast(tFarX) ,cast(tFarY )),min(cast(tFarZ ),cast(avxf(ray.tfar))));
          
// #endif
          
//           curNode = stack[sindex-1].node;
          
//           sindex--;
          
//           const avxb near_far = near8 > far8;
          
//           DBG(DBG_PRINT(near_far));
          
//           const unsigned int m_near_far = movemask(near_far) ^ 0xff;
//           if (likely((m_near_far) == 0)) continue;
          
//           size_t i_lr_hit = m_near_far;
//           const size_t num_m_lr_hit = __popcnt(i_lr_hit); 
//           sindex++;
//           const size_t pos_first = bitscan(i_lr_hit);
//           curNode = bptr->min_d[pos_first];
          
//           if (likely(num_m_lr_hit == 1)) continue;
//           unsigned int curNode_dist = ((unsigned int*)&near8)[pos_first];
	  
//           while(1)
//           {
            
//             const size_t posN = bitscan(i_lr_hit);
//             DBG(DBG_PRINT(posN));
            
            
//             const unsigned int distN   = ((unsigned int*)&near8)[posN];
//             const unsigned int nodeN   = bptr->min_d[posN];		  
            
//             if (curNode_dist <= distN)
//             {
//               stack[sindex].node = nodeN;
//             }
//             else
//             {
//               stack[sindex].node = curNode;
//               curNode_dist = distN;
//               curNode = nodeN;
//             }
            
//             sindex++;
//             assert(sindex < BVH8_MAX_STACK_DEPTH);
//             i_lr_hit &= i_lr_hit-1;
//             if (likely(i_lr_hit == 0)) break;
            
//           }
//         }
        
// 	if (unlikely(curNode == QBVH_TERMINAL_TOKEN)) { break; } // === stack empty ===
        
// 	STAT3(shadow.trav_leaves,1,1,1);
        
// 	const size_t itemOffset    = qbvhItemOffset(curNode);
// 	const Triangle1  *__restrict__ tptr    = (Triangle1*)((char*)accel + itemOffset);
// 	const size_t items = qbvhItems(curNode);
        
// 	assert(items > 0);
// 	assert(items <= 4);
        
// 	DBG(DBG_PRINT(itemOffset));
// 	DBG(DBG_PRINT(items));
// 	DBG(DBG_PRINT(org_aos));
// 	DBG(DBG_PRINT(dir_aos));
        
// 	const avxf zero = 0.0f;
// 	size_t hit = 0;
// 	for (size_t i=0;i<items;i+=2)
//         {
//           STAT3(shadow.trav_prims,1,1,1);
          
//           DBG(DBG_PRINT(tptr[i+0]));
//           DBG(DBG_PRINT(tptr[i+1]));
          
//           const avxf v0 = select(0xf,broadcast4f((__m128*)&tptr[i+0].v0),broadcast4f((__m128*)&tptr[i+1].v0));
//           //const avxf v0 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v0),*(__m128*)&tptr[i+1].v0);
//           const avxf v1 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v1),*(__m128*)&tptr[i+1].v1);
//           const avxf v2 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v2),*(__m128*)&tptr[i+1].v2);
//           const avxf Ng = insert<1>(broadcast4f((__m128*)&tptr[i+0].Ng),*(__m128*)&tptr[i+1].Ng);
          
//           const avxf org = v0 - org_aos;
//           const avxf R = cross(org,dir_aos);
          
//           const avxf e1 = v1 - v0;
//           const avxf e2 = v0 - v2;
          
//           const avxf det = dot(dir_aos,Ng);
          
//           const avxf sgnDet = signmsk(det);
//           const avxf absDet = abs(det);
          
//           const avxf U = dot(R,e2)^sgnDet;
//           const avxf V = dot(R,e1)^sgnDet;
          
//           const avxb m_aperture = (U >= 0.0f) & (V >= 0.0f) & (U+V <= absDet);
          
//           if (unlikely(none(m_aperture))) continue;
          
//           const avxf rcpDet = rcp(det);
//           const avxf T = dot(org,Ng) * rcpDet;
          
//           const avxb m_final = m_aperture & (T >= avxf(ray.tnear)) & (T < avxf(ray.tfar));
          
//           if (unlikely(none(m_final))) continue;
          
//           ray.geomID = 0;
//           AVX_ZERO_UPPER();
//           return;
//         }
//       }
      
//       DBG(DBG_PRINT(ray));
//       DBG(exit(0));    
      AVX_ZERO_UPPER();
    }

#if defined(__AVX__)    
    DEFINE_INTERSECTOR1(BVH8iTriangle8Intersector1Moeller,BVH8iIntersector1<Triangle8Intersector1MoellerTrumbore>);
#endif
  }
}
