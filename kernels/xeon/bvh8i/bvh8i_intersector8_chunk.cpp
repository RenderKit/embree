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

#include "bvh8i_intersector8_chunk.h"
#include "bvh4i/bvh4i_builder_util.h"

#define QBVH_MAX_STACK_DEPTH 64

#if defined(__AVX2__)
#define USE_AVXI_OR_MIN_MAX_CMP
#endif

#define DBG(x) 

#define START_SINDEX 5

namespace embree
{
  namespace isa
  {
    template<class T>
    __forceinline void insertionsort_dec(T *array, const size_t length)
    {
      for(size_t i = 1;i<length;++i)
      {
	T v = array[i];
	size_t j = i;
	while(j > 0 && v > array[j-1])
        {
          array[j] = array[j-1];
          --j;
        }
	array[j] = v;
      }
    }
    
#if 1 //(defined(__TARGET_AVX__) || defined(__TARGET_AVX2__)) && 1
    
    void BVH8iIntersector8Chunk::intersect(avxb* valid_i, BVH8i* bvh, Ray8& ray)
    {
      DBG(PING);
      
      const avxb init_mask(1,0,0,0,1,0,0,0); // = r_movemask(0x11);
      
      struct __align(8) StackEntry {
        unsigned int node;
        float dist;
        
        __forceinline bool operator<(const StackEntry &m) const 
        { 
          return *(unsigned int*)&dist < *(unsigned int*)&m.dist; 
        } 
        
        __forceinline bool operator>(const StackEntry &m) const 
        { 
          return *(unsigned int*)&dist > *(unsigned int*)&m.dist; 
        } 
        
      };
      
      __align(64) StackEntry stack[BVH8_MAX_STACK_DEPTH];
      
      avxb m_active = *valid_i;
      
      const avx3f &origin    = ray.org;
      const avx3f &direction = ray.dir;
      const avxf   max_dist  = ray.tfar;
      
      const BVH8i::BVH8iNode *const __restrict__ bvh8  = (BVH8i::BVH8iNode*)bvh->qbvh;
      const Triangle1    *const __restrict__ accel = (Triangle1*)bvh->accel;
      
      const avxf &origin8x = origin.x;
      const avxf &origin8y = origin.y;
      const avxf &origin8z = origin.z;
      
      const avxf ulp(6E-8f);
      const avxf direction8x = select(direction.x == 0,ulp,direction.x);
      const avxf direction8y = select(direction.y == 0,ulp,direction.y);
      const avxf direction8z = select(direction.z == 0,ulp,direction.z);
      
      const avxi near8X = select(direction8x < 0.0f,avxi(sizeof(avxf)),avxi(0)) + 0*sizeof(avxf);
      const avxi near8Y = select(direction8y < 0.0f,avxi(sizeof(avxf)),avxi(0)) + 2*sizeof(avxf);
      const avxi near8Z = select(direction8z < 0.0f,avxi(sizeof(avxf)),avxi(0)) + 4*sizeof(avxf);
      
      const avxf rdir8x = rcp(direction8x);
      const avxf rdir8y = rcp(direction8y);
      const avxf rdir8z = rcp(direction8z);
      
      const avxf org_rdir8x = origin8x * rdir8x;
      const avxf org_rdir8y = origin8y * rdir8y;
      const avxf org_rdir8z = origin8z * rdir8z;
      
      const avxf inf  = avxf(pos_inf);
      
      /* --  dummy node at index 0, removes branch in inner-loop -- */
      
#pragma unroll
      for (size_t i=0;i<START_SINDEX-1;i++)
      {
	stack[i].node = QBVH_TERMINAL_TOKEN;
	*(unsigned int*)&stack[i].dist = 0xffffffff;
      }
      
      size_t active_mask = movemask(*valid_i != 0);
      
      while(active_mask)
      {
	const size_t rayIndex = bitscan(active_mask); 
	active_mask &= active_mask-1;
        
	stack[START_SINDEX-1].node = bvh8[0].min_d[0];
	stack[START_SINDEX-1].dist = ray.tnear[rayIndex];
        
	const avxf org_aos(origin8x[rayIndex],origin8y[rayIndex],origin8z[rayIndex],0.0f);
	const avxf dir_aos(direction8x[rayIndex],direction8y[rayIndex],direction8z[rayIndex],0.0f);
        
	const avxf rdir_x(rdir8x[rayIndex]);
	const avxf rdir_y(rdir8y[rayIndex]);
	const avxf rdir_z(rdir8z[rayIndex]);
	const avxf org_rdir_x(org_rdir8x[rayIndex]);
	const avxf org_rdir_y(org_rdir8y[rayIndex]);
	const avxf org_rdir_z(org_rdir8z[rayIndex]);
        
	const unsigned int nearX = near8X[rayIndex];
	const unsigned int nearY = near8Y[rayIndex];
	const unsigned int nearZ = near8Z[rayIndex];
        
	size_t sindex = START_SINDEX;
        
	while(1)
        {
          sindex--;
          unsigned int curNode = stack[sindex].node;
	  
          while(1)
          {
            DBG(DBG_PRINT(curNode));
            
            if (unlikely(qbvhLeaf(curNode) != 0)) break;
            STAT3(normal.trav_nodes,1,1,1);
            
            const BVH8i::BVH8iNode* __restrict__ const bptr = BVH8i::bvh8ChildPtrNoMask(bvh8,curNode);
            
            const avxf tNearX = load8f((float*)((const char*)bptr + (size_t)nearX)) * rdir_x - org_rdir_x;
            const avxf tNearY = load8f((float*)((const char*)bptr + (size_t)nearY)) * rdir_y - org_rdir_y;
            const avxf tNearZ = load8f((float*)((const char*)bptr + (size_t)nearZ)) * rdir_z - org_rdir_z;
            
            const avxf tFarX = load8f((float*)((const char*)bptr + ((size_t)nearX ^ sizeof(avxf)))) * rdir_x - org_rdir_x;
            const avxf tFarY = load8f((float*)((const char*)bptr + ((size_t)nearY ^ sizeof(avxf)))) * rdir_y - org_rdir_y;
            const avxf tFarZ = load8f((float*)((const char*)bptr + ((size_t)nearZ ^ sizeof(avxf)))) * rdir_z - org_rdir_z;
            
            // const avxf near8 = max(max(tNearX,tNearY),max(tNearZ,ray.tnear[rayIndex]));
            // const avxf far8  = min(min(tFarX ,tFarY ),min( tFarZ,ray.tfar[rayIndex]));
            
            const avxi near8 = max(max(cast(tNearX),cast(tNearY)),max(cast(tNearZ),cast(avxf(ray.tnear[rayIndex]))));
            const avxi far8  = min(min(cast(tFarX) ,cast(tFarY )),min(cast(tFarZ ),cast(avxf(ray.tfar[rayIndex]))));
            
            curNode = stack[sindex-1].node;
            
            sindex--;
            
            const avxb near_far = near8 > far8;
            
            DBG(DBG_PRINT(near_far));
            
            const unsigned int m_near_far = movemask(near_far) ^ 0xff;
            if (likely((m_near_far) == 0)) continue;
            
            size_t i_lr_hit = m_near_far;
            //const size_t num_m_lr_hit = _popcnt64(i_lr_hit); 
            sindex++;
            const size_t pos_first = bitscan(i_lr_hit);
            curNode = bptr->min_d[pos_first];
            i_lr_hit &= i_lr_hit-1;
            if (likely(i_lr_hit == 0)) continue; // == 1 == 
            
            unsigned int curNode_dist = ((unsigned int*)&near8)[pos_first];	   
#if 1
            
            while(1)
            {
              const size_t posN = bitscan(i_lr_hit);
              
              const unsigned int distN   = ((unsigned int*)&near8)[posN];
              const unsigned int nodeN   = bptr->min_d[posN];		  
              
              if (curNode_dist <= distN)
              {
                size_t start = sindex-1;
                while(1)
                {
                  if (distN <= *(unsigned int*)&stack[start].dist)
                  {
                    *(unsigned int*)&stack[start+1].dist = distN;
                    stack[start+1].node = nodeN;
                    break;
                  }
                  else
                  {
                    *(long*)&stack[start+1] = *(long*)&stack[start];
                  }
                  assert(sindex > 0);
                  start--;
                }
              }
              else
              {
                size_t start = sindex-1;
                while(1)
                {
                  if (curNode_dist <= *(unsigned int*)&stack[start].dist)
                  {
                    *(unsigned int*)&stack[start+1].dist = curNode_dist;
                    stack[start+1].node = curNode;
                    break;
                  }
                  else
                  {
                    *(long*)&stack[start+1] = *(long*)&stack[start];
                    
                  }
                  assert(sindex > 0);
                  start--;
                }
                curNode_dist = distN;
                curNode = nodeN;
              }
              
              sindex++;
              
              
              assert(sindex < QBVH_MAX_STACK_DEPTH);
              
              i_lr_hit &= i_lr_hit-1;
              if (likely(i_lr_hit == 0)) break;
            }
            
#else
            
            while(1)
            {
              
              
              const size_t posN = bitscan(i_lr_hit);
              DBG(DBG_PRINT(posN));
              
              
              const unsigned int distN   = ((unsigned int*)&near8)[posN];
              const unsigned int nodeN   = bptr->min_d[posN];		  
              
              if (curNode_dist <= distN)
              {
                if (distN < *(unsigned int*)&stack[sindex-1].dist)
                {
                  stack[sindex].node = nodeN;
                  *(unsigned int*)&stack[sindex].dist = distN;
                }
                else
                {
                  *(unsigned long*)&stack[sindex] = *(unsigned long*)&stack[sindex-1];
                  stack[sindex-1].node = nodeN;
                  *(unsigned int*)&stack[sindex-1].dist = distN;
                }
              }
              else
              {
                if (curNode_dist < *(unsigned int*)&stack[sindex-1].dist)
                {
                  stack[sindex].node = curNode;
                  *(unsigned int*)&stack[sindex].dist = curNode_dist;
                }
                else
                {
                  *(unsigned long*)&stack[sindex] = *(unsigned long*)&stack[sindex-1];
                  stack[sindex-1].node = curNode;
                  *(unsigned int*)&stack[sindex-1].dist = curNode_dist;
                }
                curNode_dist = distN;
                curNode = nodeN;
              }
              
#if 1
              if (unlikely((*(unsigned int*)&stack[sindex-2].dist) < (*(unsigned int*)&stack[sindex-1].dist)))
              {
                std::swap(*((unsigned long*)&stack[sindex-2]),(*(unsigned long*)&stack[sindex-1]));
              }
#endif
              
              
              sindex++;
              assert(sindex < BVH8_MAX_STACK_DEPTH);
              
              i_lr_hit &= i_lr_hit-1;
              if (likely(i_lr_hit == 0)) break;
              
            }
#endif
          }
          
          if (unlikely(curNode == QBVH_TERMINAL_TOKEN)) { break; } // === stack empty ===
          
          STAT3(normal.trav_leaves,1,1,1);
          
          const size_t itemOffset    = qbvhItemOffset(curNode);
          const Triangle1  *__restrict__ tptr    = (Triangle1*)((char*)accel + itemOffset);
          const size_t items = qbvhItems(curNode);
          
          assert(items > 0);
          assert(items <= 4);
          
          DBG(DBG_PRINT(itemOffset));
          DBG(DBG_PRINT(items));
          DBG(DBG_PRINT(org_aos));
          DBG(DBG_PRINT(dir_aos));
          
          const avxf zero = 0.0f;
          size_t hit = 0;
          for (size_t i=0;i<items;i+=2)
          {
            STAT3(normal.trav_prims,1,1,1);
            
            DBG(DBG_PRINT(tptr[i+0]));
            DBG(DBG_PRINT(tptr[i+1]));
            
            const avxf v0 = select(0xf,broadcast4f((__m128*)&tptr[i+0].v0),broadcast4f((__m128*)&tptr[i+1].v0));
            //const avxf v0 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v0),*(__m128*)&tptr[i+1].v0);
            const avxf v1 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v1),*(__m128*)&tptr[i+1].v1);
            const avxf v2 = insert<1>(broadcast4f((__m128*)&tptr[i+0].v2),*(__m128*)&tptr[i+1].v2);
            const avxf Ng = insert<1>(broadcast4f((__m128*)&tptr[i+0].Ng),*(__m128*)&tptr[i+1].Ng);
            
            const avxf org = v0 - org_aos;
            const avxf R = cross(org,dir_aos);
            
            const avxf e1 = v1 - v0;
            const avxf e2 = v0 - v2;
            
            const avxf det = dot(dir_aos,Ng);
            
            const avxf sgnDet = signmsk(det);
            const avxf absDet = abs(det);
            
            const avxf U = dot(R,e2)^sgnDet;
            const avxf V = dot(R,e1)^sgnDet;
            
            const avxb m_aperture = (U >= 0.0f) & (V >= 0.0f) & (U+V <= absDet);
            
            if (unlikely(none(m_aperture))) continue;
            
            const avxf rcpDet = rcp(det);
            const avxf T = dot(org,Ng) * rcpDet;
            
            const avxb m_final = m_aperture & (T >= avxf(ray.tnear[rayIndex])) & (T < avxf(ray.tfar[rayIndex]));
            const avxf u = abs(U*rcpDet);
            const avxf v = abs(V*rcpDet);
            
            if (unlikely(none(m_final))) continue;
            
            STAT3(normal.trav_prim_hits,1,1,1);
            
            const avxf t = select(m_final ,T /* rcp(absDet) */,inf);
            const float t0 = _mm_cvtss_f32(extract<0>(t));
            const float t1 = _mm_cvtss_f32(extract<1>(t));
            
            DBG(DBG_PRINT(t0));
            DBG(DBG_PRINT(t1));
            
            hit = 1;
            if (t0 < t1) { 
              ray.geomID[rayIndex] = tptr[i+0].geomID();
              ray.primID[rayIndex] = tptr[i+0].primID();
              ray.tfar[rayIndex] = fextract<0>(t);
              ray.u[rayIndex] = fextract<0>(u);
              ray.v[rayIndex] = fextract<0>(v);
              ray.Ng[0][rayIndex] = tptr[i+0].Ng.x; // fextract04<0>(Ng);
              ray.Ng[1][rayIndex] = tptr[i+0].Ng.y; //fextract04<0>(shuffle<1,1,1,1>(Ng));
              ray.Ng[2][rayIndex] = tptr[i+0].Ng.z; //fextract04<0>(shuffle<2,2,2,2>(Ng));
            } else {
              ray.geomID[rayIndex] = tptr[i+1].geomID();
              ray.primID[rayIndex] = tptr[i+1].primID();
              ray.tfar[rayIndex] = fextract<1>(t);
              ray.u[rayIndex] = fextract<1>(u);
              ray.v[rayIndex] = fextract<1>(v);
              ray.Ng[0][rayIndex] = tptr[i+1].Ng.x; //fextract04<1>(Ng);
              ray.Ng[1][rayIndex] = tptr[i+1].Ng.y; //fextract04<1>(shuffle<1,1,1,1>(Ng));
              ray.Ng[2][rayIndex] = tptr[i+1].Ng.z; //fextract04<1>(shuffle<2,2,2,2>(Ng));
              
            };
          }
          
          if (unlikely(hit != 0))
          {
            //const float max_dist = ray.tfar[rayIndex];
            const unsigned int max_dist = *(unsigned int*)&ray.tfar[rayIndex];
            size_t new_sindex = START_SINDEX-1;
            for (size_t i=START_SINDEX-1;i!=sindex;i++)
              if (*(unsigned int*)&stack[i].dist <= max_dist) 			
                *(size_t*)&stack[new_sindex++] = *(size_t*)&stack[i];
            sindex = new_sindex;
            
            
            // if (unlikely((*(unsigned int*)&stack[sindex-3].dist) < (*(unsigned int*)&stack[sindex-2].dist)))
            //   std::swap(*((unsigned long*)&stack[sindex-3].dist),(*(unsigned long*)&stack[sindex-2]));
            
            // if (unlikely((*(unsigned int*)&stack[sindex-2].dist) < (*(unsigned int*)&stack[sindex-1].dist)))
            //   std::swap(*((unsigned long*)&stack[sindex-2].dist),(*(unsigned long*)&stack[sindex-1]));
            
#if 0
            // std::cout << std::endl;
            // for (size_t i=0;i<sindex;i++)
            //   std::cout << i << " " << stack[i].dist << std::endl;
	    
            // size_t min_index = 0;
            // for(size_t i = 1;i<sindex;i++)
            //   if (stack[i].dist < stack[min_index].dist)
            //     min_index = i;
            
            // std::swap(*((unsigned long*)&stack[sindex-1]),(*(unsigned long*)&stack[min_index]));
	    
            for(size_t i = sindex-4;i<sindex;++i)
            {
              StackEntry v = stack[i];
              size_t j = i;
              while(j > 0 && v > stack[j-1])
              {
                stack[j] = stack[j-1];
                --j;
              }
              stack[j] = v;
            }
            
            
            // for (size_t i=0;i<sindex;i++)
            //   std::cout << i << " " << stack[i].dist << std::endl;
#endif
            
          }
          
        }
      }
      DBG(DBG_PRINT(ray));
      DBG(exit(0));
      
    }
    
    
#else
    
    
    void BVH8iIntersector8Chunk::intersect(avxb* valid_i, BVH8i* bvh, Ray8& ray)
    {
      DBG(PING);
      avxf stack_near[QBVH_MAX_STACK_DEPTH];
      unsigned int stack_node[QBVH_MAX_STACK_DEPTH];
      
      avxb m_active = *valid_i;
      
      const avx3f &origin    = ray.org;
      const avx3f &direction = ray.dir;
      const avxf   max_dist  = ray.tfar;
      
      const BVH8i::BVH8iNode *const __restrict__ bvh8  = (BVH8i::BVH8iNode*)bvh->qbvh;
      
      const Triangle1 *const __restrict__ accel = (Triangle1*)bvh->accel;
      
      const avxf orgx = origin.x;
      const avxf orgy = origin.y;
      const avxf orgz = origin.z;  
      
      const avxf ulp(6E-8f);
      const avxf direction_x = select(direction.x == 0,ulp,direction.x);
      const avxf direction_y = select(direction.y == 0,ulp,direction.y);
      const avxf direction_z = select(direction.z == 0,ulp,direction.z);
      
      const avxf rdirx = rcp(direction_x);
      const avxf rdiry = rcp(direction_y);
      const avxf rdirz = rcp(direction_z);
      
      const avxf org_rdirx = orgx * rdirx;
      const avxf org_rdiry = orgy * rdiry;
      const avxf org_rdirz = orgz * rdirz;
      
      const avxf inf  = avxf(pos_inf);
      
      /* --  dummy node at index 0, removes branch in inner-loop -- */
      
      stack_node[0] = QBVH_TERMINAL_TOKEN;
      stack_near[0] = inf;
      
      unsigned int* __restrict__ sptr_node = stack_node + 1;
      avxf       * __restrict__ sptr_near = stack_near + 1;
      
      *sptr_node++ = bvh8[0].min_d[0];
      *sptr_near++ = ray.tnear; 
      
      avxf max_distance   = select(m_active,max_dist,avxf(zero));
      
      while (1)
      {
	sptr_node--;
	sptr_near--;
        
	unsigned int curNode = *sptr_node;
        
	if (unlikely(curNode == QBVH_TERMINAL_TOKEN)) break;
        
	avxf minDist = *sptr_near;
	const avxb m_dist = max_distance > minDist;
        
	if (unlikely( none(m_dist) )) continue;
        
	assert(curNode != QBVH_TERMINAL_TOKEN);
        
#if defined(USE_AVXI_OR_MIN_MAX_CMP)
	const avxi min_distance = cast(ray.tnear);
#else
	const avxf min_distance = ray.tnear;
#endif
        
	while(1)
        {
          DBG(DBG_PRINT(curNode));
          
          if (unlikely(qbvhLeaf(curNode) != 0)) break;
          
          assert (curNode != QBVH_TERMINAL_TOKEN);
          
          sptr_node--;
          sptr_near--;
          STAT3(normal.trav_nodes,1,popcnt(m_active),8);
          const BVH8i::BVH8iNode* __restrict__ const bptr = BVH8i::bvh8ChildPtrNoMask(bvh8,curNode);
          
          curNode = *sptr_node;
          minDist = *sptr_near;
          
          DBG(
            DBG_PRINT(rdirx);
            DBG_PRINT(rdiry);
            DBG_PRINT(rdirz);
            
            DBG_PRINT(org_rdirx);
            DBG_PRINT(org_rdiry);
            DBG_PRINT(org_rdirz);
            
            DBG_PRINT(*qptr);
            );
          
#pragma unroll(8)
          for (unsigned int i=0;i<8;i++)
          {
            const unsigned int dd = bptr->min_d[i];
            if (unlikely(dd == QBVH_LEAF_MASK)) { break; }
            
            const avxf lclipMinX = bptr->min_x[i] * rdirx - org_rdirx;
            const avxf lclipMinY = bptr->min_y[i] * rdiry - org_rdiry;
            const avxf lclipMinZ = bptr->min_z[i] * rdirz - org_rdirz;
            const avxf lclipMaxX = bptr->max_x[i] * rdirx - org_rdirx;
            const avxf lclipMaxY = bptr->max_y[i] * rdiry - org_rdiry;
            const avxf lclipMaxZ = bptr->max_z[i] * rdirz - org_rdirz;
            
#if !defined(USE_AVXI_OR_MIN_MAX_CMP)
            const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const avxb lhit   = max(lnearP,min_distance) <= min(lfarP,max_distance);      
#else
            const avxi iclipMinX = cast(lclipMinX);
            const avxi iclipMaxX = cast(lclipMaxX);
            const avxi iclipMinY = cast(lclipMinY);
            const avxi iclipMaxY = cast(lclipMaxY);
            const avxi iclipMinZ = cast(lclipMinZ);
            const avxi iclipMaxZ = cast(lclipMaxZ);
            
            const avxi lnearP = max(max(min(iclipMinX, iclipMaxX), min(iclipMinY, iclipMaxY)), min(iclipMinZ, iclipMaxZ));
            const avxi lfarP  = min(min(max(iclipMinX, iclipMaxX), max(iclipMinY, iclipMaxY)), max(iclipMinZ, iclipMaxZ));      
            //const avxb lhit   = cast(max(lnearP,min_distance)) <= cast(min(lfarP,cast(max_distance)));
            const avxb lhit   = max(lnearP,min_distance) <= min(lfarP,cast(max_distance));
            
#endif
            //const avxb lhit   = max(lnearP,ray.tnear) <= min(lfarP,ray.t);
            
            DBG(
              DBG_PRINT(i);
              DBG_PRINT(lhit);
              );
            
            if (likely(any(lhit)))
            {
#if defined(USE_AVXI_OR_MIN_MAX_CMP)
              const avxf boxDist = select(lhit,cast(lnearP),inf);
#else
              const avxf boxDist = select(lhit,lnearP,inf);
#endif
              
              
              sptr_node++;
              sptr_near++;
              
              
              if (any(boxDist < minDist))
              {
                *(sptr_node-1) = curNode;
                *(sptr_near-1) = minDist; 
                
                minDist = boxDist;
                curNode = dd;
                
              }
              else
              {
                *(sptr_node-1) = dd;
                *(sptr_near-1) = boxDist; 
              }
              
              assert(sptr_node - stack_node < QBVH_MAX_STACK_DEPTH);
            }	      
          }
          
        }
        
	if (unlikely(curNode == QBVH_TERMINAL_TOKEN)) {break; }
        
	assert (curNode != QBVH_TERMINAL_TOKEN);
        
	unsigned int itemOffset    = qbvhItemOffset(curNode);
        
	const Triangle1  *__restrict__ tptr    = (Triangle1*)((char*)accel + itemOffset);
	const unsigned int items = qbvhItems(curNode);
        STAT3(normal.trav_leaves,1,popcnt(m_active),8);
        
	assert(items > 0);
	assert(items <= 4);
        
	//prefetch<PFHINT_L1>(tptr +  1); 
	//prefetch<PFHINT_L1>(tptr +  2); 
	//prefetch<PFHINT_L1>(tptr +  3); 
        
	const avxf zero_ = avxf(zero);
        
	for(unsigned int i=0;i<items;i++,tptr++) 
	{
	  DBG(DBG_PRINT(*tptr));
          
          STAT3(normal.trav_prims,1,popcnt(m_active),8);
          const avxf v0 = broadcast4f(&tptr->v0);
          const avxf v1 = broadcast4f(&tptr->v1);
          const avxf v2 = broadcast4f(&tptr->v2);
          const avxf e1 = v0-v1;
          const avxf e2 = v2-v0;
          
          const avxf v0_x = shuffle<0>(v0);
          const avxf v0_y = shuffle<1>(v0);
          const avxf v0_z = shuffle<2>(v0);
          
          
          const avx3f org =  avx3f(avxf(v0_x) - orgx,avxf(v0_y) - orgy,avxf(v0_z) - orgz);
          
          //const vec3f normal = cross(e1,e2);
          
          const avxf nz = avxf(tptr->normal.z);
          const avxf nx = avxf(tptr->normal.x);
          const avxf ny = avxf(tptr->normal.y);
          
          const avxf den = direction_z * nz + direction_y * ny + direction_x * nx;
          
          DBG(DBG_PRINT(den));
          
          const avxf sgnDet = signmsk(den);
          const avxf absDet = abs(den);
          
          //const avxf rcp_den = rcp(den);
          
          
          const avxf odz = direction_x*org.y - org.x*direction_y;
          const avxf odx = direction_y*org.z - org.y*direction_z;
          const avxf ody = direction_z*org.x - org.z*direction_x;	  
          
          const avxf e1_x = shuffle<0>(e1);
          const avxf e1_y = shuffle<1>(e1);
          const avxf e1_z = shuffle<2>(e1);
          
          const avxf vv = odz*e1_z + ody*e1_y + odx*e1_x;
          
          DBG(DBG_PRINT(vv));
          
          const avxf e2_x = shuffle<0>(e2);
          const avxf e2_y = shuffle<1>(e2);
          const avxf e2_z = shuffle<2>(e2);
          
          const avxf uu = odz*e2_z + ody*e2_y + odx*e2_x;
          
          DBG(DBG_PRINT(uu));
          
          const avxf u = uu^sgnDet; //uu * rcp_den;
          const avxf v = vv^sgnDet; //vv * rcp_den;
          
          
          const avxb valid_u = m_active & (u >= zero_);
          const avxb valid_v = valid_u  & (v >= zero_);
          
          
          const avxb m_aperture = valid_v & (u+v <= absDet); 
          
          
          if ( unlikely(none(m_aperture)) ) continue;
          
          const avxf rcp_den = rcp(den);
          const avxf nom = org.z       * nz +       org.y * ny +       org.x * nx;
          
          const avxf t = rcp_den*nom;
          const avxb m_t_min = m_aperture & (t >= ray.tnear);
          const avxb m_final  = m_t_min & (t < max_distance);
          
          
	  
          if ( unlikely(none(m_final)) ) continue;
          
          
          const avxf geomID(*(float*)&tptr->geomID);
          const avxf primID(*(float*)&tptr->primID);
          
          max_distance = select(m_final,t,max_distance);
          
          store8f(m_final,(float*)&ray.geomID,geomID);
          store8f(m_final,(float*)&ray.primID,primID);
	  
          store8f(m_final,(float*)&ray.Ng.x,nx);
          store8f(m_final,(float*)&ray.Ng.y,ny);
          store8f(m_final,(float*)&ray.Ng.z,nz);
          
          store8f(m_final,(float*)&ray.u,u*rcp_den);
          store8f(m_final,(float*)&ray.v,v*rcp_den);
          
          DBG(DBG_PRINT(u));
          DBG(DBG_PRINT(v));
          DBG(DBG_PRINT(nx));
          DBG(DBG_PRINT(ny));
          DBG(DBG_PRINT(nz));
          DBG(DBG_PRINT(geomID));
          DBG(DBG_PRINT(primID));
          DBG(DBG_PRINT(t));
          DBG(DBG_PRINT(ray));
	  
	}
        
	// ---------------------------------------------
	// ---------------------------------------------
        
      }
      
      DBG(DBG_PRINT(ray));
      
      DBG(exit(0));
      
      ray.tfar        = select(m_active,max_distance,ray.tfar);
      
    }
    
#endif
    
    void BVH8iIntersector8Chunk::occluded(avxb* valid_i, BVH8i* bvh, Ray8& ray)
    {
#if defined(__TARGET_AVX__) || defined(__TARGET_AVX2__)
#endif
    }
    
    DEFINE_INTERSECTOR8(BVH8iTriangle1Intersector8ChunkMoeller,BVH8iIntersector8Chunk);

    /*REGISTER_INTERSECTOR(BVH8iIntersector8Chunk) 
    {
      ADD_INTERSECTORS_8_16(BVH8i, "bvh8i.triangle1.chunk", BVH8iIntersector8Chunk);
      }*/
  }
}  
  
  // while(1)
  //   {
  
  //     const size_t posN = bitscan(i_lr_hit);
  //     DBG(DBG_PRINT(posN));
  
  //     const unsigned int distN   = ((unsigned int*)&near8)[posN];
  //     const unsigned int nodeN   = bptr->min_d[posN];		  
  
  //     if (curNode_dist <= distN)
  //       {
  // 	stack[sindex].node = nodeN;
  // 	*(unsigned int*)&stack[sindex].dist = distN;
  //       }
  //     else
  //       {
  // 	stack[sindex].node = curNode;
  // 	*(unsigned int*)&stack[sindex].dist = curNode_dist;
  // 	curNode_dist = distN;
  // 	curNode = nodeN;
  //       }
  
  //     sindex++;
  //     assert(sindex < BVH8_MAX_STACK_DEPTH);
  
  //     i_lr_hit &= i_lr_hit-1;
  //     if (likely(i_lr_hit == 0)) break;
  
  //   }
  
