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

#pragma once

#include "bvh4i.h"

namespace embree
{
  // ====================================================================================================================
  // ====================================================================================================================
  // ====================================================================================================================
  class Precalculations {
  public:
    __forceinline Precalculations(const vfloat16 &org_xyz,
				  const vfloat16 &rdir_xyz) :
    rdir_xyz(rdir_xyz), org_rdir_xyz(rdir_xyz * org_xyz) /*, org_xyz(org_xyz) */ {}
				  
    //const vfloat16 org_xyz;
    const vfloat16 rdir_xyz;
    const vfloat16 org_rdir_xyz;
  };

  template<bool DECOMPRESS_NODE,bool ROBUST>
  __forceinline void traverse_single_intersect(BVH4i::NodeRef &curNode,
					       size_t &sindex,
					       const Precalculations &calc,
					       const vfloat16 &min_dist_xyz,
					       const vfloat16 &max_dist_xyz,
					       BVH4i::NodeRef *__restrict__ const stack_node,
					       float   *__restrict__ const stack_dist,
					       const BVH4i::Node      * __restrict__ const nodes,
					       const unsigned int leaf_mask)
  {
    const vbool16 m7777 = 0x7777; 
    const vbool16 m_rdir0 = lt(m7777,calc.rdir_xyz,vfloat16::zero());
    const vbool16 m_rdir1 = ge(m7777,calc.rdir_xyz,vfloat16::zero());
    
    while (1) 
      {
	/* test if this is a leaf node */
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
	STAT3(normal.trav_nodes,1,1,1);

	const BVH4i::Node* __restrict__ const node = curNode.node(nodes);

	vfloat16 tLowerXYZ = select(m7777,calc.rdir_xyz,min_dist_xyz); 
	vfloat16 tUpperXYZ = select(m7777,calc.rdir_xyz,max_dist_xyz);
	vbool16 hitm = ~m7777; 

	if (!DECOMPRESS_NODE)
	  {
	    const float* __restrict const plower = (float*)node->lower;
	    const float* __restrict const pupper = (float*)node->upper;

	    prefetch<PFHINT_L1>((char*)node + 0);
	    prefetch<PFHINT_L1>((char*)node + 64);
	    
	    /* intersect single ray with 4 bounding boxes */

	    if (ROBUST)
	      {
#if 0
		const vfloat16 lower_org = vfloat16::load(plower) - calc.org_xyz;
		const vfloat16 upper_org = vfloat16::load(pupper) - calc.org_xyz;

		tLowerXYZ = mask_mul_round_down(m_rdir1,tLowerXYZ,lower_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir0,tUpperXYZ,lower_org,calc.rdir_xyz);
		tLowerXYZ = mask_mul_round_down(m_rdir0,tLowerXYZ,upper_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir1,tUpperXYZ,upper_org,calc.rdir_xyz);
#else
		tLowerXYZ = mask_msub_round_down(m_rdir1,tLowerXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir0,tUpperXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tLowerXYZ = mask_msub_round_down(m_rdir0,tLowerXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir1,tUpperXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
#endif
	      }
	    else
	      {

		tLowerXYZ = mask_msub(m_rdir1,tLowerXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir0,tUpperXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tLowerXYZ = mask_msub(m_rdir0,tLowerXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir1,tUpperXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
	      }
	  }
	else
	  {
	    BVH4i::QuantizedNode* __restrict__ const compressed_node = (BVH4i::QuantizedNode*)node;
	    prefetch<PFHINT_L1>((char*)node + 0);
		  
	    //PRINT(*compressed_node);
	    const vfloat16 startXYZ = compressed_node->decompress_startXYZ();
	    const vfloat16 diffXYZ  = compressed_node->decompress_diffXYZ();
	    const vfloat16 clower   = compressed_node->decompress_lowerXYZ(startXYZ,diffXYZ);
	    const vfloat16 cupper   = compressed_node->decompress_upperXYZ(startXYZ,diffXYZ);

	    if (ROBUST)
	      {
#if 0
		const vfloat16 lower_org = clower - calc.org_xyz;
		const vfloat16 upper_org = cupper - calc.org_xyz;

		tLowerXYZ = mask_mul_round_down(m_rdir1,tLowerXYZ,lower_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir0,tUpperXYZ,lower_org,calc.rdir_xyz);
		tLowerXYZ = mask_mul_round_down(m_rdir0,tLowerXYZ,upper_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir1,tUpperXYZ,upper_org,calc.rdir_xyz);
#else
		tLowerXYZ = mask_msub_round_down(m_rdir1,tLowerXYZ,clower,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir0,tUpperXYZ,clower,calc.org_rdir_xyz);
		tLowerXYZ = mask_msub_round_down(m_rdir0,tLowerXYZ,cupper,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir1,tUpperXYZ,cupper,calc.org_rdir_xyz);
#endif

	      }
	    else
	      {
		tLowerXYZ = mask_msub(m_rdir1,tLowerXYZ,clower,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir0,tUpperXYZ,clower,calc.org_rdir_xyz);
		tLowerXYZ = mask_msub(m_rdir0,tLowerXYZ,cupper,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir1,tUpperXYZ,cupper,calc.org_rdir_xyz);	    
	      }
	  }

	const vfloat16 tLower = tLowerXYZ;
	const vfloat16 tUpper = tUpperXYZ;

	/* early pop of next node */
	sindex--;
	curNode = stack_node[sindex];

#ifdef RTCORE_STAT_COUNTERS
	if (!curNode.isLeaf(leaf_mask))
	  STAT3(normal.trav_stack_nodes,1,1,1);
#endif


	const vfloat16 tNear = vreduce_max4(tLower);
	const vfloat16 tFar  = vreduce_min4(tUpper);  
#if 0
	if (ROBUST)
	  {
	    const float round_down = 1.0f-2.0f*float(ulp);
	    const float round_up   = 1.0f+2.0f*float(ulp);
	    hitm = le(hitm,mul_round_down(round_down,tNear),mul_round_up(round_up,tFar));
	  }
	else
#endif
	  hitm = le(hitm,tNear,tFar);
		  
	const vfloat16 tNear_pos = select(hitm,tNear,inf);

	STAT3(normal.trav_hit_boxes[countbits(hitm)],1,1,1);

	const vint16 plower_node = vint16::load((int*)node);


	/* if no child is hit, continue with early popped child */
	if (unlikely(none(hitm))) continue;

	sindex++;        
	const unsigned long hiti = toInt(hitm);
	const unsigned long pos_first = bitscan64(hiti);
	const unsigned long num_hitm = countbits(hiti); 
        
	/* if a single child is hit, continue with that child */
	curNode = ((unsigned int *)node)[pos_first];

	if (likely(num_hitm == 1)) continue;
        
	/* if two children are hit, push in correct order */
	const unsigned long pos_second = bitscan64(pos_first,hiti);
	if (likely(num_hitm == 2))
	  {
	    const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
	    const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
	    const unsigned int node_first  = curNode;
	    const unsigned int node_second = ((unsigned int*)node)[pos_second];
          
	    if (dist_first <= dist_second)
	      {
			  
		stack_node[sindex] = node_second;
		((unsigned int*)stack_dist)[sindex] = dist_second;                      
		sindex++;
		assert(sindex < 3*BVH4i::maxDepth+1);
		continue;
	      }
	    else
	      {
		stack_node[sindex] = curNode;
		((unsigned int*)stack_dist)[sindex] = dist_first;
		curNode = node_second;
		sindex++;
		assert(sindex < 3*BVH4i::maxDepth+1);
		continue;
	      }
	  }

	/* continue with closest child and push all others */


	const vfloat16 min_dist = set_min_lanes(tNear_pos);
	const unsigned int old_sindex = sindex;
	sindex += countbits(hiti) - 1;
	assert(sindex < 3*BVH4i::maxDepth+1);
        
	const BVH4i::Node* __restrict__ const next = curNode.node(nodes);
	prefetch<PFHINT_L1>((char*)next + 0*64);
	prefetch<PFHINT_L1>((char*)next + 1*64);

	const vbool16 closest_child = eq(hitm,min_dist,tNear);
	const unsigned long closest_child_pos = bitscan64(closest_child);
	const vbool16 m_pos = andn(hitm,andn(closest_child,(vbool16)((unsigned int)closest_child - 1)));
	curNode = ((unsigned int*)node)[closest_child_pos];
	vfloat16::storeu_compact(m_pos,&stack_dist[old_sindex],tNear);
	vint16::storeu_compact(m_pos,&stack_node[old_sindex],plower_node);

	if (unlikely(((unsigned int*)stack_dist)[sindex-2] < ((unsigned int*)stack_dist)[sindex-1]))
	  {
	    std::swap(((unsigned int*)stack_dist)[sindex-1],((unsigned int*)stack_dist)[sindex-2]);
	    std::swap(((unsigned int*)stack_node)[sindex-1],((unsigned int*)stack_node)[sindex-2]);
	  }
      }

  }

  template<bool DECOMPRESS_NODE,bool ROBUST>
  __forceinline void traverse_single_occluded(BVH4i::NodeRef &curNode,
					      size_t &sindex,
					      const Precalculations &calc,
					      const vfloat16 &min_dist_xyz,
					      const vfloat16 &max_dist_xyz,
					      BVH4i::NodeRef *__restrict__ const stack_node,
					      const BVH4i::Node      * __restrict__ const nodes,
					      const unsigned int leaf_mask)
  {
    const vbool16 m7777 = 0x7777; 
    const vbool16 m_rdir0 = lt(m7777,calc.rdir_xyz,vfloat16::zero());
    const vbool16 m_rdir1 = ge(m7777,calc.rdir_xyz,vfloat16::zero());

    while (1) 
      {
	/* test if this is a leaf node */
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
	STAT3(shadow.trav_nodes,1,1,1);

	const BVH4i::Node* __restrict__ const node = curNode.node(nodes);

	vfloat16 tLowerXYZ = select(m7777,calc.rdir_xyz,min_dist_xyz); 
	vfloat16 tUpperXYZ = select(m7777,calc.rdir_xyz,max_dist_xyz);
	vbool16 hitm = ~m7777; 

	if (!DECOMPRESS_NODE)
	  {
	    const float* __restrict const plower = (float*)node->lower;
	    const float* __restrict const pupper = (float*)node->upper;

	    prefetch<PFHINT_L1>((char*)node + 0);
	    prefetch<PFHINT_L1>((char*)node + 64);
	    
	    /* intersect single ray with 4 bounding boxes */

	    if (ROBUST)
	      {
#if 0
		const vfloat16 lower_org = vfloat16::load(plower) - calc.org_xyz;
		const vfloat16 upper_org = vfloat16::load(pupper) - calc.org_xyz;

		tLowerXYZ = mask_mul_round_down(m_rdir1,tLowerXYZ,lower_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir0,tUpperXYZ,lower_org,calc.rdir_xyz);
		tLowerXYZ = mask_mul_round_down(m_rdir0,tLowerXYZ,upper_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir1,tUpperXYZ,upper_org,calc.rdir_xyz);
#else
		tLowerXYZ = mask_msub_round_down(m_rdir1,tLowerXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir0,tUpperXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tLowerXYZ = mask_msub_round_down(m_rdir0,tLowerXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir1,tUpperXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
#endif
	      }
	    else
	      {
		tLowerXYZ = mask_msub(m_rdir1,tLowerXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir0,tUpperXYZ,vfloat16::load(plower),calc.org_rdir_xyz);
		tLowerXYZ = mask_msub(m_rdir0,tLowerXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir1,tUpperXYZ,vfloat16::load(pupper),calc.org_rdir_xyz);
	      }

	  }
	else
	  {
	    BVH4i::QuantizedNode* __restrict__ const compressed_node = (BVH4i::QuantizedNode*)node;
	    prefetch<PFHINT_L1>((char*)node + 0);
		  
	    const vfloat16 startXYZ = compressed_node->decompress_startXYZ();
	    const vfloat16 diffXYZ  = compressed_node->decompress_diffXYZ();
	    const vfloat16 clower   = compressed_node->decompress_lowerXYZ(startXYZ,diffXYZ);
	    const vfloat16 cupper   = compressed_node->decompress_upperXYZ(startXYZ,diffXYZ);

	    if (ROBUST)
	      {
#if 0
		const vfloat16 lower_org = clower - calc.org_xyz;
		const vfloat16 upper_org = cupper - calc.org_xyz;

		tLowerXYZ = mask_mul_round_down(m_rdir1,tLowerXYZ,lower_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir0,tUpperXYZ,lower_org,calc.rdir_xyz);
		tLowerXYZ = mask_mul_round_down(m_rdir0,tLowerXYZ,upper_org,calc.rdir_xyz);
		tUpperXYZ = mask_mul_round_up(  m_rdir1,tUpperXYZ,upper_org,calc.rdir_xyz);
#else
		tLowerXYZ = mask_msub_round_down(m_rdir1,tLowerXYZ,clower,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir0,tUpperXYZ,clower,calc.org_rdir_xyz);
		tLowerXYZ = mask_msub_round_down(m_rdir0,tLowerXYZ,cupper,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub_round_up  (m_rdir1,tUpperXYZ,cupper,calc.org_rdir_xyz);
#endif


	      }
	    else
	      {
		tLowerXYZ = mask_msub(m_rdir1,tLowerXYZ,clower,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir0,tUpperXYZ,clower,calc.org_rdir_xyz);
		tLowerXYZ = mask_msub(m_rdir0,tLowerXYZ,cupper,calc.org_rdir_xyz);
		tUpperXYZ = mask_msub(m_rdir1,tUpperXYZ,cupper,calc.org_rdir_xyz);	    
	      }
	  }

	const vfloat16 tLower = tLowerXYZ;
	const vfloat16 tUpper = tUpperXYZ;


	sindex--;
	curNode = stack_node[sindex]; // early pop of next node

#ifdef RTCORE_STAT_COUNTERS
	if (!curNode.isLeaf(leaf_mask))
	  STAT3(shadow.trav_stack_nodes,1,1,1);
#endif

	const vfloat16 tNear = vreduce_max4(tLower);
	const vfloat16 tFar  = vreduce_min4(tUpper);  

#if 0
	if (ROBUST)
	  {
	    const float round_down = 1.0f-2.0f*float(ulp);
	    const float round_up   = 1.0f+2.0f*float(ulp);

	    hitm = le(hitm,round_down*tNear,round_up*tFar);
	  }
	else
#endif
	  hitm = le(hitm,tNear,tFar);		  

	const vfloat16 tNear_pos = select(hitm,tNear,inf);


	STAT3(shadow.trav_hit_boxes[countbits(hitm)],1,1,1);


	/* if no child is hit, continue with early popped child */
	const vint16 plower_node = vint16::load((int*)node);
	if (unlikely(none(hitm))) continue;
	sindex++;
        
	const unsigned long hiti = toInt(hitm);
	const unsigned long pos_first = bitscan64(hiti);
	const unsigned long num_hitm = countbits(hiti); 
        
	/* if a single child is hit, continue with that child */
	curNode = ((unsigned int *)node)[pos_first];
	if (likely(num_hitm == 1)) continue;
	/* if two children are hit, push in correct order */
	const unsigned long pos_second = bitscan64(pos_first,hiti);
	if (likely(num_hitm == 2))
	  {
	    const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
	    const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
	    const unsigned int node_first  = curNode;
	    const unsigned int node_second = ((unsigned int*)node)[pos_second];
          
	    if (dist_first <= dist_second)
	      {
		stack_node[sindex] = node_second;
		sindex++;
		assert(sindex < 3*BVH4i::maxDepth+1);
		continue;
	      }
	    else
	      {
		stack_node[sindex] = curNode;
		curNode = node_second;
		sindex++;
		assert(sindex < 3*BVH4i::maxDepth+1);
		continue;
	      }
	  }
	/* continue with closest child and push all others */
	const vfloat16 min_dist = set_min_lanes(tNear_pos);
	const unsigned int old_sindex = sindex;
	sindex += countbits(hiti) - 1;

	assert(sindex < 3*BVH4i::maxDepth+1);
        
	const vbool16 closest_child = eq(hitm,min_dist,tNear);
	const unsigned long closest_child_pos = bitscan64(closest_child);
	const vbool16 m_pos = andn(hitm,andn(closest_child,(vbool16)((unsigned int)closest_child - 1)));
	curNode = ((unsigned int*)node)[closest_child_pos];
	vint16::storeu_compact(m_pos,&stack_node[old_sindex],plower_node);
      }

  }

  __forceinline void compactStack(BVH4i::NodeRef *__restrict__ const stack_node,
				  float   *__restrict__ const stack_dist,
				  size_t &sindex,
				  const vfloat16 &max_dist_xyz)
  {
    if (likely(sindex >= 2))
      {
	if (likely(sindex < 16))
	  {
	    const unsigned int m_num_stack = vbool16::shift1[sindex] - 1;
	    const vbool16 m_num_stack_low  = toMask(m_num_stack);
	    const vfloat16 snear_low  = vfloat16::load(stack_dist + 0);
	    const vint16 snode_low  = vint16::load((int*)stack_node + 0);
	    const vbool16 m_stack_compact_low  = le(m_num_stack_low,snear_low,max_dist_xyz) | (vbool16)1;
	    compactustore16f_low(m_stack_compact_low,stack_dist + 0,snear_low);
	    compactustore16i_low(m_stack_compact_low,(int*)stack_node + 0,snode_low);
	    sindex = countbits(m_stack_compact_low);
	    assert(sindex < 16);
	  }
	else if (likely(sindex < 32))
	  {
	    const vbool16 m_num_stack_high = toMask(vbool16::shift1[sindex-16] - 1); 
	    const vfloat16 snear_low  = vfloat16::load(stack_dist + 0);
	    const vfloat16 snear_high = vfloat16::load(stack_dist + 16);
	    const vint16 snode_low  = vint16::load((int*)stack_node + 0);
	    const vint16 snode_high = vint16::load((int*)stack_node + 16);
	    const vbool16 m_stack_compact_low  = le(snear_low,max_dist_xyz) | (vbool16)1;
	    const vbool16 m_stack_compact_high = le(m_num_stack_high,snear_high,max_dist_xyz);
	    vfloat16::storeu_compact(m_stack_compact_low,      stack_dist + 0,snear_low);
	    vint16::storeu_compact(m_stack_compact_low,(int*)stack_node + 0,snode_low);
	    vfloat16::storeu_compact(m_stack_compact_high,      stack_dist + countbits(m_stack_compact_low),snear_high);
	    vint16::storeu_compact(m_stack_compact_high,(int*)stack_node + countbits(m_stack_compact_low),snode_high);
	    assert ((unsigned int )m_num_stack_high == ((vbool16::shift1[sindex] - 1) >> 16));

	    sindex = countbits(m_stack_compact_low) + countbits(m_stack_compact_high);
	    assert(sindex < 32);
	  }
	else
	  {
	    const vbool16 m_num_stack_32 = toMask(vbool16::shift1[sindex-32] - 1); 

	    const vfloat16 snear_0  = vfloat16::load(stack_dist + 0);
	    const vfloat16 snear_16 = vfloat16::load(stack_dist + 16);
	    const vfloat16 snear_32 = vfloat16::load(stack_dist + 32);
	    const vint16 snode_0  = vint16::load((int*)stack_node + 0);
	    const vint16 snode_16 = vint16::load((int*)stack_node + 16);
	    const vint16 snode_32 = vint16::load((int*)stack_node + 32);
	    const vbool16 m_stack_compact_0  = le(               snear_0 ,max_dist_xyz) | (vbool16)1;
	    const vbool16 m_stack_compact_16 = le(               snear_16,max_dist_xyz);
	    const vbool16 m_stack_compact_32 = le(m_num_stack_32,snear_32,max_dist_xyz);

	    sindex = 0;
	    vfloat16::storeu_compact(m_stack_compact_0,      stack_dist + sindex,snear_0);
	    vint16::storeu_compact(m_stack_compact_0,(int*)stack_node + sindex,snode_0);
	    sindex += countbits(m_stack_compact_0);
	    vfloat16::storeu_compact(m_stack_compact_16,      stack_dist + sindex,snear_16);
	    vint16::storeu_compact(m_stack_compact_16,(int*)stack_node + sindex,snode_16);
	    sindex += countbits(m_stack_compact_16);
	    vfloat16::storeu_compact(m_stack_compact_32,      stack_dist + sindex,snear_32);
	    vint16::storeu_compact(m_stack_compact_32,(int*)stack_node + sindex,snode_32);
	    sindex += countbits(m_stack_compact_32);

	    assert(sindex < 48);		  
	  }
      }
  }

  template<bool DECOMPRESS_NODE>
  __forceinline void traverse_chunk_intersect(BVH4i::NodeRef &curNode,
					      vfloat16 &curDist,
					      const Vec3vf16 &rdir,
					      const Vec3vf16 &org_rdir,
					      const vfloat16 &ray_tnear,
					      const vfloat16 &ray_tfar,
					      BVH4i::NodeRef *__restrict__ &sptr_node,
					      vfloat16 *__restrict__ &sptr_dist,
					      const BVH4i::Node      * __restrict__ const nodes,
					      const unsigned int leaf_mask)
  {

    while (1)
      {
	/* test if this is a leaf node */
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
          
	STAT3(normal.trav_nodes,1,popcnt(ray_tfar > curDist),16);
	const BVH4i::Node* __restrict__ const node = curNode.node(nodes);


	/* pop of next node */
	sptr_node--;
	sptr_dist--;
	curNode = *sptr_node; 	  
	curDist = *sptr_dist;

	prefetch<PFHINT_L1>((vfloat16*)node + 0);           
	prefetch<PFHINT_L1>((vfloat16*)node + 1); 

#pragma unroll(4)
	for (unsigned int i=0; i<4; i++)
          {
	    BVH4i::NodeRef child;
	    vfloat16 lclipMinX,lclipMinY,lclipMinZ;
	    vfloat16 lclipMaxX,lclipMaxY,lclipMaxZ;

	    if (!DECOMPRESS_NODE)
	      {
		child = node->lower[i].child;

		lclipMinX = msub(node->lower[i].x,rdir.x,org_rdir.x);
		lclipMinY = msub(node->lower[i].y,rdir.y,org_rdir.y);
		lclipMinZ = msub(node->lower[i].z,rdir.z,org_rdir.z);
		lclipMaxX = msub(node->upper[i].x,rdir.x,org_rdir.x);
		lclipMaxY = msub(node->upper[i].y,rdir.y,org_rdir.y);
		lclipMaxZ = msub(node->upper[i].z,rdir.z,org_rdir.z);
	      }
	    else
	      {
		BVH4i::QuantizedNode* __restrict__ const compressed_node = (BVH4i::QuantizedNode*)node;
		child = compressed_node->child(i);

		const vfloat16 startXYZ = compressed_node->decompress_startXYZ();
		const vfloat16 diffXYZ  = compressed_node->decompress_diffXYZ();
		const vfloat16 clower   = compressed_node->decompress_lowerXYZ(startXYZ,diffXYZ);
		const vfloat16 cupper   = compressed_node->decompress_upperXYZ(startXYZ,diffXYZ);

		lclipMinX = msub(vfloat16(clower[4*i+0]),rdir.x,org_rdir.x);
		lclipMinY = msub(vfloat16(clower[4*i+1]),rdir.y,org_rdir.y);
		lclipMinZ = msub(vfloat16(clower[4*i+2]),rdir.z,org_rdir.z);
		lclipMaxX = msub(vfloat16(cupper[4*i+0]),rdir.x,org_rdir.x);
		lclipMaxY = msub(vfloat16(cupper[4*i+1]),rdir.y,org_rdir.y);
		lclipMaxZ = msub(vfloat16(cupper[4*i+2]),rdir.z,org_rdir.z);		
	      }

	    if (unlikely(i >=2 && child == BVH4i::invalidNode)) break;
	    
            const vfloat16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const vfloat16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const vbool16 lhit   = le(max(lnearP,ray_tnear),min(lfarP,ray_tfar));   
	    const vfloat16 childDist = select(lhit,lnearP,inf);
            const vbool16 m_child_dist = lt(childDist,curDist);
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */


            if (likely(any(lhit)))
	      {
		sptr_node++;
		sptr_dist++;
              
		/* push cur node onto stack and continue with hit child */
		if (any(m_child_dist))
		  {
		    *(sptr_node-1) = curNode;
		    *(sptr_dist-1) = curDist; 
		    curDist = childDist;
		    curNode = child;
		  }              
		/* push hit child onto stack*/
		else 
		  {
		    *(sptr_node-1) = child;
		    *(sptr_dist-1) = childDist; 
		  }
	      }	      

          }
      }

  }


  template<bool DECOMPRESS_NODE>
  __forceinline void traverse_chunk_occluded(BVH4i::NodeRef &curNode,
					     vfloat16 &curDist,
					     const Vec3vf16 &rdir,
					     const Vec3vf16 &org_rdir,
					     const vfloat16 &ray_tnear,
					     const vfloat16 &ray_tfar,
					     const vbool16 &m_active,
					     BVH4i::NodeRef *__restrict__ &sptr_node,
					     vfloat16 *__restrict__ &sptr_dist,
					     const BVH4i::Node      * __restrict__ const nodes,
					     const unsigned int leaf_mask)
  {
    while (1)
      {
	/* test if this is a leaf node */
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
          
	STAT3(shadow.trav_nodes,1,popcnt(ray_tfar > curDist),16);
	const BVH4i::Node* __restrict__ const node = curNode.node(nodes);
          
	prefetch<PFHINT_L1>((vfloat16*)node + 0); 
	prefetch<PFHINT_L1>((vfloat16*)node + 1); 

	/* pop of next node */
	sptr_node--;
	sptr_dist--;
	curNode = *sptr_node; 
	curDist = *sptr_dist;
          	 
#pragma unroll(4)
	for (unsigned int i=0; i<4; i++)
          {
	    BVH4i::NodeRef child;
	    vfloat16 lclipMinX,lclipMinY,lclipMinZ;
	    vfloat16 lclipMaxX,lclipMaxY,lclipMaxZ;

	    if (!DECOMPRESS_NODE)
	      {
		child = node->lower[i].child;

		lclipMinX = msub(node->lower[i].x,rdir.x,org_rdir.x);
		lclipMinY = msub(node->lower[i].y,rdir.y,org_rdir.y);
		lclipMinZ = msub(node->lower[i].z,rdir.z,org_rdir.z);
		lclipMaxX = msub(node->upper[i].x,rdir.x,org_rdir.x);
		lclipMaxY = msub(node->upper[i].y,rdir.y,org_rdir.y);
		lclipMaxZ = msub(node->upper[i].z,rdir.z,org_rdir.z);
	      }
	    else
	      {
		BVH4i::QuantizedNode* __restrict__ const compressed_node = (BVH4i::QuantizedNode*)node;
		child = compressed_node->child(i);

		const vfloat16 startXYZ = compressed_node->decompress_startXYZ();
		const vfloat16 diffXYZ  = compressed_node->decompress_diffXYZ();
		const vfloat16 clower   = compressed_node->decompress_lowerXYZ(startXYZ,diffXYZ);
		const vfloat16 cupper   = compressed_node->decompress_upperXYZ(startXYZ,diffXYZ);

		lclipMinX = msub(vfloat16(clower[4*i+0]),rdir.x,org_rdir.x);
		lclipMinY = msub(vfloat16(clower[4*i+1]),rdir.y,org_rdir.y);
		lclipMinZ = msub(vfloat16(clower[4*i+2]),rdir.z,org_rdir.z);
		lclipMaxX = msub(vfloat16(cupper[4*i+0]),rdir.x,org_rdir.x);
		lclipMaxY = msub(vfloat16(cupper[4*i+1]),rdir.y,org_rdir.y);
		lclipMaxZ = msub(vfloat16(cupper[4*i+2]),rdir.z,org_rdir.z);		
	      }

	    if (unlikely(i >=2 && child == BVH4i::invalidNode)) break;

            const vfloat16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const vfloat16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const vbool16 lhit   = le(m_active,max(lnearP,ray_tnear),min(lfarP,ray_tfar));      
	    const vfloat16 childDist = select(lhit,lnearP,inf);
            const vbool16 m_child_dist = childDist < curDist;
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
	      {
		sptr_node++;
		sptr_dist++;
              
		/* push cur node onto stack and continue with hit child */
		if (any(m_child_dist))
		  {
		    *(sptr_node-1) = curNode; 
		    *(sptr_dist-1) = curDist; 
		    curDist = childDist;
		    curNode = child;
		  }
              
		/* push hit child onto stack*/
		else {
		  *(sptr_node-1) = child;
		  *(sptr_dist-1) = childDist; 
		}
	      }	      
          }
      }
    
  }

};
