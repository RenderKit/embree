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

#include "bvh4hair.h"

namespace embree
{
  //TODO: pre-multiply v with 1/127.0f

  static __forceinline vfloat16 xfm_row_vector(const vfloat16 &row0,
					    const vfloat16 &row1,
					    const vfloat16 &row2,
					    const vfloat16 &v)
  {
    const vfloat16 x = swAAAA(v);
    const vfloat16 y = swBBBB(v);
    const vfloat16 z = swCCCC(v);
    const vfloat16 ret = x * row0 + y * row1 + z * row2;
    return ret;
  }

  static __forceinline vfloat16 xfm_row_point(const vfloat16 &row0,
					   const vfloat16 &row1,
					   const vfloat16 &row2,
					   const vfloat16 &v)
  {
    const vfloat16 x = swAAAA(v);
    const vfloat16 y = swBBBB(v);
    const vfloat16 z = swCCCC(v);
    const vfloat16 trans  = select(0x4444,swDDDD(row2),select(0x2222,swDDDD(row1),swDDDD(row0)));   
    const vfloat16 ret = x * row0 + y * row1 + z * row2 + trans;
    return ret;
  }

  static __forceinline vfloat16 xfm(const vfloat16 &v, const BVH4Hair::UnalignedNode &node)
  {
    // alternative: 'rows' at 'columns' for lane dot products

    const vfloat16 x = ldot3_xyz(v,node.getRow(0));
    const vfloat16 y = ldot3_xyz(v,node.getRow(1));
    const vfloat16 z = ldot3_xyz(v,node.getRow(2));
    const vfloat16 ret = select(0x4444,z,select(0x2222,y,x));
    return ret;
  }

  __forceinline void traverse_single_intersect(BVH4Hair::NodeRef &curNode,
					       size_t &sindex,
					       const vfloat16 &dir_xyz,
					       const vfloat16 &org_xyz1,
					       const vfloat16 &rdir_xyz,
					       const vfloat16 &org_rdir_xyz,
					       const vfloat16 &min_dist_xyz,
					       const vfloat16 &max_dist_xyz,
					       BVH4Hair::NodeRef *__restrict__ const stack_node,
					       float   *__restrict__ const stack_dist,
					       const void      * __restrict__ const nodes,
					       const unsigned int leaf_mask,
					       const unsigned int alignednode_mask)
  {
    const vbool16 m7777 = 0x7777; 

    const vint16 invalidNode = vint16::neg_one();

    while (1) 
      {
	if (unlikely(curNode.isLeaf(leaf_mask))) break;

	vbool16 hitm;
	vfloat16 tNear,tFar;

	const BVH4Hair::UnalignedNode *__restrict__ const u_node = (BVH4Hair::UnalignedNode *)curNode.node(nodes);

	BVH4Hair::NodeRef *__restrict__ ref; 
	
	STAT3(normal.trav_nodes,1,1,1);	    

	if (likely((curNode & alignednode_mask) == 0)) // unaligned nodes
	  {	    
	    u_node->prefetchNode<PFHINT_L1>();
	    
	    const vfloat16 row0 = u_node->getRow(0);
	    const vfloat16 row1 = u_node->getRow(1);
	    const vfloat16 row2 = u_node->getRow(2);
		  	
	    const vfloat16 xfm_dir_xyz = xfm_row_vector(row0,row1,row2,dir_xyz);
	    const vfloat16 xfm_org_xyz = xfm_row_vector(row0,row1,row2,org_xyz1); 

	    const vfloat16 rcp_xfm_dir_xyz = rcp_safe( xfm_dir_xyz );
		
	    const vfloat16 xfm_org_rdir_xyz = xfm_org_xyz * rcp_xfm_dir_xyz;
	    const vfloat16 tLowerXYZ = msub(rcp_xfm_dir_xyz,vfloat16::load((float*)u_node->lower),xfm_org_rdir_xyz);
	    const vfloat16 tUpperXYZ = msub(rcp_xfm_dir_xyz,vfloat16::load((float*)u_node->upper),xfm_org_rdir_xyz);
		   
	    const vfloat16 tLower = select(m7777,min(tLowerXYZ,tUpperXYZ),min_dist_xyz);
	    const vfloat16 tUpper = select(m7777,max(tLowerXYZ,tUpperXYZ),max_dist_xyz);
	    ref = u_node->nodeRefPtr();

	    hitm = ne(0x8888,invalidNode,vint16::load((const int*)ref));

	    /* early pop of next node */
	    sindex--;
	    curNode = stack_node[sindex];

	    tNear = vreduce_max4(tLower);
	    tFar  = vreduce_min4(tUpper);  


	  }
	else
	  {
	    prefetch<PFHINT_L1>((vfloat16*)u_node + 0);
	    prefetch<PFHINT_L1>((vfloat16*)u_node + 1);

	    const BVH4Hair::AlignedNode* __restrict__ node = (BVH4Hair::AlignedNode*)((size_t)u_node ^ BVH4Hair::alignednode_mask);

	    //node->prefetchNode<PFHINT_L1>();

	    const float* __restrict const plower = (float*)node->lower;
	    const float* __restrict const pupper = (float*)node->upper;

        
	    /* intersect single ray with 4 bounding boxes */
	    ref = node->nodeRefPtr();
	    hitm = ne(0x8888,invalidNode,vint16::load((const int*)ref));


	    const vfloat16 tLowerXYZ = vfloat16::load(plower) * rdir_xyz - org_rdir_xyz;
	    const vfloat16 tUpperXYZ = vfloat16::load(pupper) * rdir_xyz - org_rdir_xyz;
	    const vfloat16 tLower = mask_min(0x7777,min_dist_xyz,tLowerXYZ,tUpperXYZ);
	    const vfloat16 tUpper = mask_max(0x7777,max_dist_xyz,tLowerXYZ,tUpperXYZ);


	    /* if no child is hit, continue with early popped child */	    
	    sindex--;
	    curNode = stack_node[sindex];
	    tNear = vreduce_max4(tLower);
	    tFar  = vreduce_min4(tUpper);  
	  }

	hitm = le(hitm,tNear,tFar);
	const vfloat16 tNear_pos = select(hitm,tNear,inf);

	STAT3(normal.trav_hit_boxes[countbits(hitm)],1,1,1);

	/* if no child is hit, continue with early popped child */
	if (unlikely(none(hitm))) continue;

		  
	sindex++;        
	const unsigned long hiti = toInt(hitm);
	const unsigned long pos_first = bitscan64(hiti);
	const unsigned long num_hitm = countbits(hiti); 
        
	assert(num_hitm <= 4);

	/* if a single child is hit, continue with that child */
	//curNode = u_node->child_ref(pos_first);
	curNode = ref[pos_first];

	assert(curNode != BVH4Hair::invalidNode);

	if (likely(num_hitm == 1)) continue;
        

	/* if two children are hit, push in correct order */
	const unsigned long pos_second = bitscan64(pos_first,hiti);
	if (likely(num_hitm == 2))
	  {
	    const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
	    const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
	    const BVH4Hair::NodeRef node_first  = curNode;
	    const BVH4Hair::NodeRef node_second = ref[pos_second];

	    assert(node_first  != BVH4Hair::invalidNode);
	    assert(node_second != BVH4Hair::invalidNode);
          
	    if (dist_first <= dist_second)
	      {			  
		stack_node[sindex] = node_second;
		((unsigned int*)stack_dist)[sindex] = dist_second;                      
		sindex++;
		assert(sindex < 3*BVH4Hair::maxDepth+1);
		continue;
	      }
	    else
	      {
		stack_node[sindex] = node_first;
		((unsigned int*)stack_dist)[sindex] = dist_first;
		curNode = node_second;
		sindex++;
		assert(sindex < 3*BVH4Hair::maxDepth+1);
		continue;
	      }
	  }

	/* continue with closest child and push all others */

	const vfloat16 min_dist = set_min_lanes(tNear_pos);
	const unsigned int old_sindex = sindex;
	sindex += countbits(hiti) - 1;
	assert(sindex < 3*BVH4Hair::maxDepth+1);
	const vint16 children = vint16::load((const int*)ref); 
        
	const vbool16 closest_child = eq(hitm,min_dist,tNear);
	const unsigned long closest_child_pos = bitscan64(closest_child);
	const vbool16 m_pos = andn(hitm,andn(closest_child,(vbool16)((unsigned int)closest_child - 1)));
	curNode = ref[closest_child_pos];

	vfloat16::storeu_compact(m_pos,&stack_dist[old_sindex],tNear);
	vint16::storeu_compact(m_pos,&stack_node[old_sindex],children);

	if (unlikely(((unsigned int*)stack_dist)[sindex-3] < ((unsigned int*)stack_dist)[sindex-2]))
	  {
	    std::swap(((unsigned int*)stack_dist)[sindex-2],((unsigned int*)stack_dist)[sindex-3]);
	    std::swap(((unsigned int*)stack_node)[sindex-2],((unsigned int*)stack_node)[sindex-3]);
	  }

	if (unlikely(((unsigned int*)stack_dist)[sindex-2] < ((unsigned int*)stack_dist)[sindex-1]))
	  {
	    std::swap(((unsigned int*)stack_dist)[sindex-1],((unsigned int*)stack_dist)[sindex-2]);
	    std::swap(((unsigned int*)stack_node)[sindex-1],((unsigned int*)stack_node)[sindex-2]);
	  }
      }
 
  }



  __forceinline void traverse_single_occluded(BVH4Hair::NodeRef &curNode,
					      size_t &sindex,
					      const vfloat16 &dir_xyz,
					      const vfloat16 &org_xyz1,
					      const vfloat16 &rdir_xyz,
					      const vfloat16 &org_rdir_xyz,
					      const vfloat16 &min_dist_xyz,
					      const vfloat16 &max_dist_xyz,
					      BVH4Hair::NodeRef *__restrict__ const stack_node,
					      const void      * __restrict__ const nodes,
					      const unsigned int leaf_mask)
  {
    const vbool16 m7777 = 0x7777; 
    const vint16 invalidNode = vint16::neg_one();
    
    while (1) 
      {
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
		  
	vbool16 hitm;
	vfloat16 tNear,tFar;

	const BVH4Hair::UnalignedNode *__restrict__ const u_node = (BVH4Hair::UnalignedNode *)curNode.node(nodes);

	BVH4Hair::NodeRef *__restrict__ ref; 
	
	STAT3(normal.trav_nodes,1,1,1);	    

	if (likely(((size_t)curNode & BVH4Hair::alignednode_mask) == 0)) // unaligned nodes
	  {	    
	    u_node->prefetchNode<PFHINT_L1>();
	    
	    const vfloat16 row0 = u_node->getRow(0);
	    const vfloat16 row1 = u_node->getRow(1);
	    const vfloat16 row2 = u_node->getRow(2);
		  	
	    const vfloat16 xfm_dir_xyz = xfm_row_vector(row0,row1,row2,dir_xyz);
	    const vfloat16 xfm_org_xyz = xfm_row_vector(row0,row1,row2,org_xyz1); 

	    const vfloat16 rcp_xfm_dir_xyz = rcp_safe( xfm_dir_xyz );
		
	    const vfloat16 xfm_org_rdir_xyz = xfm_org_xyz * rcp_xfm_dir_xyz;
	    const vfloat16 tLowerXYZ = msub(rcp_xfm_dir_xyz,vfloat16::load((float*)u_node->lower),xfm_org_rdir_xyz);
	    const vfloat16 tUpperXYZ = msub(rcp_xfm_dir_xyz,vfloat16::load((float*)u_node->upper),xfm_org_rdir_xyz);
		   
	    const vfloat16 tLower = select(m7777,min(tLowerXYZ,tUpperXYZ),min_dist_xyz);
	    const vfloat16 tUpper = select(m7777,max(tLowerXYZ,tUpperXYZ),max_dist_xyz);

	    /* early pop of next node */
	    sindex--;
	    curNode = stack_node[sindex];

	    tNear = vreduce_max4(tLower);
	    tFar  = vreduce_min4(tUpper);  
	    ref = u_node->nodeRefPtr();

	    hitm = ne(0x8888,invalidNode,vint16::load((const int*)ref));

	  }
	else
	  {
	    prefetch<PFHINT_L1>((vfloat16*)u_node + 0);
	    prefetch<PFHINT_L1>((vfloat16*)u_node + 1);

	    const BVH4Hair::AlignedNode* __restrict__ node = (BVH4Hair::AlignedNode*)((size_t)u_node ^ BVH4Hair::alignednode_mask);

	    //node->prefetchNode<PFHINT_L1>();

	    const float* __restrict const plower = (float*)node->lower;
	    const float* __restrict const pupper = (float*)node->upper;

        
	    /* intersect single ray with 4 bounding boxes */
	    ref = node->nodeRefPtr();
	    hitm = ne(0x8888,invalidNode,vint16::load((const int*)ref));


	    const vfloat16 tLowerXYZ = vfloat16::load(plower) * rdir_xyz - org_rdir_xyz;
	    const vfloat16 tUpperXYZ = vfloat16::load(pupper) * rdir_xyz - org_rdir_xyz;
	    const vfloat16 tLower = mask_min(0x7777,min_dist_xyz,tLowerXYZ,tUpperXYZ);
	    const vfloat16 tUpper = mask_max(0x7777,max_dist_xyz,tLowerXYZ,tUpperXYZ);


	    /* if no child is hit, continue with early popped child */	    
	    sindex--;
	    curNode = stack_node[sindex];
	    tNear = vreduce_max4(tLower);
	    tFar  = vreduce_min4(tUpper);  
	  }

	hitm = le(hitm,tNear,tFar);
	const vfloat16 tNear_pos = select(hitm,tNear,inf);

	STAT3(normal.trav_hit_boxes[countbits(hitm)],1,1,1);

	/* if no child is hit, continue with early popped child */
	if (unlikely(none(hitm))) continue;

		  
	sindex++;        
	const unsigned long hiti = toInt(hitm);
	const unsigned long pos_first = bitscan64(hiti);
	const unsigned long num_hitm = countbits(hiti); 
        
	assert(num_hitm <= 4);

	/* if a single child is hit, continue with that child */
	//curNode = u_node->child_ref(pos_first);
	curNode = ref[pos_first];

	assert(curNode != BVH4Hair::invalidNode);

	if (likely(num_hitm == 1)) continue;
        

	/* if two children are hit, push in correct order */
	const unsigned long pos_second = bitscan64(pos_first,hiti);
	if (likely(num_hitm == 2))
	  {
	    const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
	    const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
	    const BVH4Hair::NodeRef node_first  = curNode;
	    const BVH4Hair::NodeRef node_second = ref[pos_second];

	    assert(node_first  != BVH4Hair::invalidNode);
	    assert(node_second != BVH4Hair::invalidNode);
          
	    if (dist_first <= dist_second)
	      {			  
		stack_node[sindex] = node_second;
		sindex++;
		assert(sindex < 3*BVH4Hair::maxDepth+1);
		continue;
	      }
	    else
	      {
		stack_node[sindex] = node_first;
		curNode = node_second;
		sindex++;
		assert(sindex < 3*BVH4Hair::maxDepth+1);
		continue;
	      }
	  }

	/* continue with closest child and push all others */

	const vfloat16 min_dist = set_min_lanes(tNear_pos);
	const unsigned int old_sindex = sindex;
	sindex += countbits(hiti) - 1;
	assert(sindex < 3*BVH4Hair::maxDepth+1);
	const vint16 children = vint16::load((const int*)ref); 
        
	const vbool16 closest_child = eq(hitm,min_dist,tNear);
	const unsigned long closest_child_pos = bitscan64(closest_child);
	const vbool16 m_pos = andn(hitm,andn(closest_child,(vbool16)((unsigned int)closest_child - 1)));
	curNode = ref[closest_child_pos];

	vint16::storeu_compact(m_pos,&stack_node[old_sindex],children);
      }
  }



__forceinline void compactStack(BVH4Hair::NodeRef *__restrict__ const stack_node,
				float   *__restrict__ const stack_dist,
				size_t &sindex,
				const unsigned int current_dist,
				const vfloat16 &max_dist_xyz)
{
#if 0
  size_t new_sindex = 1;
  for (size_t s=1;s<sindex;s++)
    if (((unsigned int*)stack_dist)[s] <= current_dist)
      {
	stack_dist[new_sindex] = stack_dist[s];
	stack_node[new_sindex] = stack_node[s];
	new_sindex++;
      }
  sindex = new_sindex;
#else
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

#endif   
}
  
};

