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

#pragma once

#include "bvh4hair.h"

namespace embree
{

  static __forceinline mic_f xfm_row_vector(const mic_f &row0,
					    const mic_f &row1,
					    const mic_f &row2,
					    const mic_f &v)
  {
    const mic_f x = swAAAA(v);
    const mic_f y = swBBBB(v);
    const mic_f z = swCCCC(v);
    const mic_f ret = x * row0 + y * row1 + z * row2;
    return ret;
  }

  static __forceinline mic_f xfm_row_point(const mic_f &row0,
					   const mic_f &row1,
					   const mic_f &row2,
					   const mic_f &v)
  {
    const mic_f x = swAAAA(v);
    const mic_f y = swBBBB(v);
    const mic_f z = swCCCC(v);
    const mic_f trans  = select(0x4444,swDDDD(row2),select(0x2222,swDDDD(row1),swDDDD(row0)));   
    const mic_f ret = x * row0 + y * row1 + z * row2 + trans;
    return ret;
  }

  static __forceinline mic_f xfm(const mic_f &v, const BVH4Hair::UnalignedNode &node)
  {
    // alternative: 'rows' at 'columns' for lane dot products
    const mic_f x = ldot3_xyz(v,node.matrixRowXYZW[0]);
    const mic_f y = ldot3_xyz(v,node.matrixRowXYZW[1]);
    const mic_f z = ldot3_xyz(v,node.matrixRowXYZW[2]);
    const mic_f ret = select(0x4444,z,select(0x2222,y,x));
    return ret;
  }

  __forceinline void traverse_single_intersect(BVH4Hair::NodeRef &curNode,
					       size_t &sindex,
					       const mic_f &dir_xyz,
					       const mic_f &org_xyz1,
					       const mic_f &min_dist_xyz,
					       const mic_f &max_dist_xyz,
					       BVH4Hair::NodeRef *__restrict__ const stack_node,
					       float   *__restrict__ const stack_dist,
					       const size_t leaf_mask)
  {
    const mic_m m7777 = 0x7777; 


    while (1) 
      {
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
		  
	STAT3(normal.trav_nodes,1,1,1);
	const BVH4Hair::UnalignedNode *__restrict__ const u_node = (BVH4Hair::UnalignedNode *)curNode.node();

	prefetch<PFHINT_L1>((char*)u_node + 0*64);
	prefetch<PFHINT_L1>((char*)u_node + 1*64);
	prefetch<PFHINT_L1>((char*)u_node + 2*64);
	prefetch<PFHINT_L1>((char*)u_node + 3*64);
	
	const mic_f row0 = u_node->matrixRowXYZW[0];
	const mic_f row1 = u_node->matrixRowXYZW[1];
	const mic_f row2 = u_node->matrixRowXYZW[2];

	const mic_f xfm_dir_xyz = xfm_row_vector(row0,row1,row2,dir_xyz);
	const mic_f xfm_org_xyz = xfm_row_point (row0,row1,row2,org_xyz1);

	const mic_f rcp_xfm_dir_xyz = rcp_safe( xfm_dir_xyz );
		  		  
	const mic_f tLowerXYZ = -(xfm_org_xyz * rcp_xfm_dir_xyz);
	const mic_f tUpperXYZ = (mic_f::one()  - xfm_org_xyz) * rcp_xfm_dir_xyz;

		    
	mic_m hitm = eq(0x1111, xfm_org_xyz,xfm_org_xyz);

	const mic_f tLower = select(m7777,min(tLowerXYZ,tUpperXYZ),min_dist_xyz);
	const mic_f tUpper = select(m7777,max(tLowerXYZ,tUpperXYZ),max_dist_xyz);



	/* early pop of next node */
	sindex--;
	curNode = stack_node[sindex];


	const mic_f tNear = vreduce_max4(tLower);
	const mic_f tFar  = vreduce_min4(tUpper);  


	hitm = le(hitm,tNear,tFar);

	const mic_f tNear_pos = select(hitm,tNear,inf);
	// todo: packstore 0x1111, avoids index>>2
	STAT3(normal.trav_hit_boxes[countbits(hitm)],1,1,1);

	/* if no child is hit, continue with early popped child */
	if (unlikely(none(hitm))) continue;

		  
	sindex++;        
	const unsigned long hiti = toInt(hitm);
	const unsigned long pos_first = bitscan64(hiti);
	const unsigned long num_hitm = countbits(hiti); 
        
	/* if a single child is hit, continue with that child */
	curNode = u_node->child(pos_first>>2);
	assert(curNode != BVH4Hair::emptyNode);

	if (likely(num_hitm == 1)) continue;
        

	/* if two children are hit, push in correct order */
	const unsigned long pos_second = bitscan64(pos_first,hiti);
	if (likely(num_hitm == 2))
	  {
	    const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
	    const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
	    const BVH4Hair::NodeRef node_first  = curNode;
	    const BVH4Hair::NodeRef node_second = u_node->child(pos_second>>2);

	    assert(node_first  != BVH4Hair::emptyNode);
	    assert(node_second != BVH4Hair::emptyNode);
          
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


	const mic_f min_dist = set_min_lanes(tNear_pos);
	assert(sindex < 3*BVH4i::maxDepth+1);
        
	const mic_m closest_child = eq(hitm,min_dist,tNear);
	const unsigned long closest_child_pos = bitscan64(closest_child);
	const mic_m m_pos = andn(hitm,andn(closest_child,(mic_m)((unsigned int)closest_child - 1)));
	curNode = u_node->child(closest_child_pos>>2);


	assert(curNode  != BVH4Hair::emptyNode);

	long i = -1;
	while((i = bitscan64(i,m_pos)) != BITSCAN_NO_BIT_SET_64)	    
	  {
	    ((unsigned int*)stack_dist)[sindex] = ((unsigned int*)&tNear)[i];		      
	    stack_node[sindex] = u_node->child(i>>2);
	    assert(stack_node[sindex]  != BVH4Hair::emptyNode);
	    sindex++;
	  }
      }
    
  }

  __forceinline void traverse_single_occluded(BVH4Hair::NodeRef &curNode,
					      size_t &sindex,
					      const mic_f &dir_xyz,
					      const mic_f &org_xyz1,
					      const mic_f &min_dist_xyz,
					      const mic_f &max_dist_xyz,
					      BVH4Hair::NodeRef *__restrict__ const stack_node,
					      const size_t leaf_mask)
  {
    const mic_m m7777 = 0x7777; 

    while (1) 
      {
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
		  
	STAT3(normal.trav_nodes,1,1,1);
	const BVH4Hair::UnalignedNode *__restrict__ const u_node = (BVH4Hair::UnalignedNode *)curNode.node();

	prefetch<PFHINT_L1>((char*)u_node + 0*64);
	prefetch<PFHINT_L1>((char*)u_node + 1*64);
	prefetch<PFHINT_L1>((char*)u_node + 2*64);
	prefetch<PFHINT_L1>((char*)u_node + 3*64);


	const mic_f row0 = u_node->matrixRowXYZW[0];
	const mic_f row1 = u_node->matrixRowXYZW[1];
	const mic_f row2 = u_node->matrixRowXYZW[2];

	const mic_f xfm_dir_xyz = xfm_row_vector(row0,row1,row2,dir_xyz);
	const mic_f xfm_org_xyz = xfm_row_point (row0,row1,row2,org_xyz1);

	const mic_f rcp_xfm_dir_xyz = rcp_safe( xfm_dir_xyz );
		  		  
	const mic_f tLowerXYZ = -(xfm_org_xyz * rcp_xfm_dir_xyz);
	const mic_f tUpperXYZ = (mic_f::one()  - xfm_org_xyz) * rcp_xfm_dir_xyz;

		    
	mic_m hitm = eq(0x1111, xfm_org_xyz,xfm_org_xyz);

	const mic_f tLower = select(m7777,min(tLowerXYZ,tUpperXYZ),min_dist_xyz);
	const mic_f tUpper = select(m7777,max(tLowerXYZ,tUpperXYZ),max_dist_xyz);


	/* early pop of next node */
	sindex--;
	curNode = stack_node[sindex];


	const mic_f tNear = vreduce_max4(tLower);
	const mic_f tFar  = vreduce_min4(tUpper);  


	hitm = le(hitm,tNear,tFar);

	const mic_f tNear_pos = select(hitm,tNear,inf);

	STAT3(normal.trav_hit_boxes[countbits(hitm)],1,1,1);


	/* if no child is hit, continue with early popped child */
	if (unlikely(none(hitm))) continue;

		  
	sindex++;        
	const unsigned long hiti = toInt(hitm);
	const unsigned long pos_first = bitscan64(hiti);
	const unsigned long num_hitm = countbits(hiti); 
        
	/* if a single child is hit, continue with that child */
	curNode = u_node->child(pos_first>>2);
	assert(curNode != BVH4Hair::emptyNode);

	if (likely(num_hitm == 1)) continue;
        
	/* if two children are hit, push in correct order */
	const unsigned long pos_second = bitscan64(pos_first,hiti);
	if (likely(num_hitm == 2))
	  {
	    const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
	    const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
	    const BVH4Hair::NodeRef node_first  = curNode;
	    const BVH4Hair::NodeRef node_second = u_node->child(pos_second>>2);

	    assert(node_first  != BVH4Hair::emptyNode);
	    assert(node_second != BVH4Hair::emptyNode);
          
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


	const mic_f min_dist = set_min_lanes(tNear_pos);
	assert(sindex < 3*BVH4Hair::maxDepth+1);
        
	const mic_m closest_child = eq(hitm,min_dist,tNear);
	const unsigned long closest_child_pos = bitscan64(closest_child);
	const mic_m m_pos = andn(hitm,andn(closest_child,(mic_m)((unsigned int)closest_child - 1)));
	curNode = u_node->child(closest_child_pos>>2);

	assert(curNode  != BVH4Hair::emptyNode);

	long i = -1;
	while((i = bitscan64(i,m_pos)) != BITSCAN_NO_BIT_SET_64)	    
	  {
	    stack_node[sindex] = u_node->child(i>>2);
	    assert(stack_node[sindex]  != BVH4Hair::emptyNode);
	    sindex++;
	  }
      }  
  }


  __forceinline void compactStack(BVH4Hair::NodeRef *__restrict__ const stack_node,
				  float   *__restrict__ const stack_dist,
				  size_t &sindex,
				  const unsigned int current_dist,
				  const mic_f &max_dist_xyz)
  {
    size_t new_sindex = 1;
    for (size_t s=1;s<sindex;s++)
      if (*(unsigned int*)&stack_dist[s] <= current_dist)
	{
	  stack_dist[new_sindex] = stack_dist[s];
	  stack_node[new_sindex] = stack_node[s];
	  new_sindex++;
	}
    sindex = new_sindex;
    
  }
  
};
