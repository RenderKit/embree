// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "bvh.h"
#include "bvh_builder.h"
#include "../builders/primrefgen.h"
#include "../builders/splitter.h"
#include "../geometry/triangle1v.h"
#include "../geometry/quad1v.h"

#include "../common/state.h"
#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

#include "../gpu/AABB.h"
#include "../gpu/AABB3f.h"
#include "../gpu/builder.h"

#define PROFILE 0
#define PROFILE_RUNS 20
#define ENABLE_BREADTH_FIRST_PHASE 1
#define ENABLE_SINGLE_THREAD_SERIAL_BUILD 0
#define ENABLE_STATS 1
#define BUILD_CHECKS 0

namespace embree
{
#if defined(EMBREE_DPCPP_SUPPORT)

  inline float4 Vec3fa_to_float4(const Vec3fa& v)
  {
    return (float4){v.x,v.y,v.z,v.w};
  }
  
  
  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void serial_find_split(cl::sycl::intel::sub_group &subgroup,
									      const gpu::BuildRecord &record,
									      const gpu::AABB *const primref,
									      gpu::BinMapping &binMapping,
									      gpu::BinInfo &binInfo,
									      uint *primref_index0,
									      uint *primref_index1)
  {
    const uint startID = record.start;
    const uint endID   = record.end;
  
    binInfo.init(subgroup);

    const uint subgroupLocalID = subgroup.get_local_id()[0];
    const uint subgroupSize    = subgroup.get_local_range().size();

    for (uint t=startID+subgroupLocalID;t<endID;t+=subgroupSize)
      {
	const uint index = primref_index0[t];
	
	primref_index1[t] = index;	
	binInfo.atomicUpdate(binMapping,primref[index]);      
      }
  }


  inline bool is_left(const gpu::BinMapping &binMapping, const gpu::Split &split, const gpu::AABB &primref)
  {
    const uint   dim  = split.dim;    
#if 1 
    const float lower = ((float*)&primref.lower)[dim]; // FIXME    
    const float upper = ((float*)&primref.upper)[dim];
    const float c     = lower+upper;
    const uint pos    = (uint)cl::sycl::floor((c-((float*)&binMapping.ofs)[dim])*((float*)&binMapping.scale)[dim]); // FIXME
#else
    // ORG OCL CODE    
    const float lower = primref.lower[dim];    
    const float upper = primref.upper[dim];
    const float c     = lower+upper;
    const uint pos    = (uint)cl::sycl::floor((c-binMapping.ofs[dim])*binMapping.scale[dim]);
#endif
    return pos < split.pos;    
  }


  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void serial_partition_index(cl::sycl::intel::sub_group &sg,
										   const gpu::AABB *const primref,
										   const gpu::BinMapping &binMapping,
										   const gpu::BuildRecord &current,
										   gpu::Split &split,
										   gpu::BuildRecord &outLeft,
										   gpu::BuildRecord &outRight,
										   gpu::AABB &outGeometryBoundsLeft,
										   gpu::AABB &outGeometryBoundsRight,
										   uint *const primref_index0,
										   const uint *const primref_index1,
										   const cl::sycl::stream &out)
  {
    const uint subgroupLocalID = sg.get_local_id()[0];
    const uint subgroupSize    = sg.get_local_range().size();

    const uint start = current.start;
    const uint end   = current.end;  
    
    gpu::AABB leftCentroid;
    gpu::AABB rightCentroid;
  
    leftCentroid .init();
    rightCentroid.init();

    gpu::AABB leftAABB;
    gpu::AABB rightAABB;

    leftAABB.init();
    rightAABB.init();
  
    uint* l = primref_index0 + start;
    uint* r = primref_index0 + end;

    /* no valid split, just split in the middle */
    if (split.sah == (float)(INFINITY))
      {      
	for (uint i=start + subgroupLocalID;i<split.pos;i+=subgroupSize)
	  {
	    const uint index       = primref_index1[i];
	    const uint count       = sg.reduce<uint,cl::sycl::intel::plus>(1);
	    leftCentroid.extend(primref[index].centroid2());
	    leftAABB.extend(primref[index]);
	    l[subgroupLocalID] = index;	  
	    l+=count;
	  }

	for (uint i=split.pos + subgroupLocalID;i<end;i+=subgroupSize)
	  {
	    const uint index       = primref_index1[i];
	    const uint count       = sg.reduce<uint,cl::sycl::intel::plus>(1);
	    rightCentroid.extend(primref[index].centroid2());
	    rightAABB.extend(primref[index]);	  
	    r-=count;
	    r[subgroupLocalID] = index;	  
	  }      
      }
    else
      {
	for (uint i=start + subgroupLocalID;i<end;i+=subgroupSize)
	  {
	    const uint index       = primref_index1[i];
	    const uint isLeft      = is_left(binMapping, split,primref[index]) ? 1 : 0;
	    const uint isRight     = 1 - isLeft;
	    const uint countLeft   = sg.reduce<uint,cl::sycl::intel::plus>(isLeft );
	    const uint countRight  = sg.reduce<uint,cl::sycl::intel::plus>(isRight);
	    const uint prefixLeft  = sg.exclusive_scan<uint,cl::sycl::intel::plus>(isLeft);	    
	    const uint prefixRight = sg.exclusive_scan<uint,cl::sycl::intel::plus>(isRight);
          
	    r -= countRight;
      
	    if (isLeft)
	      {
		leftCentroid.extend(primref[index].centroid2());
		leftAABB.extend(primref[index]);	      
		l[prefixLeft] = index;
	      }
	    else
	      {
		rightCentroid.extend(primref[index].centroid2());
		rightAABB.extend(primref[index]);	  	      
		r[prefixRight] = index;
	      }
	    l += countLeft;
	  }
      }

    leftCentroid  = leftCentroid.sub_group_reduce(sg);
    rightCentroid = rightCentroid.sub_group_reduce(sg);
    leftAABB  = leftAABB.sub_group_reduce(sg);
    rightAABB = rightAABB.sub_group_reduce(sg);
  
    if (subgroupLocalID == 0)
      {
	uint pos =  l - primref_index0;  // single lane needs to compute "pos"
	
	outLeft.init(start,pos,leftCentroid);
	outRight.init(pos,end,rightCentroid);
      
	const uint sizeLeft  = outLeft.size();
	const uint sizeRight = outRight.size();
            
	leftAABB.upper.w() = gpu::as_float(sizeLeft);
	rightAABB.upper.w() = gpu::as_float(sizeRight);

	outGeometryBoundsLeft   = leftAABB;
	outGeometryBoundsRight  = rightAABB;
      }
  
  }
  

  /* ======================================== */  
  /* === build bvh for single buildrecord === */
  /* ======================================== */
  
     
  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void bvh_build_serial(cl::sycl::intel::sub_group &subgroup,
									     gpu::BuildRecord &record,
									     gpu::Globals &globals,
									     char *bvh_mem,
									     const gpu::AABB *const primref,
									     uint *primref_index0,
									     uint *primref_index1,
									     gpu::BinInfo &binInfo,
									     gpu::BuildRecord &current,
									     gpu::BuildRecord &brecord,
									     gpu::AABB childrenAABB[BVH_NODE_N],
									     gpu::BuildRecord stack[BUILDRECORD_STACK_SIZE],
									     const cl::sycl::stream &out)
  {
    const uint subgroupLocalID = subgroup.get_local_id()[0];
    const uint cfg_minLeafSize = BVH_LEAF_N_MIN;

    uint sindex = 1;
    stack[0] = record;
        
    while(sindex) 
      {
	/* next element from stack */
	sindex--;      
	current = stack[sindex];

	DBG_BUILD(
	    if (subgroupLocalID == 0)
	      out << "sindex = " << sindex << " current " << current << cl::sycl::endl;
	    );
	
	gpu::BinMapping binMapping;

	const uint items = current.size();
	gpu::Split split;
	
	/*! create a leaf node when #items < threshold */
	if (items <= cfg_minLeafSize)
	  {
	    if (subgroup.get_local_id() == 0)
	      {
		const uint leaf_offset = createLeaf(globals,current.start,items);		
		DBG_BUILD(out << "leaf_offset " << leaf_offset << cl::sycl::endl);
		*current.parent = gpu::encodeOffset(bvh_mem,current.parent,leaf_offset);		
	      }
	  }
	else
	  {
		
	    uint numChildren = 2;
	    struct gpu::BuildRecord *children = &stack[sindex];

	    
	    binMapping.init(current.centroidBounds,BINS);
	    
	    serial_find_split(subgroup,current,primref,binMapping,binInfo,primref_index0,primref_index1);
	    
	    split = binInfo.reduceBinsAndComputeBestSplit16(subgroup,binMapping.scale,current.start,current.end);

	    DBG_BUILD(if (subgroup.get_local_id() == 0) out << "split " << split << cl::sycl::endl);
	    
	    serial_partition_index(subgroup,primref,binMapping,current,split,children[0],children[1],childrenAABB[0],childrenAABB[1],primref_index0,primref_index1,out);
	    
	    while (numChildren < BVH_NODE_N)
	      {
		/*! find best child to split */
		float bestArea = -(float)INFINITY;
		int bestChild = -1;
		for (int i=0; i<numChildren; i++)
		  {
		    /* ignore leaves as they cannot get split */
		    if (children[i].size() <= cfg_minLeafSize) continue;

		    /* find child with largest surface area */
		    if (gpu::halfarea(childrenAABB[i].size()) > bestArea) {
		      bestChild = i;
		      bestArea = gpu::halfarea(childrenAABB[i].size());
		    }
		  }
		if (bestChild == -1) break;

		/* perform best found split */
		brecord = children[bestChild];
		gpu::BuildRecord &lrecord = children[bestChild];
		gpu::BuildRecord &rrecord = children[numChildren];	

		binMapping.init(brecord.centroidBounds,BINS);
		serial_find_split(subgroup,brecord,primref,binMapping,binInfo,primref_index0,primref_index1);
		split = binInfo.reduceBinsAndComputeBestSplit16(subgroup,binMapping.scale,brecord.start,brecord.end);

		DBG_BUILD(if (subgroup.get_local_id() == 0) out << "split " << split << cl::sycl::endl);
		
		serial_partition_index(subgroup,primref,binMapping,brecord,split,lrecord,rrecord,childrenAABB[bestChild],childrenAABB[numChildren],primref_index0,primref_index1,out);		    
		numChildren++;		
	      }

	    /* sort children based on range size */
#if 1
	    const float _sortID = childrenAABB[subgroupLocalID].upper.w();
	    const uint sortID = gpu::as_uint(_sortID);
	    const uint numPrimsIDs = cselect((int)(subgroupLocalID < numChildren), (sortID << BVH_NODE_N_LOG) | subgroupLocalID, (uint)0);
	    const uint IDs = gpu::sortBVHChildrenIDs(subgroup,numPrimsIDs) & (BVH_NODE_N-1);
#else	    
	    const uint IDs = subgroupLocalID; 
#endif
	    /* create bvh node */
	    uint node_offset = gpu::createNode(subgroup,globals,IDs,childrenAABB,numChildren,bvh_mem,out);

	    /* set parent pointer in child build records */
	    struct gpu::QBVHNodeN *node = (struct gpu::QBVHNodeN*)(bvh_mem + node_offset);
	    if (subgroupLocalID < numChildren)
		children[IDs].parent = ((uint *)&node->offset[0]) + subgroupLocalID;

	    /* update parent pointer */
	    if (current.parent != nullptr)
	      *current.parent = gpu::encodeOffset(bvh_mem,current.parent,node_offset);

	    sindex += numChildren;
	  }
      }			      
  }


  /* ======================================== */  
  /* ======================================== */


  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void parallel_find_split(cl::sycl::nd_item<1> &item,
										const gpu::BuildRecord &record,
										const gpu::AABB *const primref,
										gpu::Split &bestSplit,										
										gpu::BinMapping &binMapping,
										gpu::BinInfo2 &binInfo2,
										const uint *const primref_index0,
										uint *const primref_index1,
										const cl::sycl::stream &out)
  {
    const uint localID   = item.get_local_id(0);
    const uint localSize = item.get_local_range().size();
    cl::sycl::intel::sub_group sg = item.get_sub_group();
    const uint subgroupID      = sg.get_group_id()[0];

    const uint startID    = record.start;
    const uint endID      = record.end;
    
    /* init bininfo */    
    if (subgroupID == 0)
      binInfo2.init(sg);

    
    item.barrier(cl::sycl::access::fence_space::local_space);

    for (uint t=startID + localID;t<endID;t+=localSize)
      {
	const uint index = primref_index0[t];
	primref_index1[t] = index;      
	binInfo2.atomicUpdate(binMapping,primref[index],out);
      }

    item.barrier(cl::sycl::access::fence_space::local_space);

    /* find best split */
    if (subgroupID == 0)
	bestSplit = binInfo2.reduceBinsAndComputeBestSplit32(sg,binMapping.scale,startID,endID);

    item.barrier(cl::sycl::access::fence_space::local_space);    
  }

  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void parallel_partition_index(cl::sycl::nd_item<1> &item,
										     const gpu::BuildRecord &record,
										     const gpu::AABB *const primref,
										     gpu::Split &bestSplit,										     
										     gpu::BinMapping &binMapping,										     
										     gpu::BuildRecord *outLeft,
										     gpu::BuildRecord *outRight,
										     gpu::AABB *outGeometryBoundsLeft,
										     gpu::AABB *outGeometryBoundsRight,
										     uint *const primref_index0,
										     const uint *const primref_index1,
										     uint *atomicCountLeft,
										     uint *atomicCountRight,										     
										     const cl::sycl::stream &out)
  {
    const uint localID   = item.get_local_id(0);

    cl::sycl::intel::sub_group sg = item.get_sub_group();
    const uint subgroupID      = sg.get_group_id()[0];
    const uint subgroupLocalID = sg.get_local_id()[0];
    const uint subgroupSize    = sg.get_local_range().size();
    const uint numSubGroups    = sg.get_group_range();

    const uint begin = record.start;
    const uint end   = record.end;
    const uint size  = end - begin;

    if (localID == 0)
      {
	outLeft->init(begin,end);
	outRight->init(begin,end);
	outGeometryBoundsLeft->init(); // FIXME: unnecessary?
	outGeometryBoundsRight->init();
	*atomicCountLeft  = 0;
	*atomicCountRight = 0;	
      }

    item.barrier(cl::sycl::access::fence_space::local_space);

    const gpu::Split split = bestSplit;
    
    gpu::BuildRecord left;
    gpu::BuildRecord right;
    left.init(begin,end);
    right.init(begin,end);

    gpu::AABB leftAABB;
    gpu::AABB rightAABB;
    leftAABB.init();
    rightAABB.init();
      
    const uint startID = begin + ((subgroupID+0)*size/numSubGroups);
    const uint endID   = begin + ((subgroupID+1)*size/numSubGroups);
  
    for (uint i=startID + subgroupLocalID;i<endID;i+=subgroupSize)
      {
	const uint index       = primref_index1[i];
	const uint isLeft      = is_left(binMapping, split,primref[index]) ? 1 : 0;
	const uint isRight     = 1 - isLeft;
	const uint countLeft   = sg.reduce<uint,cl::sycl::intel::plus>(isLeft );
	const uint countRight  = sg.reduce<uint,cl::sycl::intel::plus>(isRight);
	const uint prefixLeft  = sg.exclusive_scan<uint,cl::sycl::intel::plus>(isLeft);
	const uint prefixRight = sg.exclusive_scan<uint,cl::sycl::intel::plus>(isRight);

	uint offsetLeft  = subgroupLocalID == 0 ? gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>(atomicCountLeft,  countLeft) : 0;
	offsetLeft = sg.broadcast<uint>(offsetLeft,0);
	uint offsetRight = subgroupLocalID == 0 ? gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>(atomicCountRight,countRight) : 0;
	offsetRight = sg.broadcast<uint>(offsetRight,0);

	const gpu::AABB &pref = primref[index];
	if (isLeft)
	  {
	    left.extend(pref);
	    leftAABB.extend(pref);
	    primref_index0[begin + offsetLeft + prefixLeft] = index;
	  }
	else
	  {
	    right.extend(pref);
	    rightAABB.extend(pref);
	    primref_index0[end - (offsetRight+countRight) + prefixRight] = index;
	  }             
      }
    left.centroidBounds  = left.centroidBounds.sub_group_reduce(sg);
    right.centroidBounds = right.centroidBounds.sub_group_reduce(sg);
    leftAABB  = leftAABB.sub_group_reduce(sg);
    rightAABB = rightAABB.sub_group_reduce(sg);

    left.centroidBounds.atomic_merge_local(outLeft->centroidBounds);
    right.centroidBounds.atomic_merge_local(outRight->centroidBounds);

    leftAABB.atomic_merge_local(*outGeometryBoundsLeft);
    rightAABB.atomic_merge_local(*outGeometryBoundsRight);

    item.barrier(cl::sycl::access::fence_space::local_space);

    if (localID == 0)
      {
	const uint pos = begin + *atomicCountLeft;  // single first thread needs to compute "pos"
	outLeft->end    = pos;
	outRight->start = pos;
	outGeometryBoundsLeft->lower.w() = 0.0f;
	outGeometryBoundsLeft->upper.w() = gpu::as_float(outLeft->size());	
	outGeometryBoundsRight->lower.w() = 0.0f;      
	outGeometryBoundsRight->upper.w() =  gpu::as_float(outRight->size());

#if BUILD_CHECKS == 1    	
	if (outLeft->end <= begin) out << "pos begin error" << cl::sycl::endl;
	if (outLeft->end >  end  ) out << "pos end error" << cl::sycl::endl;
      
	for (uint i=outLeft->start;i<outLeft->end;i++)
	  {
	    const uint index = primref_index0[i];
	    //printf("left %d -> %d \n",i,index);
	    if (!is_left(binMapping, split,primref[index]))
	      out << "check left " << i << cl::sycl::endl;
	    if (!gpu::checkPrimRefBounds(*outLeft,*outGeometryBoundsLeft, primref[index]))
	      out << "check prim ref bounds left " << i << cl::sycl::endl; 
	  }  
	for (uint i=outRight->start;i<outRight->end;i++)
	  {
	    const uint index =  primref_index0[i];
	    //printf("right %d -> %d \n",i,index);	      
	    if (is_left(binMapping, split,primref[index]))
	      out << "check right " << i << cl::sycl::endl;	    
	    if (!gpu::checkPrimRefBounds(*outRight,*outGeometryBoundsRight,primref[index]))
	      out << "check prim ref bounds right " << i << cl::sycl::endl;
	  }
#endif    	
      }
    
    item.barrier(cl::sycl::access::fence_space::local_space);    
  }

  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void bvh_build_parallel_breadth_first(cl::sycl::nd_item<1> &item,
											     cl::sycl::intel::sub_group &subgroup,
											     gpu::BuildRecord *records,
											     const uint recordID,
											     gpu::Globals &globals,
											     char *bvh_mem,
											     const gpu::AABB *const primref,
											     uint *primref_index0,
											     uint *primref_index1,
											     gpu::Split &bestSplit,
											     gpu::BinInfo2 &binInfo2,
											     gpu::BuildRecord &local_current,
											     gpu::AABB *childrenAABB,
											     gpu::BuildRecord *children,
											     uint *atomicCountLeft,
											     uint *atomicCountRight,
											     const uint subtreeThreshold,
											     const cl::sycl::stream &out)
  {
    const uint cfg_minLeafSize = BVH_LEAF_N_MIN;    
    
    cl::sycl::intel::sub_group sg = item.get_sub_group();    
    const uint localID   = item.get_local_id(0);
    const uint numGroups = item.get_group_range(0);    
    const uint subgroupID      = sg.get_group_id()[0];
    const uint subgroupLocalID = sg.get_local_id()[0];
    
    gpu::BuildRecord &record = records[recordID];

    DBG_BUILD(
	      const uint items = record.size();	      
	      if (localID == 0)
		{
		  out << "items " << items << cl::sycl::endl;
		  out << record << cl::sycl::endl;
		});

    local_current = record;
    
    item.barrier(cl::sycl::access::fence_space::global_and_local);

    if (local_current.size() >= subtreeThreshold)
      {
	gpu::BinMapping binMapping;    
	binMapping.init(record.centroidBounds,2*BINS);    
	parallel_find_split(item,local_current,primref,bestSplit,binMapping,binInfo2,primref_index0,primref_index1,out);

	DBG_BUILD(
		  if (localID == 0)
		    out << "bestSplit " << bestSplit << cl::sycl::endl;
		  );
    
	parallel_partition_index(item,local_current,primref,bestSplit,binMapping,&children[0],&children[1],&childrenAABB[0],&childrenAABB[1],primref_index0,primref_index1,atomicCountLeft,atomicCountRight,out);

	/*! create a leaf node when threshold reached or SAH tells us to stop */
	uint numChildren = 2;
	while (numChildren < BVH_NODE_N)
	  {
	    /*! find best child to split */
	    float bestArea = -(float)INFINITY;
	    int bestChild = -1;
	    for (int i=0; i<numChildren; i++)
	      {
		/* ignore leaves as they cannot get split */
		if (children[i].size() <= cfg_minLeafSize) continue;

		/* find child with largest surface area */
		if (childrenAABB[i].halfArea() > bestArea) {
		  bestChild = i;
		  bestArea = childrenAABB[i].halfArea();
		}
	      }
	    if (bestChild == -1) break;

	    /* perform best found split */
	    gpu::BuildRecord &brecord = children[bestChild];
	    gpu::BuildRecord &lrecord = children[numChildren+0];
	    gpu::BuildRecord &rrecord = children[numChildren+1];	
	  
	    binMapping.init(brecord.centroidBounds,2*BINS);
	    parallel_find_split(item,brecord,primref,bestSplit,binMapping,binInfo2,primref_index0,primref_index1,out);

	    DBG_BUILD(
		      if (localID == 0)
			out << "numChildren " << numChildren << " bestSplit " << bestSplit << cl::sycl::endl;
		      );
	    
	    parallel_partition_index(item,brecord,primref,bestSplit,binMapping,&lrecord,&rrecord,&childrenAABB[numChildren+0],&childrenAABB[numChildren+1],primref_index0,primref_index1,atomicCountLeft,atomicCountRight,out);
	    brecord = rrecord;
	    childrenAABB[bestChild] = childrenAABB[numChildren+1];
	    
	    item.barrier(cl::sycl::access::fence_space::global_and_local);
	  
	    numChildren++;
	  }

	if (localID <= 16 && subgroupID == 0)
	  {
	    /* sort children based on size */
	    const float _sortID = childrenAABB[subgroupLocalID].upper.w();
	    const uint sortID = gpu::as_uint(_sortID);
	    const uint numPrimsIDs = cselect((int)(subgroupLocalID < numChildren), (sortID << BVH_NODE_N_LOG) | subgroupLocalID, (uint)0);
	    const uint IDs = gpu::sortBVHChildrenIDs(subgroup,numPrimsIDs) & (BVH_NODE_N-1);
	    /* update parent pointer*/

	    /* create bvh node */
	    uint node_offset = gpu::createNode(subgroup,globals,IDs,childrenAABB,numChildren,bvh_mem,out);

	    /* set parent pointer in child build records */
	    struct gpu::QBVHNodeN *node = (struct gpu::QBVHNodeN*)(bvh_mem + node_offset);
	    if (subgroupLocalID < numChildren)
	      children[IDs].parent = ((uint *)&node->offset[0]) + subgroupLocalID;
	    
	    /* write out child buildrecords to memory */	  
	    if (localID == 0)
	      {
		DBG_BUILD(out << "numChildren " << numChildren << cl::sycl::endl);
		
		/* update parent pointer */
		if (local_current.parent != nullptr)
		  *local_current.parent = gpu::encodeOffset(bvh_mem,local_current.parent,node_offset);

		uint global_records_offset = gpu::atomic_add<uint,cl::sycl::access::address_space::global_space>(&globals.numBuildRecords_extended,numChildren-1);

		DBG_BUILD(out << "global_records_offset " << global_records_offset << cl::sycl::endl);
		
		records[recordID] = children[0];
		  
		for (uint i=1;i<numChildren;i++)
		  records[globals.numBuildRecords + global_records_offset+i-1] = children[i];				
	      }
	  }
	item.barrier(cl::sycl::access::fence_space::global_and_local);	
      }

    /* last active HW thread ? */
    if (localID == 0)
      {
	const uint sync = gpu::atomic_add<uint,cl::sycl::access::address_space::global_space>(&globals.sync, 1);
	if (sync + 1 == numGroups)
	  {
	    globals.sync = 0;
	    /* set final number of buildrecords */
	    globals.numBuildRecords += globals.numBuildRecords_extended;
	    globals.numBuildRecords_extended = 0;
	    DBG_BUILD(
		      out << "globals.numBuildRecords " << globals.numBuildRecords << cl::sycl::endl;
		      for (uint i=0;i<globals.numBuildRecords;i++)
			out << i << " -> " << records[i] << cl::sycl::endl;
		      );
	  }
      }
  }
  

  inline bool outsideAABBTest(gpu::AABB &big, gpu::AABB &small)
  {
    int4 b0 = small.lower < big.lower;
    int4 b1 = small.upper > big.upper;
    int4 b = b0 | b1;
    return b.x() | b.y() | b.z();
  }

  
  inline void printBVHStats(gpu::Globals *globals, char *bvh_mem, const cl::sycl::stream &out)
  {
    struct StatStack
    {
      gpu::AABB aabb;
      uint node;
      float area;
      uint depth;
      uint tmp;
    };

    out << "BVH STATS:" << cl::sycl::endl;
    
    StatStack stack[BVH_MAX_STACK_ENTRIES];    
    
    float sah_nodes  = 0.0f;
    
    float sah_leaves = 0.0f;
    uint leaves = 0;
    uint inner_nodes = 0;
    uint mixed_inner_nodes = 0;
    uint max_depth = 0;
    uint leaf_items = 0;
    uint inner_nodes_valid_children = 0;
    
    gpu::AABB root_aabb = ((struct gpu::QBVHNodeN*)(bvh_mem + sizeof(gpu::BVHBase)))->getBounds();
    const float root_area = root_aabb.halfArea();
    
    uint sindex = 1;
    stack[0].node = sizeof(gpu::BVHBase);
    stack[0].area = root_area;
    stack[0].aabb = root_aabb;
    stack[0].depth = 1;
    
      while(sindex)
	{
	  sindex--;
	  gpu::NodeRef current    = stack[sindex].node;	
	  float current_area      = stack[sindex].area;
	  gpu::AABB current_aabb  = stack[sindex].aabb;
	  uint current_depth      = stack[sindex].depth;

	  max_depth = max(max_depth,current_depth);
	  
	  if (current & BVH_LEAF_MASK)
	    {
	      unsigned int prims = current.getNumLeafPrims();	    
	      leaf_items += prims;
	      sah_leaves += current_area;
	      leaves++;

	      if (prims > BVH_LEAF_N_MAX)
		out << "too many items in leaf " << prims << cl::sycl::endl;	      

#if 0
	      unsigned int prims_offset = current.getLeafOffset();	      
	      gpu::AABB leafAABB;
	      leafAABB.init();

	      gpu::Quad1v *quads = (gpu::Quad1v *)(bvh_mem + prims_offset);
	    
	      for (uint i=0;i<prims;i++)
		{
		  gpu::AABB quadAABB = quads[i].getBounds();
		  leafAABB.extend(quadAABB);
		}
	      if (outsideAABBTest(current_aabb,leafAABB))
		{
		  out << "leaf error: current " << current << " depth " << current_depth << cl::sycl::endl;
		  out << "current_aabb: " << current_aabb << cl::sycl::endl;
		  out << "leaf bounds:  " << leafAABB << cl::sycl::endl;
		}
#endif	      
	    }
	  else
	    {
	      inner_nodes++;
	      sah_nodes += current_area;
	      gpu::QBVHNodeN *nodeN = (gpu::QBVHNodeN*)(bvh_mem + current);
	      uint children = 0;
	      for (uint i=0;i<BVH_NODE_N;i++)
		{
		  if (nodeN->offset[i] == (unsigned int)-1) break;
		  children++;
		}
	    
	      uint leavesInNode = 0;
	      for (uint i=0;i<BVH_NODE_N;i++)
		{
		  if (nodeN->offset[i] == (unsigned int)-1) break;
		  inner_nodes_valid_children++;
		
		  gpu::AABB aabb = nodeN->getBounds(i);
		  const float area = aabb.halfArea();

		  if (aabb.lower.x() == (float)(INFINITY))
		    {
		      out << "aabb inf error " << i << " current " << (uint)current << " nodeN " << children << cl::sycl::endl;
		      break;
		    }
		
		  if (nodeN->offset[i] > globals->totalAllocatedMem || (int)nodeN->offset[i] < 0)
		    {
		      out << "offset error " << i << " nodeN->offset[i] " << nodeN->offset[i] << cl::sycl::endl;
		      break;
		    }
				
		  stack[sindex].node = current + nodeN->offset[i];
		  stack[sindex].area = area;
		  stack[sindex].aabb = aabb;
		  stack[sindex].depth = current_depth + 1;

		  //out << "current " << (uint)current << " nodeN->offset[i] " << (uint)nodeN->offset[i] << " stack[sindex].node " << (uint)stack[sindex].node << cl::sycl::endl;
		  
		  if (stack[sindex].node & BVH_LEAF_MASK)
		    leavesInNode++;

		  sindex++;
		}
	      if (leavesInNode >0 && leavesInNode != children)
		mixed_inner_nodes++;
	    }
	}
      sah_nodes  *= 1.0f / root_area;
      sah_leaves *= 1.0f / root_area;
      float sah = sah_nodes + sah_leaves;
      float node_util = 100.0f * (float)inner_nodes_valid_children / (inner_nodes * BVH_NODE_N);
      float leaf_util = 100.0f * (float)leaf_items / (leaves * BVH_LEAF_N_MAX);

      out << "BVH_NODE_N " << BVH_NODE_N << " BVH_LEAF_N_MIN " << BVH_LEAF_N_MIN <<  " BVH_LEAF_N_MAX " << BVH_LEAF_N_MAX << cl::sycl::endl;
      out << "allocators: node " << globals->node_mem_allocator_start << " -> " << globals->node_mem_allocator_cur
	  << " ; leaf " << globals->leaf_mem_allocator_start << " -> " << globals->leaf_mem_allocator_cur
	  << " max allocated memory " << globals->totalAllocatedMem << cl::sycl::endl;
      
      out << "inner_nodes " << inner_nodes
	  << " leaves " << leaves
	  << " mixed_inner_nodes " << mixed_inner_nodes
	  << " sah " << sah
	  << " sah_nodes  " << sah_nodes
	  << " sah_leaves " << sah_leaves
	  << " max_depth " << max_depth
	  << " leaf_items " << leaf_items
	  << " node_util " << node_util << "%"
	  << " leaf_util " << leaf_util << "%"
	  << " (" << (float)leaf_items / leaves << " out of " << BVH_LEAF_N_MAX << ")"
	  << cl::sycl::endl;
    
      uint node_mem        = globals->node_mem_allocator_cur-globals->node_mem_allocator_start;
      uint max_node_mem    = globals->leaf_mem_allocator_start;
      float node_mem_ratio = 100.0f * (float)node_mem / max_node_mem;
    
      uint leaf_mem        = globals->leaf_mem_allocator_cur - globals->leaf_mem_allocator_start;
      uint max_leaf_mem    = globals->totalAllocatedMem      - globals->leaf_mem_allocator_start;
      float leaf_mem_ratio = 100.0f * (float)leaf_mem / max_leaf_mem;
		      
      uint total_mem        = node_mem + leaf_mem;
      float total_mem_ratio = 100.0f * (float)total_mem / globals->totalAllocatedMem;
      
      out << "used node memory " << node_mem << " (" << node_mem_ratio
	  << "%) / used leaf memory " << leaf_mem << " (" << leaf_mem_ratio
	  << "%) / total memory used " << total_mem << " (" << total_mem_ratio
	  << "%) / total memory allocated " << globals->totalAllocatedMem << cl::sycl::endl;
  }

#endif

  namespace isa
  {    
    template<int N, typename Mesh, typename Primitive>
    struct BVHGPUBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      GeneralBVHBuilder::Settings settings;
      bool primrefarrayalloc;

      BVHGPUBuilderSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize,
			const size_t mode, bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device,0),
          settings(sahBlockSize, minLeafSize, maxLeafSize, travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(primrefarrayalloc) {}

      BVHGPUBuilderSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device,0), settings(sahBlockSize, minLeafSize, maxLeafSize, travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(false) {}

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
        }

	/* skip build for empty scene */
        const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,false>();
        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

            /* enable USM allocation for primrefs */
#if defined(EMBREE_DPCPP_SUPPORT)
	    assert(sizeof(gpu::Globals) == 4*64);
	    assert(sizeof(gpu::Quad1v)  == 64);
	    assert(sizeof(gpu::Triangle1v) == 48);
	    DeviceGPU* deviceGPU = (DeviceGPU*)scene->device;
	    prims.getAlloc().enableUSM(&deviceGPU->getDevice(),&deviceGPU->getContext());	    
#endif	    

            /* create primref array */	    
            prims.resize(numPrimitives); 

            PrimInfo pinfo = mesh ?
              createPrimRefArray(mesh,prims,bvh->scene->progressInterface) :
              createPrimRefArray(scene,Mesh::geom_type,false,prims,bvh->scene->progressInterface);
	    	    
            /* pinfo might has zero size due to invalid geometry */
            if (unlikely(pinfo.size() == 0))
	      {
		bvh->clear();
		prims.clear();
		return;
	      }
	    PRINT(pinfo.size());

	    NodeRef root(BVH::emptyNode);

#if defined(EMBREE_DPCPP_SUPPORT)
	      
	    cl::sycl::queue &gpu_queue = deviceGPU->getQueue();
	    const int maxWorkGroupSize = deviceGPU->getMaxWorkGroupSize();
	    
	    PRINT(maxWorkGroupSize);

	    /* --- estimate size of the BVH --- */
	    const uint leaf_primitive_size = sizeof(Primitive);
	    const uint node_size       = numPrimitives * sizeof(gpu::QBVHNodeN) / 2;
	    const uint leaf_size       = numPrimitives * leaf_primitive_size;
	    const uint totalSize       = sizeof(gpu::BVHBase) + node_size + leaf_size; 
	    const uint node_data_start = sizeof(gpu::BVHBase);
	    const uint leaf_data_start = sizeof(gpu::BVHBase) + node_size;
	    
	    /* --- allocate and set buffers --- */

	    gpu::AABB *aabb = (gpu::AABB*)prims.data();
	    assert(aabb);

	    char *bvh_mem = (char*)cl::sycl::aligned_alloc(64,totalSize,deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(bvh_mem);

	    uint *primref_index = (uint*)cl::sycl::aligned_alloc(64,sizeof(uint)*2*numPrimitives,deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(primref_index);

	    gpu::Globals *globals = (gpu::Globals*)cl::sycl::aligned_alloc(64,sizeof(gpu::Globals),deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(globals);
	    	    	    
	    /* --- init globals (device) --- */
	    {
	      cl::sycl::event queue_event =  gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cgh.single_task<class init_first_kernel>([=]() {
		      globals->init(bvh_mem,numPrimitives,node_data_start,leaf_data_start,totalSize,0,leaf_primitive_size);
		      globals->leaf_mem_allocator_cur += leaf_primitive_size*numPrimitives;
		    });
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }
	    
	    const cl::sycl::nd_range<1> nd_range1(cl::sycl::range<1>((int)maxWorkGroupSize),cl::sycl::range<1>((int)maxWorkGroupSize));	    
	    {	      
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  cgh.parallel_for<class init_bounds0>(nd_range1,[=](cl::sycl::nd_item<1> item)
		{
		  const uint startID = item.get_global_id(0);
		  const uint step    = item.get_global_range().size();

		  gpu::AABB local_geometry_aabb;
		  gpu::AABB local_centroid_aabb;
		  local_geometry_aabb.init();
		  local_centroid_aabb.init();
		  
		  for (uint i=startID;i<numPrimitives;i+=step)
		    {
		      const gpu::AABB aabb_geom = aabb[i];
		      const gpu::AABB aabb_centroid(aabb_geom.centroid2());
		      primref_index[i] = i;
		      local_geometry_aabb.extend(aabb_geom);
		      local_centroid_aabb.extend(aabb_centroid);		      
		    }
		  cl::sycl::multi_ptr<gpu::Globals,cl::sycl::access::address_space::global_space> ptr(globals);
		  local_geometry_aabb.atomic_merge_global(ptr.get()->geometryBounds);
		  local_centroid_aabb.atomic_merge_global(ptr.get()->centroidBounds);		  
		});
		  
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }

	    /* --- init bvh sah builder (device) --- */
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  cgh.single_task<class init_builder>([=]() {
		      gpu::BuildRecord *record = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      record->init(0,numPrimitives,globals->centroidBounds);
		      globals->numBuildRecords = 1;
		      DBG_BUILD(
				out << "geometryBounds: " << globals->geometryBounds << cl::sycl::endl;
				out << "centroiBounds : " << globals->centroidBounds << cl::sycl::endl;
			  );
		    });
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }

	    /* --- parallel thread breadth first build (device) --- */

#if ENABLE_BREADTH_FIRST_PHASE == 1
	    double t1 = getSeconds();
	    
	    uint old_numBuildRecords = 0;
	    while(old_numBuildRecords != globals->numBuildRecords)
	    {
	      old_numBuildRecords = globals->numBuildRecords;	      
	      const uint subtreeThreshold = 512;	      
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {

		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(globals->numBuildRecords*(int)maxWorkGroupSize),cl::sycl::range<1>((int)maxWorkGroupSize));
		  
		  /* local variables */
		  cl::sycl::accessor< gpu::BinInfo2   , 0, sycl_read_write, sycl_local> binInfo2(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> local_current(cgh);
		  cl::sycl::accessor< gpu::AABB       , 1, sycl_read_write, sycl_local> childrenAABB(cl::sycl::range<1>(BVH_NODE_N+1),cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 1, sycl_read_write, sycl_local> children(cl::sycl::range<1>(BVH_NODE_N+1),cgh);
		  cl::sycl::accessor< gpu::Split      , 0, sycl_read_write, sycl_local> bestSplit(cgh);		  
		  cl::sycl::accessor< uint            , 0, sycl_read_write, sycl_local> atomicCountLeft(cgh);
		  cl::sycl::accessor< uint            , 0, sycl_read_write, sycl_local> atomicCountRight(cgh);
		  		  		  		  
		  cgh.parallel_for<class parallel_build>(nd_range,[=](cl::sycl::nd_item<1> item) {
		      const uint groupID   = item.get_group(0);
		      cl::sycl::intel::sub_group subgroup = item.get_sub_group();		      
		      gpu::AABB *primref     = aabb;
		      uint *primref_index0   = primref_index + 0;
		      uint *primref_index1   = primref_index + globals->numPrimitives;		      
		      gpu::BuildRecord *records = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      		      
		      bvh_build_parallel_breadth_first(item,subgroup,records,groupID,*globals,bvh_mem,primref,primref_index0,primref_index1,bestSplit,binInfo2,local_current,childrenAABB.get_pointer(),children.get_pointer(),atomicCountLeft.get_pointer(),atomicCountRight.get_pointer(),subtreeThreshold,out);
		    });		  
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	      //PRINT(globals->numBuildRecords);	      
	    }

	    double t2 = getSeconds();
	    std::cout << "Parallel Breadth First Phase " << 1000 * (t2 - t1) << " ms" << std::endl;
#endif	    
	    
	    /* --- single HW thread recursive build (device) --- */
	    
	    t1 = getSeconds();
	    
#if ENABLE_SINGLE_THREAD_SERIAL_BUILD == 1
	    const uint numParallelRecords = 1;
#else
	    const uint numParallelRecords = globals->numBuildRecords;	    
#endif	    
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {

		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(numParallelRecords * BVH_NODE_N),cl::sycl::range<1>(BVH_NODE_N));
		  
		  /* local variables */
		  cl::sycl::accessor< gpu::BinInfo    , 0, sycl_read_write, sycl_local> binInfo(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> current(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> brecord(cgh);
		  cl::sycl::accessor< gpu::AABB       , 1, sycl_read_write, sycl_local> childrenAABB(cl::sycl::range<1>(BVH_NODE_N),cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 1, sycl_read_write, sycl_local> stack(cl::sycl::range<1>(BUILDRECORD_STACK_SIZE),cgh);
		  		  		  
		  cgh.parallel_for<class serial_build>(nd_range,[=](cl::sycl::nd_item<1> item) {
		      const uint groupID   = item.get_group(0);
		      cl::sycl::intel::sub_group subgroup = item.get_sub_group();
		      
		      gpu::AABB *primref     = aabb;
		      uint *primref_index0   = primref_index + 0;
		      uint *primref_index1   = primref_index + globals->numPrimitives;		      
		      gpu::BuildRecord *record = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      
#if ENABLE_SINGLE_THREAD_SERIAL_BUILD == 1
		      const uint numRecords = globals->numBuildRecords;		      
		      const uint numGroups = item.get_group_range(0);		      
		      for (uint recordID = groupID;recordID<numRecords;recordID+=numGroups)
#else
			const uint recordID = groupID;
#endif			
			bvh_build_serial(subgroup,record[recordID],*globals,bvh_mem,primref,primref_index0,primref_index1,binInfo,current,brecord,childrenAABB.get_pointer(),stack.get_pointer(),out);
		    });		  
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }

	    t2 = getSeconds();
	    std::cout << "Parallel Depth First Phase " << 1000 * (t2 - t1) << " ms" << std::endl;		    

	    /* --- convert primrefs to primitives (host) --- */

	    Primitive *leaf_prims = (Primitive *)(bvh_mem + globals->leaf_mem_allocator_start);
	    parallel_for( size_t(0), numPrimitives, size_t(1), [&](const range<size_t>& r) -> void {
		for (size_t i=r.begin(); i<r.end(); i++)
		  {
		    const uint index = primref_index[i];
		    const uint geomID = gpu::as_uint((float)aabb[index].lower.w());
		    const uint primID = gpu::as_uint((float)aabb[index].upper.w());
		    leaf_prims[i].init(geomID,primID,scene);
		  }
	      });
	    
	    
            /* call BVH builder */
            root = NodeRef((size_t)bvh_mem);

	    // = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeaf<N,Primitive>(bvh),bvh->scene->progressInterface,prims.data(),pinfo,settings);
#endif	    
            bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());

	    std::cout << "BVH GPU Builder DONE: bvh " << bvh << " bvh->root " << bvh->root << std::endl << std::flush;
	    
	    /* print BVH stats */
#if defined(EMBREE_DPCPP_SUPPORT) && ENABLE_STATS == 1	    
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  cgh.single_task<class printStats>([=]() {
		      printBVHStats(globals,bvh_mem,out);
		    });

		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }	      
	    }
#endif
	    
	    /* --- deallocate temporary data structures --- */
#if defined(EMBREE_DPCPP_SUPPORT)	    
	    cl::sycl::free(primref_index,deviceGPU->getContext());
	    cl::sycl::free(globals      ,deviceGPU->getContext());
#endif
	    
#if PROFILE
          });
#endif

        /* for static geometries we can do some cleanups */
        if (scene && scene->isStaticAccel()) {
          bvh->shrink();
          prims.clear();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
#if defined(EMBREE_GEOMETRY_TRIANGLE)
    Builder* BVHGPUTriangle1vSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHGPUBuilderSAH<4,TriangleMesh,Triangle1v>((BVH4*)bvh,scene,1,1.0f,1,inf,mode,true); }    
    Builder* BVHGPUQuad1vSceneBuilderSAH     (void* bvh, Scene* scene, size_t mode) { return new BVHGPUBuilderSAH<4,QuadMesh,Quad1v>((BVH4*)bvh,scene,1,1.0f,1,inf,mode,true); }    
#endif
    
  }
}
