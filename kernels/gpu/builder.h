// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#if defined(EMBREE_DPCPP_SUPPORT)
#include "common.h"
#include "AABB.h"
#include "AABB3f.h"
#include "bvh.h"
#include "geometry.h"


#define BUILDRECORD_STACK_SIZE 96
#define SAH_LOG_BLOCK_SHIFT     2
#define BVH_LEAF_N_MIN          4
#define BVH_LEAF_N_MAX          4

#define DBG_BUILD(x) 

namespace embree
{
  namespace gpu
  {
    
    struct Globals
    {
      /* 1. cacheline */
      struct AABB geometryBounds;
      struct AABB centroidBounds;

      /* 2. cacheline */
      unsigned int node_mem_allocator_start;
      unsigned int node_mem_allocator_cur;
      unsigned int node_mem_allocator_pad[14];

      /* 3. cacheline */
      unsigned int leaf_mem_allocator_start;
      unsigned int leaf_mem_allocator_cur;
      unsigned int leaf_mem_pad[14];

      /* 4. cacheline */
      unsigned int numPrimitives;
      unsigned int numOriginalPrimitives;
      unsigned int numSplittedPrimitives;
      unsigned int init_numPrimitives;
      unsigned int leafPrimType;
      unsigned int leafSize;
      unsigned int numBuildRecords;
      unsigned int numBuildRecords_extended;
      unsigned int totalAllocatedMem;
      float presplitPrioritySum;

      unsigned int sync;
      float probThreshold;
      unsigned int numGlobalBuildRecords;
      unsigned int counter;
      unsigned int unused[2];

      inline void init(char *bvh_mem,
		       unsigned int _numPrimitives,
		       unsigned int _node_data_start,
		       unsigned int _leaf_data_start,
		       unsigned int _totalBytes,
		       unsigned int _leafPrimType,
		       unsigned int _leafSize)
      {
	struct BVHBase *base       = (struct BVHBase*)bvh_mem;
	base->nodeDataStart        = _node_data_start/64;
	base->nodeDataCur          = _node_data_start/64;
	base->leafDataStart        = _leaf_data_start/64;
	base->leafDataCur          = _leaf_data_start/64;
	base->rootNodeOffset       = _node_data_start; // FIXME: should be set by builder

	geometryBounds.init();
	centroidBounds.init();

	node_mem_allocator_cur     = _node_data_start;
	node_mem_allocator_start   = _node_data_start;
	leaf_mem_allocator_cur     = _leaf_data_start;
	leaf_mem_allocator_start   = _leaf_data_start;

	numBuildRecords          = 0;
	numBuildRecords_extended = 0;
	numPrimitives            = _numPrimitives;
	init_numPrimitives       = 0;
	numSplittedPrimitives    = 0;
	totalAllocatedMem        = _totalBytes;
	sync                     = 0;
	probThreshold            = 0.0f;
	leafPrimType             = _leafPrimType;
	leafSize                 = _leafSize;
	numGlobalBuildRecords    = 0;
      }

            
      inline void resetGlobalCounters()
      {
	node_mem_allocator_cur = node_mem_allocator_start;
	leaf_mem_allocator_cur = leaf_mem_allocator_start;
	numBuildRecords = 0;	
      }
      
      inline uint alloc_node_mem(const uint size)
      {
	const uint aligned_size = ((size+63)/64)*64; /* allocate in 64 bytes blocks */
	cl::sycl::multi_ptr<unsigned int,cl::sycl::access::address_space::global_space> ptr(&node_mem_allocator_cur);
	cl::sycl::atomic<unsigned int> counter(ptr);
	return atomic_fetch_add(counter,aligned_size);
      }

      inline uint alloc_leaf_mem(const uint size)
      {
	const uint aligned_size = ((size+15)/16)*16; /* allocate in 16 bytes blocks */
	cl::sycl::multi_ptr<unsigned int,cl::sycl::access::address_space::global_space> ptr(&leaf_mem_allocator_cur);
	cl::sycl::atomic<unsigned int> counter(ptr);
	return atomic_fetch_add(counter,aligned_size);
      }
      
    };
    
    struct Range {
      unsigned int start, end;
    };

    struct MortonCodePrimitive
    {
      uint64_t index_code; // 64bit code + index combo
    };

    struct BuildRecord
    {
      struct AABB centroidBounds;
      unsigned int start, end;
      uint *parent;

      inline void init(unsigned int _start, unsigned int _end, AABB &bounds)
      {
	centroidBounds = bounds;
	start = _start;
	end   = _end;
	parent = NULL;	
      }

      inline void init(unsigned int _start, unsigned int _end)
      {
	centroidBounds.init();
	start = _start;
	end   = _end;
	parent = NULL;	
      }
      

      inline void extend(const AABB &primref)
      {
	centroidBounds.extend(primref.centroid2());
      }

      inline unsigned int size() const { return end - start; }
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const BuildRecord& r) {
      return out << " start " << r.start << " end " << r.end << " size " << r.size() << " parent " << r.parent << " bounds " << r.centroidBounds;
    }


    struct BinaryMortonCodeHierarchy
    {
      struct Range range;
      unsigned int leftChild;
      unsigned int rightChild;
      unsigned int flag;
    };

    struct StatStackEntry
    {
      struct AABB aabb;
      unsigned int node;
      unsigned int type;
      unsigned int depth;
      float area;
    };

    struct BuildRecordMorton {
      unsigned int nodeID;
      unsigned int items;
      unsigned int current_index;
      unsigned int parent_index;
    };

    struct Split
    {
      float sah;
      uint dim;
      uint pos;

      Split() {}

    Split(const float sah, const uint dim, const uint pos) : sah(sah), dim(dim), pos(pos) {}
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const Split& s) {
      return out << " sah " << s.sah << " dim " << s.dim << " pos " << s.pos;
    }


    struct BinMapping
    {
      float4 ofs, scale;

      inline void init(const AABB &centBounds, const uint bins)
      {
	const float4 eps(1E-34f);
	const float4 diag = max(eps, centBounds.upper - centBounds.lower);
	scale = (float4)(0.99f*(float)bins)/diag;
	scale = cselect(diag > eps, scale, (float4)(0.0f));
	ofs  = centBounds.lower;
      }

    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const BinMapping& bm) {
      return out << "ofs " << bm.ofs.xyz() << " scale " << bm.scale.xyz();
    }


    inline AABB3f convert_AABB3f(const AABB &aabb)
    {
      AABB3f aabb3f;
      aabb3f.lower = aabb.lower.xyz();
      aabb3f.upper = aabb.upper.xyz();      
      return aabb3f;
    }


    [[cl::intel_reqd_sub_group_size(BINS)]] inline float left_to_right_area16(const cl::sycl::intel::sub_group &sg, const AABB3f &low)
      {
	struct AABB3f low_prefix = low.sub_group_scan_exclusive_min_max(sg);
	return low_prefix.halfArea();
      }

    [[cl::intel_reqd_sub_group_size(BINS)]] inline uint left_to_right_counts16(const cl::sycl::intel::sub_group &sg, uint low)
      {
	return sg.exclusive_scan<uint>(low, cl::sycl::intel::plus<>());
      }


    [[cl::intel_reqd_sub_group_size(BINS)]] inline float right_to_left_area16(const cl::sycl::intel::sub_group &sg, const AABB3f &low)
      {
	const uint subgroupLocalID = sg.get_local_id()[0];
	const uint subgroupSize    = sg.get_local_range().size();	
	const uint ID              = subgroupSize - 1 - subgroupLocalID;  
	AABB3f low_reverse         = low.sub_group_shuffle(sg,ID);
	AABB3f low_prefix          = low_reverse.sub_group_scan_inclusive_min_max(sg);
	const float low_area       = sg.shuffle<float>(low_prefix.halfArea(),ID);
	return low_area;
      }

    [[cl::intel_reqd_sub_group_size(BINS)]] inline uint right_to_left_counts16(const cl::sycl::intel::sub_group &sg, uint low)
      {
	const uint subgroupLocalID = sg.get_local_id()[0];
	const uint subgroupSize    = sg.get_local_range().size();	
	const uint ID              = subgroupSize - 1 - subgroupLocalID;  
	const uint low_reverse     = sg.shuffle<uint>(low,ID);
	const uint low_prefix      = sg.inclusive_scan<float>(low_reverse, cl::sycl::intel::plus<>());
	return sg.shuffle<uint>(low_prefix,ID);
      }

    [[cl::intel_reqd_sub_group_size(BINS)]] inline float2 left_to_right_area32(const cl::sycl::intel::sub_group &sg, const AABB3f &low, const AABB3f &high)
      {
	AABB3f low_prefix     = low.sub_group_scan_exclusive_min_max(sg);
	AABB3f low_reduce     = low.sub_group_reduce(sg);	      	      
	AABB3f high_prefix    = high.sub_group_scan_exclusive_min_max(sg);
	high_prefix.extend(low_reduce);
	const float low_area  = low_prefix.halfArea();
	const float high_area = high_prefix.halfArea();
	return float2(low_area,high_area);
      }
    
    [[cl::intel_reqd_sub_group_size(BINS)]] inline uint2 left_to_right_counts32(const cl::sycl::intel::sub_group &sg, uint low, uint high)
      {
	const uint low_prefix  = sg.exclusive_scan<uint>(low, cl::sycl::intel::plus<>());
	const uint low_reduce  = sg.reduce<uint>(low, cl::sycl::intel::plus<>());
	const uint high_prefix = sg.exclusive_scan<uint>(high, cl::sycl::intel::plus<>());
	return uint2(low_prefix,low_reduce+high_prefix);
      }
        
    [[cl::intel_reqd_sub_group_size(BINS)]] inline float2 right_to_left_area32(const cl::sycl::intel::sub_group &sg, const AABB3f &low, const AABB3f &high)
      {
	const uint subgroupLocalID = sg.get_local_id()[0];
	const uint subgroupSize    = sg.get_local_range().size();	
	const uint ID              = subgroupSize - 1 - subgroupLocalID;  	
	AABB3f low_reverse         = high.sub_group_shuffle(sg,ID);
	AABB3f high_reverse        = low.sub_group_shuffle(sg,ID);
	AABB3f low_prefix          = low_reverse.sub_group_scan_inclusive_min_max(sg);
	AABB3f low_reduce          = low_reverse.sub_group_reduce(sg);	      	      
	AABB3f high_prefix         = high_reverse.sub_group_scan_inclusive_min_max(sg);
	high_prefix.extend(low_reduce);
	const float low_area       = sg.shuffle<float>(high_prefix.halfArea(),ID);
	const float high_area      = sg.shuffle<float>( low_prefix.halfArea(),ID);
	return float2(low_area,high_area);	
      }
    
    [[cl::intel_reqd_sub_group_size(BINS)]] inline uint2 right_to_left_counts32(const cl::sycl::intel::sub_group &sg, uint low, uint high)
      {
	const uint subgroupLocalID = sg.get_local_id()[0];
	const uint subgroupSize    = sg.get_local_range().size();	
	const uint ID              = subgroupSize - 1 - subgroupLocalID;  	
	const uint low_reverse     = sg.shuffle<uint>(high,ID);
	const uint high_reverse    = sg.shuffle<uint>(low,ID);  
	const uint low_prefix      = sg.inclusive_scan<uint>(low_reverse, cl::sycl::intel::plus<>());
	const uint low_reduce      = sg.reduce<uint>(low_reverse, cl::sycl::intel::plus<>());
	const uint high_prefix     = sg.inclusive_scan<uint>(high_reverse, cl::sycl::intel::plus<>()) + low_reduce;
	return uint2(sg.shuffle<uint>(high_prefix,ID),sg.shuffle<uint>(low_prefix,ID));	
      }

      [[cl::intel_reqd_sub_group_size(BINS)]] inline ulong getBestSplit(cl::sycl::intel::sub_group &sg, const float3 sah, uint ID, const float4 scale, const ulong defaultSplit)
      {
	ulong splitX = (((ulong)as_uint((float)sah.x())) << 32) | ((uint)ID << 2) | 0;
	ulong splitY = (((ulong)as_uint((float)sah.y())) << 32) | ((uint)ID << 2) | 1;
	ulong splitZ = (((ulong)as_uint((float)sah.z())) << 32) | ((uint)ID << 2) | 2;
	
	
	/* ignore zero sized dimensions */
	splitX = cselect( (ulong)(scale.x() == 0), defaultSplit, splitX);
	splitY = cselect( (ulong)(scale.y() == 0), defaultSplit, splitY);
	splitZ = cselect( (ulong)(scale.z() == 0), defaultSplit, splitZ);
	ulong bestSplit = min(min(splitX,splitY),splitZ);
	bestSplit = sg.reduce<ulong>(bestSplit, cl::sycl::intel::minimum<>());
	return bestSplit;
      }
    
    
    struct BinInfo {
      AABB3f boundsX[BINS];
      AABB3f boundsY[BINS];
      AABB3f boundsZ[BINS];
      uint3 counts[BINS];

      inline void init()
      {
	for (uint i=0;i<BINS;i++)
	  {
	    boundsX[i].init();
	    boundsY[i].init();
	    boundsZ[i].init();
	    counts[i] = (uint3)(0);
	  }	
      }

      [[cl::intel_reqd_sub_group_size(BINS)]] inline void init(const cl::sycl::intel::sub_group &subgroup)
      {
	for (uint i=subgroup.get_local_id()[0];i<BINS;i+=subgroup.get_local_range().size())
	  {
	    boundsX[i].init();
	    boundsY[i].init();
	    boundsZ[i].init();
	    counts[i] = (uint3)(0);
	  }
      }

      [[cl::intel_reqd_sub_group_size(BINS)]] inline void atomicUpdate( const gpu::BinMapping &binMapping, const gpu::AABB &primref)
      {
	const float4 p = primref.centroid2();
	const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
	const cl::sycl::uint4 i = bin4.convert<cl::sycl::uint,cl::sycl::rounding_mode::rtz>();
  
	gpu::AABB3f bounds = convert_AABB3f(primref);

	bounds.atomic_merge_local(boundsX[i.x()]);
	bounds.atomic_merge_local(boundsY[i.y()]);
	bounds.atomic_merge_local(boundsZ[i.z()]);

	gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&counts[i.x()].x(),1);	
	gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&counts[i.y()].y(),1);
	gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&counts[i.z()].z(),1);        
      }
      

      
      [[cl::intel_reqd_sub_group_size(BINS)]] inline gpu::Split reduceBinsAndComputeBestSplit16(cl::sycl::intel::sub_group &sg, const float4 scale, const uint startID, const uint endID)
      {
	const uint subgroupLocalID = sg.get_local_id()[0];		
	const AABB3f &bX      = boundsX[subgroupLocalID];
	const float lr_areaX  = left_to_right_area16(sg,bX);	
	const float rl_areaX  = right_to_left_area16(sg,bX);
	const AABB3f &bY      = boundsY[subgroupLocalID];
	const float lr_areaY  = left_to_right_area16(sg,bY);
	const float rl_areaY  = right_to_left_area16(sg,bY);
	const AABB3f &bZ      = boundsZ[subgroupLocalID];
	const float lr_areaZ  = left_to_right_area16(sg,bZ);
	const float rl_areaZ  = right_to_left_area16(sg,bZ);
	const uint3 &c = counts[subgroupLocalID];
	const uint lr_countsX = left_to_right_counts16(sg,c.x());
	const uint rl_countsX = right_to_left_counts16(sg,c.x());
	const uint lr_countsY = left_to_right_counts16(sg,c.y());
	const uint rl_countsY = right_to_left_counts16(sg,c.y());  
	const uint lr_countsZ = left_to_right_counts16(sg,c.z());
	const uint rl_countsZ = right_to_left_counts16(sg,c.z());
	
	const uint blocks_shift = SAH_LOG_BLOCK_SHIFT;       
	const uint blocks_add = ((1 << blocks_shift)-1);	
	const uint3 lr_count( (lr_countsX+blocks_add)>>blocks_shift , (lr_countsY+blocks_add)>>blocks_shift, (lr_countsZ+blocks_add)>>blocks_shift );       
	const float3 lr_area(lr_areaX,lr_areaY,lr_areaZ);
	const float3 rl_area(rl_areaX,rl_areaY,rl_areaZ);	
	const uint3 rl_count( (rl_countsX+blocks_add)>>blocks_shift , (rl_countsY+blocks_add)>>blocks_shift, (rl_countsZ+blocks_add)>>blocks_shift );
	const float3 lr_count_f ( (float)lr_count.x(),(float)lr_count.y(),(float)lr_count.z() );
	const float3 rl_count_f ( (float)rl_count.x(),(float)rl_count.y(),(float)rl_count.z() );	

	/* first bin is invalid */
	const float pos_inf = (float)INFINITY;
	const int m_subgroup = subgroupLocalID != 0;
	const float3 cost = cl::sycl::fma(lr_area,lr_count_f,rl_area*rl_count_f);
#if 1	// workaround for compiler bug
	float3 sah;
	sah.x() = cselect( m_subgroup, cost.x() , pos_inf);
	sah.y() = cselect( m_subgroup, cost.y() , pos_inf);
	sah.z() = cselect( m_subgroup, cost.z() , pos_inf);
#else	
	const float3 sah = cselect( int3(m_subgroup), cost , float3(pos_inf));
#endif
	
	/* select best split */
	const uint mid = (startID+endID)/2;
	const uint maxSAH = as_uint(pos_inf);	
	const ulong defaultSplit = (((ulong)maxSAH) << 32) | ((uint)mid << 2) | 0;    
	const ulong bestSplit = getBestSplit(sg, sah, subgroupLocalID, scale, defaultSplit);
	const uint bestSplit32 = (uint)(bestSplit >> 32);
	const float split_sah = as_float(bestSplit32);	
	return gpu::Split(split_sah,(uint)bestSplit & 3,(uint)bestSplit >> 2);
      }
            
    };

    struct BinInfo2 {
      AABB3f boundsX[BINS*2];
      AABB3f boundsY[BINS*2];
      AABB3f boundsZ[BINS*2];
      uint3 counts[BINS*2];

      [[cl::intel_reqd_sub_group_size(BINS)]] inline void init(const cl::sycl::intel::sub_group &subgroup)
      {
	for (uint i=subgroup.get_local_id()[0];i<BINS*2;i+=subgroup.get_local_range().size())
	  {
	    boundsX[i].init();
	    boundsY[i].init();
	    boundsZ[i].init();
	    counts[i] = (uint3)(0);
	  }
      }

      [[cl::intel_reqd_sub_group_size(BINS)]] inline void atomicUpdate( const gpu::BinMapping &binMapping, const gpu::AABB &primref,const cl::sycl::stream &out)
      {
	const float4 p = primref.centroid2();
	const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
	const cl::sycl::uint4 i = bin4.convert<cl::sycl::uint,cl::sycl::rounding_mode::rtz>();
  
	gpu::AABB3f bounds = convert_AABB3f(primref);

	bounds.atomic_merge_local(boundsX[i.x()]);
	bounds.atomic_merge_local(boundsY[i.y()]);
	bounds.atomic_merge_local(boundsZ[i.z()]);

	gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&counts[i.x()].x(),1);
	gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&counts[i.y()].y(),1);
	gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&counts[i.z()].z(),1);        
      }
      
      [[cl::intel_reqd_sub_group_size(BINS)]] inline gpu::Split reduceBinsAndComputeBestSplit32(cl::sycl::intel::sub_group &sg, const float4 scale, const uint startID, const uint endID)
      {
	const uint subgroupLocalID = sg.get_local_id()[0];		
	const uint subgroup_size   = BINS;  

	AABB3f boundsX_low  = boundsX[subgroupLocalID              ];
	AABB3f boundsX_high = boundsX[subgroupLocalID+subgroup_size];

	const float2 lr_areaX = left_to_right_area32(sg,boundsX_low,boundsX_high);
	const float2 rl_areaX = right_to_left_area32(sg,boundsX_low,boundsX_high);
  
	AABB3f boundsY_low  = boundsY[subgroupLocalID              ];
	AABB3f boundsY_high = boundsY[subgroupLocalID+subgroup_size];

	const float2 lr_areaY = left_to_right_area32(sg,boundsY_low,boundsY_high);
	const float2 rl_areaY = right_to_left_area32(sg,boundsY_low,boundsY_high);
  
	AABB3f boundsZ_low  = boundsZ[subgroupLocalID              ];
	AABB3f boundsZ_high = boundsZ[subgroupLocalID+subgroup_size];

	const float2 lr_areaZ = left_to_right_area32(sg,boundsZ_low,boundsZ_high);
	const float2 rl_areaZ = right_to_left_area32(sg,boundsZ_low,boundsZ_high);
  
	const uint3 counts_low  = counts[subgroupLocalID              ];
	const uint3 counts_high = counts[subgroupLocalID+subgroup_size];
	
	const uint2 lr_countsX = left_to_right_counts32(sg,counts_low.x(),counts_high.x());
	const uint2 rl_countsX = right_to_left_counts32(sg,counts_low.x(),counts_high.x());
	const uint2 lr_countsY = left_to_right_counts32(sg,counts_low.y(),counts_high.y());
	const uint2 rl_countsY = right_to_left_counts32(sg,counts_low.y(),counts_high.y());  
	const uint2 lr_countsZ = left_to_right_counts32(sg,counts_low.z(),counts_high.z());
	const uint2 rl_countsZ = right_to_left_counts32(sg,counts_low.z(),counts_high.z());
  
	const uint blocks_shift = SAH_LOG_BLOCK_SHIFT;  
	uint3 blocks_add = (uint3)((1 << blocks_shift)-1);
	const float pos_inf = (float)INFINITY;

	/* low part: bins 0..15 */
	const float3 lr_area_low(lr_areaX.x(),lr_areaY.x(),lr_areaZ.x());
	const float3 rl_area_low(rl_areaX.x(),rl_areaY.x(),rl_areaZ.x());
	const uint3 lr_count_low = (uint3(lr_countsX.x(),lr_countsY.x(),lr_countsZ.x())+blocks_add) >> blocks_shift;
	const uint3 rl_count_low = (uint3(rl_countsX.x(),rl_countsY.x(),rl_countsZ.x())+blocks_add) >> blocks_shift;
	
	const float3 lr_count_low_f = float3( (float)lr_count_low.x(),(float)lr_count_low.y(),(float)lr_count_low.z() );
	const float3 rl_count_low_f = float3( (float)rl_count_low.x(),(float)rl_count_low.y(),(float)rl_count_low.z() );

	float3 sah_low     = cl::sycl::fma(lr_area_low,lr_count_low_f,rl_area_low*rl_count_low_f);

	/* first bin is invalid */
	const int m_subgroup = subgroupLocalID != 0;	
#if 1 // workaround for compiler bug
	sah_low.x() = cselect( m_subgroup, sah_low.x(), pos_inf);
	sah_low.y() = cselect( m_subgroup, sah_low.y(), pos_inf);
	sah_low.z() = cselect( m_subgroup, sah_low.z(), pos_inf);
#else	
	sah_low = cselect( int3(m_subgroup), sah_low, float3(pos_inf));
#endif	  
	/* high part: bins 16..31 */
	const float3 lr_area_high(lr_areaX.y(),lr_areaY.y(),lr_areaZ.y());
	const float3 rl_area_high(rl_areaX.y(),rl_areaY.y(),rl_areaZ.y());
	const uint3 lr_count_high = (uint3(lr_countsX.y(),lr_countsY.y(),lr_countsZ.y())+blocks_add) >> blocks_shift;
	const uint3 rl_count_high = (uint3(rl_countsX.y(),rl_countsY.y(),rl_countsZ.y())+blocks_add) >> blocks_shift;
	const float3 lr_count_high_f = float3( (float)lr_count_high.x(),(float)lr_count_high.y(),(float)lr_count_high.z() );
	const float3 rl_count_high_f = float3( (float)rl_count_high.x(),(float)rl_count_high.y(),(float)rl_count_high.z() );	
	const float3 sah_high        = fma(lr_area_high,lr_count_high_f,rl_area_high*rl_count_high_f);

	const uint mid = (startID+endID)/2;
	const ulong defaultSplit = (((ulong)as_uint((float)(INFINITY))) << 32) | ((uint)mid << 2) | 0;  
  
	const ulong bestSplit_low  = getBestSplit(sg, sah_low ,subgroupLocalID,scale,defaultSplit);
	const ulong bestSplit_high = getBestSplit(sg, sah_high,subgroupLocalID+subgroup_size,scale,defaultSplit);
	const ulong bestSplit = min(bestSplit_low,bestSplit_high);
	const uint bestSplit32 = (uint)(bestSplit >> 32);
	const float split_sah = as_float(bestSplit32);	
	return gpu::Split(split_sah,(uint)bestSplit & 3,(uint)bestSplit >> 2);
      }
      
    };

    inline uint encodeOffset(char *bvh_mem, uint *parent, uint global_child_offset)
    {
      ulong global_parent_offset = (ulong)parent - (ulong)bvh_mem;
      // parent node address starts at least on 64 bytes boundary      
      global_parent_offset = global_parent_offset & (~63); 
      uint relative_offset = global_child_offset - global_parent_offset;
      return relative_offset;
    }

    inline uint createLeaf(const Globals &globals,		 
			   const uint start,
			   const uint items)
    {	
      const uint offset = globals.leaf_mem_allocator_start + start * globals.leafSize;
      const unsigned int final = offset | BVH_LEAF_MASK | (items-1);
      return final;
    }

    
    inline uint createNode(cl::sycl::intel::sub_group &subgroup, Globals &globals, const uint ID, struct AABB *childrenAABB, uint numChildren, char *bvh_mem, const cl::sycl::stream &out)
    {
      const uint subgroupLocalID = subgroup.get_local_id()[0];

      uint node_offset = 0;
      if (subgroupLocalID == 0)
	{
	  node_offset = globals.alloc_node_mem(sizeof(gpu::QBVHNodeN));
	  DBG_BUILD(out << "node offset " << node_offset << cl::sycl::endl);
	}
      node_offset = subgroup.broadcast<uint>(node_offset,0); 

      gpu::QBVHNodeN &node = *(gpu::QBVHNodeN*)(bvh_mem + node_offset);
      gpu::QBVHNodeN::init(subgroup,node,childrenAABB,numChildren,ID,out);
      return node_offset;      
    }


    inline bool checkPrimRefBounds(const BuildRecord &record, const AABB &geometryBounds, const AABB &primref)
    {
      const float4 centroid2 = primref.lower+primref.upper;

      if (centroid2.x() < record.centroidBounds.lower.x()) return false;
      if (centroid2.y() < record.centroidBounds.lower.y()) return false;
      if (centroid2.z() < record.centroidBounds.lower.z()) return false;

      if (centroid2.x() > record.centroidBounds.upper.x()) return false;
      if (centroid2.y() > record.centroidBounds.upper.y()) return false;
      if (centroid2.z() > record.centroidBounds.upper.z()) return false;

      if (primref.lower.x() < geometryBounds.lower.x()) return false;
      if (primref.lower.y() < geometryBounds.lower.y()) return false;
      if (primref.lower.z() < geometryBounds.lower.z()) return false;

      if (primref.upper.x() > geometryBounds.upper.x()) return false;
      if (primref.upper.y() > geometryBounds.upper.y()) return false;
      if (primref.upper.z() > geometryBounds.upper.z()) return false;
 
      return true;  
    }
    

     const uint shuffle2mirror[16]  = { 1,0,3,2,5,4,7,6,9,8,11,10,13,12,15,14 };
     const uint shuffle4mirror[16]  = { 3,2,1,0,7,6,5,4,11,10,9,8,15,14,13,12 };
     const uint shuffle4rotate[16]  = { 2,3,0,1,6,7,4,5,10,11,8,9,14,15,12,13 };
     const uint shuffle8mirror[16]  = { 7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8 };
     const uint shuffle8rotate[16]  = { 4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11 };
     const uint shuffle16mirror[16] = { 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0 };

     const uint selGo2[16]  = { 0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1 };
     const uint selGo4[16]  = { 0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1 };
     const uint selGo8[16]  = { 0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1 };
     const uint selGo16[16] = { 0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1 };
     
     inline uint compare_exchange(const cl::sycl::intel::sub_group &sg, const uint a0, const uint shuffleMask, const uint selectMask, const bool ascending)
     {
       const uint a1 = sg.shuffle(a0,shuffleMask);
       const uint a_min = min(a0,a1);
       const uint a_max = max(a0,a1);
       return cl::sycl::select(ascending ? a_min : a_max,ascending ? a_max : a_min,selectMask);
     }

     inline uint sort16(const cl::sycl::intel::sub_group &sg, const uint aa, const bool ascending)
     { 
       const uint slotID = sg.get_local_id()[0];		
       const uint bb = compare_exchange(sg,aa,shuffle2mirror[slotID],selGo2[slotID],ascending);
       const uint cc = compare_exchange(sg,bb,shuffle4mirror[slotID],selGo4[slotID],ascending);
       const uint dd = compare_exchange(sg,cc,shuffle2mirror[slotID],selGo2[slotID],ascending);
       const uint ee = compare_exchange(sg,dd,shuffle8mirror[slotID],selGo8[slotID],ascending);
       const uint ff = compare_exchange(sg,ee,shuffle4rotate[slotID],selGo4[slotID],ascending);
       const uint gg = compare_exchange(sg,ff,shuffle2mirror[slotID],selGo2[slotID],ascending);
       const uint hh = compare_exchange(sg,gg,shuffle16mirror[slotID],selGo16[slotID],ascending);
       const uint ii = compare_exchange(sg,hh,shuffle8rotate[slotID],selGo8[slotID],ascending);
       const uint jj = compare_exchange(sg,ii,shuffle4rotate[slotID],selGo4[slotID],ascending);
       const uint kk = compare_exchange(sg,jj,shuffle2mirror[slotID],selGo2[slotID],ascending);
       return kk;
       
     }

     inline uint sort8(const cl::sycl::intel::sub_group &sg, const uint aa, const bool ascending)
     {
       const uint slotID = sg.get_local_id()[0];		
       const uint bb = compare_exchange(sg,aa,shuffle2mirror[slotID],selGo2[slotID],ascending);
       const uint cc = compare_exchange(sg,bb,shuffle4mirror[slotID],selGo4[slotID],ascending);
       const uint dd = compare_exchange(sg,cc,shuffle2mirror[slotID],selGo2[slotID],ascending);
       const uint ee = compare_exchange(sg,dd,shuffle8mirror[slotID],selGo8[slotID],ascending);
       const uint ff = compare_exchange(sg,ee,shuffle4rotate[slotID],selGo4[slotID],ascending);
       const uint gg = compare_exchange(sg,ff,shuffle2mirror[slotID],selGo2[slotID],ascending);
       return gg;
     }

     inline uint sort4(const cl::sycl::intel::sub_group &sg, const uint aa, const bool ascending)
     {
       const uint slotID = sg.get_local_id()[0];		
       const uint bb = compare_exchange(sg,aa,shuffle2mirror[slotID],selGo2[slotID],ascending);
       const uint cc = compare_exchange(sg,bb,shuffle4mirror[slotID],selGo4[slotID],ascending);
       const uint dd = compare_exchange(sg,cc,shuffle2mirror[slotID],selGo2[slotID],ascending);
       return dd;
     }

     inline uint sortBVHChildrenIDs(const cl::sycl::intel::sub_group &sg, uint input)
     {
#if BVH_NODE_N == 16
       return sort16(sg, input,false);
#elif BVH_NODE_N == 8
       return sort8(sg, input,false);
#else
       return sort4(sg, input,false);  
#endif
     }

    
  };
};

#endif
