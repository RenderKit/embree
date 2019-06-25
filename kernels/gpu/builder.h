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

#include "../common/device.h"

#if defined(EMBREE_DPCPP_SUPPORT)

#include <CL/sycl.hpp>

#include "AABB.h"
#include "AABB3f.h"

#define BUILDRECORD_STACK_SIZE 64
#define BINS 16

#define BVH_LEAF_MASK       8
#define BVH_INVALID_NODE    3
#define BVH_NODE_N           16
#define BVH_NODE_N_LOG       4
#define SAH_LOG_BLOCK_SHIFT  2
#define BVH_LEAF_N_MIN       4
#define BVH_LEAF_N_MAX       4


namespace embree
{
  namespace gpu
  {
    
    struct BVHBase
    {
      unsigned long rootNodeOffset; 
      AABB3f bounds;

      unsigned int nodeDataStart;
      unsigned int nodeDataCur;
      unsigned int leafDataStart;
      unsigned int leafDataCur;
      unsigned int proceduralDataStart;
      unsigned int proceduralDataCur;
      unsigned int backPointerDataStart;
      unsigned int backPointerDataEnd;
    };
    
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
      unsigned int procedural_mem_allocator_start;
      unsigned int procedural_mem_allocator_cur;
      unsigned int back_pointer_start;
      unsigned int leaf_mem_allocator[11];

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
      unsigned int numGlobalBinaryNodes;
      unsigned int counter;

      /* morton code builder state */
      unsigned int shift;                // used by adaptive mc-builder
      unsigned int shift_mask;           // used by adaptive mc-builder
      unsigned int binary_hierarchy_root;

      inline void init(char *bvh_mem,
		       unsigned int _numPrimitives,
		       unsigned int _node_data_start,
		       unsigned int _leaf_data_start,
		       unsigned int _procedural_data_start,
		       unsigned int _back_pointer_start,
		       unsigned int _totalBytes,
		       unsigned int _leafPrimType,
		       unsigned int _leafSize)
      {
	struct BVHBase *base       = (struct BVHBase*)bvh_mem;
	base->nodeDataStart        = _node_data_start/64;
	base->nodeDataCur          = _node_data_start/64;
	base->leafDataStart        = _leaf_data_start/64;
	base->leafDataCur          = _leaf_data_start/64;
	base->proceduralDataStart  = _procedural_data_start/64;
	base->proceduralDataCur    = _procedural_data_start/64;
	base->backPointerDataStart = _back_pointer_start/64;
	base->backPointerDataEnd   = _totalBytes/64;
	base->rootNodeOffset       = _node_data_start; // FIXME: should be set by builder

	geometryBounds.init();
	centroidBounds.init();

	node_mem_allocator_cur         = _node_data_start;
	node_mem_allocator_start       = _node_data_start;
	leaf_mem_allocator_cur         = _leaf_data_start;
	leaf_mem_allocator_start       = _leaf_data_start;
	procedural_mem_allocator_cur   = _procedural_data_start;
	procedural_mem_allocator_start = _procedural_data_start;
	back_pointer_start             = _back_pointer_start;

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

      inline void init(char *bvh_mem,
		       unsigned int _numPrimitives,
		       unsigned int _node_data_start,
		       unsigned int _leaf_data_start,
		       unsigned int _totalBytes)
      {
	init(bvh_mem,_numPrimitives,_node_data_start,_leaf_data_start,0,0,_totalBytes,0,64);
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
	const uint aligned_size = ((size+63)/64)*64; /* allocate in 64 bytes blocks */
	cl::sycl::multi_ptr<unsigned int,cl::sycl::access::address_space::global_space> ptr(&leaf_mem_allocator_cur);
	cl::sycl::atomic<unsigned int> counter(ptr);
	return atomic_fetch_add(counter,aligned_size);
      }
      
    };

    struct Range {
      unsigned int start, end;
    };

    struct Triangle
    {
      unsigned int vtx[3];
      //unsigned int primID;
      //unsigned int geomID;
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

      inline void extend(AABB &primref)
      {
	centroidBounds.extend(primref.centroid2());
      }

      inline unsigned int size() { return end - start; }

      inline void print()
      {
	printf("buildrecord: start %d end %d size %d parent %p \n",start,end,size(),parent);
	centroidBounds.print();
      }
    };

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
      int dim;
      int pos;
    };

    struct BinMapping
    {
      cl::sycl::float4 ofs, scale;

      inline void init(const AABB &centBounds, const uint bins)
      {
	const cl::sycl::float4 eps(1E-34f);
	const cl::sycl::float4 diag = max(eps, centBounds.upper - centBounds.lower);
	scale = (cl::sycl::float4)(0.99f*(float)bins)/diag;
	scale = select((cl::sycl::float4)(0.0f), scale, (diag > eps));
	ofs  = centBounds.lower;
      }

    };

    inline AABB3f convert_AABB3f(const AABB &aabb)
    {
      AABB3f aabb3f;
      aabb3f.lower.x() = aabb.lower.x();
      aabb3f.lower.y() = aabb.lower.y();
      aabb3f.lower.z() = aabb.lower.z();      
      aabb3f.upper.x() = aabb.upper.x();
      aabb3f.upper.y() = aabb.upper.y();
      aabb3f.upper.z() = aabb.upper.z();
      return aabb3f;
    }
    
    struct BinInfo {
      struct AABB3f boundsX[BINS];
      struct AABB3f boundsY[BINS];
      struct AABB3f boundsZ[BINS];
      cl::sycl::uint3 counts[BINS];

      inline void init()
      {
	for (uint i=0;i<BINS;i++)
	  {
	    boundsX[i].init();
	    boundsY[i].init();
	    boundsZ[i].init();
	    counts[i] = (cl::sycl::uint3)(0);
	  }	
      }

      inline void init(const cl::sycl::intel::sub_group &subgroup)
      {
	printf("subgroup.get_local_range().size() %d \n",(uint)subgroup.get_local_range().size());
	for (uint i=subgroup.get_local_id()[0];i<BINS;i+=subgroup.get_local_range().size())
	  {
	    boundsX[i].init();
	    boundsY[i].init();
	    boundsZ[i].init();
	    counts[i] = (cl::sycl::uint3)(0);
	  }	
      }
      
      
    };

    struct BinInfo2 {
      struct AABB3f boundsX[BINS*2];
      struct AABB3f boundsY[BINS*2];
      struct AABB3f boundsZ[BINS*2];
      cl::sycl::uint3 counts[BINS*2];
    };

    inline uint encodeOffset(char *bvh_mem, uint *parent, uint global_child_offset)
    {
      ulong global_parent_offset = (ulong)parent - (ulong)bvh_mem;
      global_parent_offset = global_parent_offset & (~(64-1));
      uint relative_offset = global_child_offset - global_parent_offset;
      return relative_offset;
    }

    inline uint createLeaf(const Globals &globals,		 
			   const uint start,
			   const uint items,
			   const uint stride)
    {
      const uint offset = globals.leaf_mem_allocator[1] + start * stride;
      const unsigned int final = offset | BVH_LEAF_MASK | (items-1);
      return final;
    }

    struct Quad1
    {
      cl::sycl::float4 v0,v2,v1,v3; //v1v3 loaded once
    };


    template<cl::sycl::access::address_space space>
    inline uint atomic_add_uint(uint *dest, const uint count=1)
    {
      cl::sycl::multi_ptr<unsigned int,space> ptr(dest);
      cl::sycl::atomic<unsigned int> counter(ptr);
      return atomic_fetch_add(counter,count);      
    }
    
  };
};

#endif
