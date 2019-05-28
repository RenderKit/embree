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

#define BUILDRECORD_STACK_SIZE 48
#define BINS 16

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
      void *current;
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
    };

    struct BinInfo {
      struct AABB3f boundsX[BINS];
      struct AABB3f boundsY[BINS];
      struct AABB3f boundsZ[BINS];
      cl::sycl::uint3 counts[BINS];
    };

    struct BinInfo2 {
      struct AABB3f boundsX[BINS*2];
      struct AABB3f boundsY[BINS*2];
      struct AABB3f boundsZ[BINS*2];
      cl::sycl::uint3 counts[BINS*2];
    };

  };
};

#endif
