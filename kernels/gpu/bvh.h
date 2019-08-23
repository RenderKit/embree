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

#define BINS 16
#define BVH_LEAF_MASK        8
#define BVH_INVALID_NODE     3
#define BVH_NODE_N          16
#define BVH_NODE_N_LOG       4

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

    /* ======================================================================== */
    /* ============================== BVH NODES =============================== */
    /* ======================================================================== */

    struct BVHNodeN
    {              
      uint offset[BVH_NODE_N];  
      uint parent[BVH_NODE_N]; 
      float lower_x[BVH_NODE_N]; 
      float upper_x[BVH_NODE_N]; 
      float lower_y[BVH_NODE_N]; 
      float upper_y[BVH_NODE_N]; 
      float lower_z[BVH_NODE_N]; 
      float upper_z[BVH_NODE_N]; 

      inline void initBVHNodeN(uint slotID)
      {
	const float pos_inf =  INFINITY;
	const float neg_inf = -INFINITY;	
	offset[slotID]  =  (uint)(-1);  
	parent[slotID]  =  (uint)(-1); 
	lower_x[slotID] =  pos_inf; 
	upper_x[slotID] =  neg_inf;
	lower_y[slotID] =  pos_inf; 
	upper_y[slotID] =  neg_inf;
	lower_z[slotID] =  pos_inf; 
	upper_z[slotID] =  neg_inf;  
      }


      inline void setBVHNodeN(const struct AABB &aabb, uint slot)
      {
	lower_x[slot] = aabb.lower.x();
	lower_y[slot] = aabb.lower.y();
	lower_z[slot] = aabb.lower.z();
	upper_x[slot] = aabb.upper.x();
	upper_y[slot] = aabb.upper.y();
	upper_z[slot] = aabb.upper.z();
      }

      inline void setBVHNodeN_offset(const struct AABB &aabb, const uint _offset, const uint _parent, uint slot)
      {
	offset[slot] = _offset;
	parent[slot] = _parent;  
	lower_x[slot] = aabb.lower.x();
	lower_y[slot] = aabb.lower.y();
	lower_z[slot] = aabb.lower.z();
	upper_x[slot] = aabb.upper.x();
	upper_y[slot] = aabb.upper.y();
	upper_z[slot] = aabb.upper.z();
      }


    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const BVHNodeN& node) {
      for (uint i=0;i<BVH_NODE_N;i++)
	{
	  out << " i " << i << " offset " << node.offset[i] << " lower_x " << node.lower_x[i] << " upper_x " << node.upper_x[i] << " lower_y " << node.lower_y[i] << " upper_y " << node.upper_y[i] << " lower_z " << node.lower_z[i] << " upper_z " << node.upper_z[i];
	}      
      return out; 
    }
    
  };
};

#endif
