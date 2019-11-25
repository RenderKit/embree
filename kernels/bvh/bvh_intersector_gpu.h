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

#include "bvh_traverser_stream.h"

#if defined(EMBREE_DPCPP_SUPPORT)
#include "../gpu/bvh.h"
#include "../gpu/ray.h"
#include "../gpu/geometry.h"
#endif

#define STACK_CULLING 1

namespace embree
{

#if defined(EMBREE_DPCPP_SUPPORT)   

  static const uint max_uint  = 0xffffffff;  
  static const uint mask_uint = 0xfffffff0;

  
    struct TraversalStats
    {
      uint nrays;
      uint tsteps;
      uint isteps;
      
      inline void reset() {
	nrays  = 0;
	tsteps = 0;
	isteps = 0;
      }

      inline void nrays_inc()  { gpu::atomic_add<uint,cl::sycl::access::address_space::global_space>(&nrays,1); }
      inline void tsteps_inc() { gpu::atomic_add<uint,cl::sycl::access::address_space::global_space>(&tsteps,1); }
      inline void isteps_inc() { gpu::atomic_add<uint,cl::sycl::access::address_space::global_space>(&isteps,1); }      
    };

    inline std::ostream &operator<<(std::ostream &cout, const TraversalStats& s) {
      return cout << "tsteps/ray " << (float)s.tsteps / s.nrays << " isteps/ray " << (float)s.isteps / s.nrays;
    };

    inline void getClosestChildNode(const cl::sycl::intel::sub_group &sg,
				    const gpu::NodeIntersectionData &isec,
				    gpu::NodeRef &cur,
				    uint &sindex,
				    const float tfar,
				    unsigned int stack_offset[BVH_MAX_STACK_ENTRIES],
				    float stack_dist[BVH_MAX_STACK_ENTRIES])
    {
      const uint subgroupLocalID  = sg.get_local_id()[0];      
      const float fnear = isec.dist;
      const uint valid  = isec.valid;
      uint offset       = isec.offset;
      const uint mask   = intel_sub_group_ballot(valid);
	      
      if (mask == 0)
	{
#if STACK_CULLING == 1
	  do { 
	    sindex--;
	  } while (stack_dist[sindex] > tfar);
#else		  
	  sindex--;
#endif		  
	  cur = stack_offset[sindex];
	  return;
	}

      offset += cur; /* relative encoding */
      const uint popc = cl::sycl::popcount(mask); 
      cur = sg.broadcast<uint>(offset, cl::sycl::intel::ctz(mask));
	      	      
      if (popc == 1) return; // single hit only
      int t = (gpu::as_int(fnear) & mask_uint) | subgroupLocalID;  // make the integer distance unique by masking off the least significant bits and adding the slotID
      t = valid ? t : max_uint; // invalid slots set to MIN_INT, as we sort descending;	
	      
      for (uint i=0;i<popc-1;i++)
	{
	  const int t_max = sg.reduce<int>(t, cl::sycl::intel::maximum<int>()); // from larger to smaller distance
	  t = (t == t_max) ? max_uint : t;
	  const uint index = t_max & (~mask_uint);
	  stack_offset[sindex] = sg.broadcast<uint> (offset,index);
	  stack_dist[sindex]   = sg.broadcast<float>(fnear,index);
	  sindex++;
	}
      const int t_max = sg.reduce<int>(t, cl::sycl::intel::maximum<int>()); // from larger to smaller distance
      cur = sg.broadcast<uint>(offset,t_max & (~mask_uint));      
    }
		       
#endif
    
};

