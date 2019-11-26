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
#define DBG_GPU_TRAV(x) 
#define TSTATS(x) 

namespace embree
{

#if defined(EMBREE_DPCPP_SUPPORT)   

  static const uint max_uint  = 0xffffffff;  
  static const uint mask_uint = 0xfffffff0;

  template<typename T>
    __forceinline T wg_align(T x, unsigned int alignment)
    {
      return ((x + alignment-1)/alignment)*alignment;
    }  
  
  
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


  template<typename BVHNodeType, typename Primitive>
    [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void traceRayBVH16(const cl::sycl::intel::sub_group &sg,
									    const uint m_active,
									    gpu::RTCRayGPU &ray,
									    gpu::RTCHitGPU &hit,
									    void *bvh_mem,
									    TraversalStats *tstats)
  {
    unsigned int stack_offset[BVH_MAX_STACK_ENTRIES]; 
    float        stack_dist[BVH_MAX_STACK_ENTRIES];  

    // === init local hit ===
    gpu::RTCHitGPU local_hit;

    const uint subgroupLocalID  = sg.get_local_id()[0];
    DBG_GPU_TRAV(const uint subgroupSize = sg.get_local_range().size());

    /* cannot handle masked control flow yet */
    uint m_activeLanes = m_active;
    m_activeLanes = cselect((uint)(m_activeLanes == (uint)(1<<BVH_NODE_N)-1),m_activeLanes,(uint)0);

    const float3 org16   = ray.org();
    const float3 dir16   = ray.dir();
    const float  tnear16 = ray.tnear;      
    const float  tfar16  = ray.tfar;
#pragma nounroll      
    while(m_activeLanes)
      {
	const uint rayID = cl::sycl::intel::ctz(m_activeLanes);
	m_activeLanes &= m_activeLanes-1;
	  
	local_hit.init();	  
	const float3 org(sg.broadcast<float>(org16.x(),rayID),sg.broadcast<float>(org16.y(),rayID),sg.broadcast<float>(org16.z(),rayID));	  
	const float3 dir(sg.broadcast<float>(dir16.x(),rayID),sg.broadcast<float>(dir16.y(),rayID),sg.broadcast<float>(dir16.z(),rayID));	  
            
	const float tnear = sg.broadcast<float>(tnear16,rayID);
	float tfar        = sg.broadcast<float>(tfar16,rayID);
	float hit_tfar    = tfar;
	        
	const uint3 dir_mask = cselect(dir >= 0.0f,uint3(0),uint3(1));
	const float3 new_dir = cselect(dir != 0.0f, dir, float3(1E-18f));
	const float3 inv_dir = cl::sycl::native::recip(new_dir);
	const float3 inv_dir_org = -inv_dir * org; 
            
	const char *const bvh_base = (char*)bvh_mem;
	stack_offset[0] = max_uint; // sentinel
	stack_dist[0]   = -(float)INFINITY;
	stack_offset[1] = sizeof(struct gpu::BVHBase); // single node after bvh start 
	stack_dist[1]   = -(float)INFINITY;

	unsigned int sindex = 2;

	TSTATS(tstats->nrays_inc());

	while(1)
	  {
	    /* stack pop */
	    sindex--;

#if STACK_CULLING  == 1    
	    if (stack_dist[sindex] > tfar) continue;
#endif
	  
	    gpu::NodeRef cur = stack_offset[sindex]; 

	    /* BVH down traversal */
	    while(!cur.isLeaf()) 
	      {
		TSTATS(tstats->tsteps_inc());	      
		const BVHNodeType &node = *(BVHNodeType*)(bvh_base + cur);
		const gpu::NodeIntersectionData isec = intersectNode(sg,node,dir_mask,inv_dir,inv_dir_org,tnear,tfar);
		getClosestChildNode(sg,isec,cur,sindex,tfar,stack_offset,stack_dist);
	      }

	    /* stack empty */
	    if (cur == max_uint) break; // sentinel reached -> exit

	    /* leaf intersection */
	    const uint numPrims = cur.getNumLeafPrims();
	    const uint leafOffset = cur.getLeafOffset();    

	    const Primitive *const prim = (Primitive *)(bvh_base + leafOffset);
	    TSTATS(tstats->isteps_inc());	  
	    hit_tfar = intersectPrimitive1v(sg, prim, numPrims, org, dir, tnear, hit_tfar, local_hit, subgroupLocalID);

	    /* update tfar */
	    tfar = sg.reduce<float>(hit_tfar, cl::sycl::intel::minimum<float>());
	  }

	DBG_GPU_TRAV(
	    for (uint i=0;i<subgroupSize;i++)
	      if (i == subgroupLocalID)
		out << "i " << i << " local_hit " << local_hit << " tfar " << tfar << " hit_tfar " << hit_tfar << cl::sycl::endl;
	    );

	/* select hit with shortest intersection distance */
      
	const uint index = cl::sycl::intel::ctz(intel_sub_group_ballot(tfar == hit_tfar));
	local_hit.broadcast(sg,index);
	  
	if (subgroupLocalID == rayID)
	  if (local_hit.primID != -1)
	    {	 
	      hit = local_hit;
	      ray.tfar = tfar;
	    }
      }
  }
		       
#endif
    
};

