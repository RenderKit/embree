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

#include "bvh_intersector_gpu.h"

#define DBG(x) 
#define TSTATS(x) 

#define STACK_CULLING 1

namespace embree
{

#if defined(EMBREE_DPCPP_SUPPORT)   
    
    template<typename Primitive>
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
      DBG(const uint subgroupSize = sg.get_local_range().size());

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
		  const gpu::QBVHNodeN &node = *(gpu::QBVHNodeN*)(bvh_base + cur);
		  const gpu::NodeIntersectionData isec = intersectQBVHNodeN(sg,node,dir_mask,inv_dir,inv_dir_org,tnear,tfar);
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

	  DBG(
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

  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]]  SYCL_EXTERNAL void rtcIntersectGPUTest(cl::sycl::intel::sub_group &sg,
											cl::sycl::global_ptr<RTCSceneTy> scene,
											struct RTCRayHit &rtc_rayhit,
											ulong ext_fct)
  {
    size_t *scene_data = (size_t*)scene.get();
    void *bvh_root = (void*)scene_data[2]; // root node is at 16 bytes offset    
    gpu::RTCRayGPU &ray = static_cast<gpu::RTCRayGPU&>(rtc_rayhit.ray);
    gpu::RTCHitGPU &hit = static_cast<gpu::RTCHitGPU&>(rtc_rayhit.hit);
    uint m_active = intel_sub_group_ballot(true);
    traceRayBVH16<gpu::Triangle1v>(sg,m_active,ray,hit,bvh_root,nullptr);
    //uint (*testfct)(uint, uint) = reinterpret_cast<uint (*)(uint, uint)>(ext_fct);
    //hit.primID = testfct(hit.primID,hit.primID); 
  }

  template<typename T>
  __forceinline T wg_align(T x, unsigned int alignment)
  {
    return ((x + alignment-1)/alignment)*alignment;
  }  


#endif
  
  namespace isa
  {
    /*! BVH ray stream GPU intersector */
    template<typename Primitive>
    class BVHNGPUIntersectorStream
    {
      typedef BVHN<4> BVH;

    public:
      static void intersect(Accel::Intersectors* This, RayHitN** inputRays, size_t numRays, IntersectContext* context);
      static void occluded (Accel::Intersectors* This, RayN** inputRays, size_t numRays, IntersectContext* context);

    };       

    template<typename Primitive>    
    void BVHNGPUIntersectorStream<Primitive>::intersect(Accel::Intersectors* This, RayHitN** _inputRays, size_t numRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;      
      if (bvh->root == BVH::emptyNode) return;
      
#if defined(EMBREE_DPCPP_SUPPORT)
            
      gpu::RTCRayHitGPU* inputRays = (gpu::RTCRayHitGPU*)_inputRays;
      void *bvh_mem = (void*)(size_t)(bvh->root);
      assert( sizeof(gpu::RTCRayHitGPU) == sizeof(RTCRayHit) );      
      DBG(numRays = 1);      
      DeviceGPU* deviceGPU = (DeviceGPU*)bvh->device;
      cl::sycl::queue &gpu_queue = deviceGPU->getGPUQueue();

      TraversalStats *tstats = nullptr;
      TSTATS(tstats = (TraversalStats *)cl::sycl::aligned_alloc(64,sizeof(TraversalStats),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),cl::sycl::usm::alloc::shared));
      TSTATS(tstats->reset());

      {
	cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
	    const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(wg_align(numRays,BVH_NODE_N)),cl::sycl::range<1>(BVH_NODE_N));
	    
	    cgh.parallel_for<class trace_ray_stream>(nd_range,[=](cl::sycl::nd_item<1> item) {
		const uint globalID   = item.get_global_id(0);
		cl::sycl::intel::sub_group sg = item.get_sub_group();
		//if (globalID < numRays)
		  {
		    uint m_activeLanes = intel_sub_group_ballot(globalID < numRays);
		    if (m_activeLanes == 0xffff)
		      traceRayBVH16<Primitive>(sg,m_activeLanes,inputRays[globalID].ray,inputRays[globalID].hit,bvh_mem,tstats);
		  }
	      });		  
	  });
	try {
	  gpu_queue.wait_and_throw();
	} catch (cl::sycl::exception const& e) {
	  std::cout << "Caught synchronous SYCL exception:\n"
		    << e.what() << std::endl;
	  FATAL("OpenCL Exception");
	}
      TSTATS(cl::sycl::free(tstats,deviceGPU->getGPUContext()););
      TSTATS(std::cout << "RAY TRAVERSAL STATS: " << *tstats << std::endl);
      }
      DBG(exit(0));
#endif      
    }

    template<typename Primitive>        
    void BVHNGPUIntersectorStream<Primitive>::occluded (Accel::Intersectors* This, RayN** inputRays, size_t numRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;      
      if (bvh->root == BVH::emptyNode) return;
      
    }

#if defined(EMBREE_DPCPP_SUPPORT)

    typedef BVHNGPUIntersectorStream< gpu::Triangle1v > BVHNGPUTriangle1vIntersectorStream;
    typedef BVHNGPUIntersectorStream< gpu::Quad1v     > BVHNGPUQuad1vIntersectorStream;
        
#endif
    
    /*! BVH ray GPU intersectors */
    
    class BVHNGPUTriangle1vIntersector1
    {
    public:
      static void intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context);
      static void occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context);
      static bool pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context);
    };
    
    void BVHNGPUTriangle1vIntersector1::intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context) {}
    void BVHNGPUTriangle1vIntersector1::occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context) {}   
    bool BVHNGPUTriangle1vIntersector1::pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context) { return false; }
    
    class BVHNGPUTriangle1vIntersector4
    {
    public:
      static void intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context);
      static void occluded (vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context);
    };

    void BVHNGPUTriangle1vIntersector4::intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context) {}    
    void BVHNGPUTriangle1vIntersector4::occluded(vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context) {}


    class BVHNGPUQuad1vIntersector1
    {
    public:
      static void intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context);
      static void occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context);
      static bool pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context);
    };
    
    void BVHNGPUQuad1vIntersector1::intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context) {}
    void BVHNGPUQuad1vIntersector1::occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context) {}   
    bool BVHNGPUQuad1vIntersector1::pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context) { return false; }
    
    class BVHNGPUQuad1vIntersector4
    {
    public:
      static void intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context);
      static void occluded (vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context);
    };

    void BVHNGPUQuad1vIntersector4::intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context) {}    
    void BVHNGPUQuad1vIntersector4::occluded(vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context) {}
    
    ////////////////////////////////////////////////////////////////////////////////
    /// General BVHIntersectorStreamPacketFallback Intersector
    ////////////////////////////////////////////////////////////////////////////////
#if defined(EMBREE_DPCPP_SUPPORT)

    DEFINE_INTERSECTORN(BVHGPUTriangle1vIntersectorStream,BVHNGPUTriangle1vIntersectorStream);
    DEFINE_INTERSECTOR1(BVHGPUTriangle1vIntersector1,BVHNGPUTriangle1vIntersector1);
    DEFINE_INTERSECTOR4(BVHGPUTriangle1vIntersector4,BVHNGPUTriangle1vIntersector4);    

    DEFINE_INTERSECTORN(BVHGPUQuad1vIntersectorStream,BVHNGPUQuad1vIntersectorStream);
    DEFINE_INTERSECTOR1(BVHGPUQuad1vIntersector1,BVHNGPUQuad1vIntersector1);
    DEFINE_INTERSECTOR4(BVHGPUQuad1vIntersector4,BVHNGPUQuad1vIntersector4);    

#endif    
  };
};
