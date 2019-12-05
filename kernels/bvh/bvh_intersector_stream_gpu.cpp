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

#define EMBREE_SYCL_SIMD_LIBRARY

#include "bvh_intersector_gpu.h"

#define DBG(x) 
#define DBG_PRINT_BUFFER_SIZE 1024*1024
//#define DBG_PRINT_BUFFER_SIZE 0
#define DBG_PRINT_LINE_SIZE 512

namespace embree
{

#if defined(EMBREE_DPCPP_SUPPORT)   
    
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
    //traceRayBVH16<gpu::QBVHNodeN,gpu::Triangle1v>(sg,m_active,ray,hit,bvh_root,nullptr);
    //uint (*testfct)(uint, uint) = reinterpret_cast<uint (*)(uint, uint)>(ext_fct);
    //hit.primID = testfct(hit.primID,hit.primID); 
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
      void *bvh_mem = (void*)(bvh->scene->gpu_bvh_root);
      assert( sizeof(gpu::RTCRayHitGPU) == sizeof(RTCRayHit) );      
      DBG(numRays = 1);      
      DeviceGPU* deviceGPU = (DeviceGPU*)bvh->device;
      cl::sycl::queue &gpu_queue = deviceGPU->getGPUQueue();

      TraversalStats *tstats = nullptr;
      TSTATS(tstats = (TraversalStats *)cl::sycl::aligned_alloc(64,sizeof(TraversalStats),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),cl::sycl::usm::alloc::shared));
      TSTATS(tstats->reset());

      {
	cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
	    //cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);	    	    
	    const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(block_align(numRays,BVH_NODE_N)),cl::sycl::range<1>(BVH_NODE_N));
	    
	    cgh.parallel_for<class trace_ray_stream>(nd_range,[=](cl::sycl::nd_item<1> item) {
		const uint globalID   = item.get_global_id(0);
		cl::sycl::intel::sub_group sg = item.get_sub_group();
		//if (globalID < numRays)
		{
		  uint m_activeLanes = intel_sub_group_ballot(globalID < numRays);
		  if (m_activeLanes == 0xffff)
		    traceRayBVH16<gpu::QBVHNodeN,Primitive>(sg,m_activeLanes,inputRays[globalID].ray,inputRays[globalID].hit,bvh_mem,tstats);
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
