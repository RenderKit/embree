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

namespace embree
{  
  namespace isa
  {

#if defined(EMBREE_DPCPP_SUPPORT)
    
#endif
    
    /*! BVH ray stream GPU intersector */
    template<typename Primitive>
    class BVHNGPUIntersectorStreamMB
    {
      typedef BVHN<4> BVH;

    public:
      static void intersect(Accel::Intersectors* This, RayHitN** inputRays, size_t numRays, IntersectContext* context);
      static void occluded (Accel::Intersectors* This, RayN** inputRays, size_t numRays, IntersectContext* context);

    };       

    template<typename Primitive>    
    void BVHNGPUIntersectorStreamMB<Primitive>::intersect(Accel::Intersectors* This, RayHitN** _inputRays, size_t numRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;      
      if (bvh->root == BVH::emptyNode) return;
      
#if defined(EMBREE_DPCPP_SUPPORT)
      PING;
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
		{
		  uint m_activeLanes = intel_sub_group_ballot(globalID < numRays);
		  if (m_activeLanes == 0xffff)
		    traceRayBVH16<gpu::QBVHNodeNMB,Primitive>(sg,m_activeLanes,inputRays[globalID].ray,inputRays[globalID].hit,bvh_mem,tstats);
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
    void BVHNGPUIntersectorStreamMB<Primitive>::occluded (Accel::Intersectors* This, RayN** inputRays, size_t numRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;      
      if (bvh->root == BVH::emptyNode) return;
      
    }

#if defined(EMBREE_DPCPP_SUPPORT)

    typedef BVHNGPUIntersectorStreamMB< gpu::Triangle1vMB > BVHNGPUTriangle1vMBIntersectorStream;
    typedef BVHNGPUIntersectorStreamMB< gpu::Quad1vMB     > BVHNGPUQuad1vMBIntersectorStream;
        
#endif
    
    /*! BVH ray GPU intersectors */
    
    class BVHNGPUTriangle1vMBIntersector1
    {
    public:
      static void intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context);
      static void occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context);
      static bool pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context);
    };
    
    void BVHNGPUTriangle1vMBIntersector1::intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context) {}
    void BVHNGPUTriangle1vMBIntersector1::occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context) {}   
    bool BVHNGPUTriangle1vMBIntersector1::pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context) { return false; }
    
    class BVHNGPUTriangle1vMBIntersector4
    {
    public:
      static void intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context);
      static void occluded (vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context);
    };

    void BVHNGPUTriangle1vMBIntersector4::intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context) {}    
    void BVHNGPUTriangle1vMBIntersector4::occluded(vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context) {}


    class BVHNGPUQuad1vMBIntersector1
    {
    public:
      static void intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context);
      static void occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context);
      static bool pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context);
    };
    
    void BVHNGPUQuad1vMBIntersector1::intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context) {}
    void BVHNGPUQuad1vMBIntersector1::occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context) {}   
    bool BVHNGPUQuad1vMBIntersector1::pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context) { return false; }
    
    class BVHNGPUQuad1vMBIntersector4
    {
    public:
      static void intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context);
      static void occluded (vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context);
    };

    void BVHNGPUQuad1vMBIntersector4::intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context) {}    
    void BVHNGPUQuad1vMBIntersector4::occluded(vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context) {}
    
    ////////////////////////////////////////////////////////////////////////////////
    /// General BVHIntersectorStreamPacketFallback Intersector
    ////////////////////////////////////////////////////////////////////////////////
#if defined(EMBREE_DPCPP_SUPPORT)

    DEFINE_INTERSECTORN(BVHGPUTriangle1vMBIntersectorStream,BVHNGPUTriangle1vMBIntersectorStream);
    DEFINE_INTERSECTOR1(BVHGPUTriangle1vMBIntersector1,BVHNGPUTriangle1vMBIntersector1);
    DEFINE_INTERSECTOR4(BVHGPUTriangle1vMBIntersector4,BVHNGPUTriangle1vMBIntersector4);    

    DEFINE_INTERSECTORN(BVHGPUQuad1vMBIntersectorStream,BVHNGPUQuad1vMBIntersectorStream);
    DEFINE_INTERSECTOR1(BVHGPUQuad1vMBIntersector1,BVHNGPUQuad1vMBIntersector1);
    DEFINE_INTERSECTOR4(BVHGPUQuad1vMBIntersector4,BVHNGPUQuad1vMBIntersector4);    

#endif    
  };
};
