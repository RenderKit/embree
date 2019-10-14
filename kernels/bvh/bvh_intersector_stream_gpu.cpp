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

#define DBG(x) 

#if defined(ENABLE_RAY_STATS)
#define RAY_STATS(x) x
#else
#define RAY_STATS(x) 
#endif

#define TSTATS(x) x

#define STACK_CULLING 1

namespace embree
{
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

#if defined(EMBREE_DPCPP_SUPPORT)   

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

    
    template<typename Primitive>
    [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void traceRayBVH16(const cl::sycl::intel::sub_group &sg, gpu::RTCRayGPU &ray, gpu::RTCHitGPU &hit, void *bvh_mem, TraversalStats *tstats, const cl::sycl::stream &out)
    {
      unsigned int stack_offset[BVH_MAX_STACK_ENTRIES]; 
      float        stack_dist[BVH_MAX_STACK_ENTRIES];  

      // === init local hit ===
      gpu::RTCHitGPU local_hit;
      local_hit.init();

      const uint subgroupLocalID  = sg.get_local_id()[0];
      DBG(const uint subgroupSize = sg.get_local_range().size());

      const float3 org(ray.org[0],ray.org[1],ray.org[2]);
      const float3 dir(ray.dir[0],ray.dir[1],ray.dir[2]);
            
      const float tnear = ray.tnear;
      float tfar        = ray.tfar;
      float hit_tfar    = ray.tfar;

      DBG(
	  if (subgroupLocalID == 0)
	    out << "org " << org << " dir " << dir << " tnear " << tnear << " tfar " << tfar << cl::sycl::endl;
	  );
      
      const uint3 dir_mask = cselect(dir >= 0.0f,uint3(0),uint3(1));
      const float3 new_dir = cselect(dir != 0.0f, dir, float3(1E-18f));
      const float3 inv_dir = cl::sycl::native::recip(new_dir);
      const float3 inv_dir_org = -inv_dir * org; 
            
      const uint max_uint  = 0xffffffff;  
      const uint mask_uint = 0xfffffff0;

      const char *const bvh_base = (char*)bvh_mem;
      stack_offset[0] = max_uint; // sentinel
      stack_dist[0]   = -(float)INFINITY;
      stack_offset[1] = sizeof(struct gpu::BVHBase); // single node after bvh start 
      stack_dist[1]   = -(float)INFINITY;

      unsigned int sindex = 2;

      TSTATS(tstats->nrays_inc());

      while(1)
	{ 
	  sindex--;

#if STACK_CULLING  == 1    
	  if (stack_dist[sindex] > tfar) continue;
#endif
	  
	  gpu::NodeRef cur = stack_offset[sindex]; 

	  while(!cur.isLeaf()) 
	    {
	      TSTATS(tstats->tsteps_inc());
	      
	      const gpu::QBVHNodeN &node = *(gpu::QBVHNodeN*)(bvh_base + cur);
	      const gpu::NodeIntersectionData isec = intersectQBVHNodeN(sg,node,dir_mask,inv_dir,inv_dir_org,tnear,tfar);
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
		  continue;
		}

	      offset += cur; /* relative encoding */
	      const uint popc = cl::sycl::popcount(mask); 
	      cur = sg.broadcast<uint>(offset, cl::sycl::intel::ctz(mask));
	      	      
	      if (popc == 1) continue; // single hit only
	      int t = (gpu::as_int(fnear) & mask_uint) | subgroupLocalID;  // make the integer distance unique by masking off the least significant bits and adding the slotID
	      t = valid ? t : max_uint; // invalid slots set to MIN_INT, as we sort descending;	
	      
          cl::sycl::intel::maximum<int> maxBinOp;
	      for (uint i=0;i<popc-1;i++)
		{
		  const int t_max = sg.reduce<int>(t, maxBinOp); // from larger to smaller distance
		  t = (t == t_max) ? max_uint : t;
		  const uint index = t_max & (~mask_uint);
		  stack_offset[sindex] = sg.broadcast<uint> (offset,index);
		  stack_dist[sindex]   = sg.broadcast<float>(fnear,index);
		  sindex++;
		}
	      const int t_max = sg.reduce<int>(t, maxBinOp); // from larger to smaller distance
	      cur = sg.broadcast<uint>(offset,t_max & (~mask_uint));
	    }
	  
	  if (cur == max_uint) break; // sentinel reached -> exit

	  const uint numPrims = cur.getNumLeafPrims();
	  const uint leafOffset = cur.getLeafOffset();    

	  const Primitive *const prim = (Primitive *)(bvh_base + leafOffset);
	  TSTATS(tstats->isteps_inc());	  
	  hit_tfar = intersectPrimitive1v(sg, prim, numPrims, org, dir, tnear, hit_tfar, local_hit, subgroupLocalID);  
      cl::sycl::intel::minimum<float> minBinOp;
	  tfar = sg.reduce<float>(hit_tfar, minBinOp);	  
	}

      DBG(
	  for (uint i=0;i<subgroupSize;i++)
	    if (i == subgroupLocalID)
	      out << "i " << i << " local_hit " << local_hit << " tfar " << tfar << " hit_tfar " << hit_tfar << cl::sycl::endl;
	  );

      /* select hit with shortest distance */
      
      const uint index = cl::sycl::intel::ctz(intel_sub_group_ballot(tfar == hit_tfar));
      if (subgroupLocalID == index)
	if (local_hit.primID != -1)
	  {	 
	    hit = local_hit;
	    ray.tfar = tfar;
	    DBG(out << "ray.tfar " << ray.tfar << " hit " << hit << cl::sycl::endl);
	  }
    }
#endif

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

	    cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
	    const cl::sycl::nd_range<1> nd_range(numRays*cl::sycl::range<1>(BVH_NODE_N),cl::sycl::range<1>(BVH_NODE_N));		  
	    cgh.parallel_for<class trace_ray_stream>(nd_range,[=](cl::sycl::nd_item<1> item) {
		const uint groupID   = item.get_group(0);
		cl::sycl::intel::sub_group sg = item.get_sub_group();		
		traceRayBVH16<Primitive>(sg,inputRays[groupID].ray,inputRays[groupID].hit,bvh_mem,tstats,out);
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
