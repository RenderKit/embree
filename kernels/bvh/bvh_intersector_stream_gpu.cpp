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

#define STACK_CULLING 1



namespace embree
{
  namespace isa
  {

    /*! BVH ray stream GPU intersector */
    class BVHNGPUIntersectorStream
    {
      typedef BVHN<4> BVH;
      typedef typename BVH::NodeRef NodeRef;

    public:
      static void intersect(Accel::Intersectors* This, RayHitN** inputRays, size_t numRays, IntersectContext* context);
      static void occluded (Accel::Intersectors* This, RayN** inputRays, size_t numRays, IntersectContext* context);

    };

#if defined(EMBREE_DPCPP_SUPPORT)


    inline float dot3(const float3 &a,
		      const float3 &b)
    {
#if 0
      // this is currently broken
      return cl::sycl::dot(a,b);
#else      
      return a.x()*b.x() + a.y()*b.y() + a.z()*b.z();
#endif      
    }

    //const float4 _v0 = (slotID % 2) == 0 ? quad1[quadID].v0 : quad1[quadID].v2;
    
    inline float intersectQuad1(const cl::sycl::intel::sub_group &sg,
				const gpu::Quad1 *const quad1,
				const uint numQuads,
				const float3 &org,
				const float3 &dir,
				const float &tnear,
				const float &tfar,
				gpu::RTCHitGPU &hit,
				const unsigned int slotID)
    {
      float new_tfar = tfar;
      const uint quadID = slotID >> 1;  
      if (slotID < numQuads*2)
	{
	  //const float4 _v0 = cselect((int)((slotID % 2) == 0),quad1[quadID].v0,quad1[quadID].v2);	  
	  const float4 *const _v0_ptr = (float4*)&quad1[quadID];
	  const float4 _v0 = _v0_ptr[slotID & 1];
	  const uint primID = gpu::as_uint((float)_v0.w());
	  const float4 _v1 = quad1[quadID].v1;
	  const float4 _v2 = quad1[quadID].v3;
	  const uint geomID = gpu::as_uint((float)_v1.w());	    
	  const float3 v0 = _v0.xyz();
	  const float3 v1 = _v1.xyz();
	  const float3 v2 = _v2.xyz();

	  /* moeller-trumbore test */	  
	  const float3 e1 = v0 - v1;
	  const float3 e2 = v2 - v0;
	  const float3 tri_Ng = cl::sycl::cross(e1,e2);
	  const float den = dot3(tri_Ng,dir);   			   
	  const float inv_den = cl::sycl::native::recip(den); 
	  const float3 tri_v0_org = v0 - org;
	  const float3 R = cl::sycl::cross(dir,tri_v0_org);
	  const float u = dot3(R,e2) * inv_den;
	  const float v = dot3(R,e1) * inv_den;
	  float t = dot3(tri_v0_org,tri_Ng) * inv_den; 
	  int m_hit = (u >= 0.0f) & (v >= 0.0f) & (u+v <= 1.0f);
	  //if (m_hit == 0) return; // early out
	  m_hit &= (tnear <= t) & (t < tfar); // den != 0.0f &&
	  if (m_hit) 
	    {
	      new_tfar = t;
	      hit.Ng[0]  = tri_Ng.x();
	      hit.Ng[1]  = tri_Ng.y();
	      hit.Ng[2]  = tri_Ng.z();	      
	      hit.u      = u;
	      hit.v      = v;
	      hit.primID = primID;
	      hit.geomID = geomID;	      
	    }	  
	}
      return new_tfar;
    }
    

    [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void traceRayBVH16(const cl::sycl::intel::sub_group &sg, gpu::RTCRayGPU &ray, gpu::RTCHitGPU &hit, void *bvh_mem, const cl::sycl::stream &out)
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

      //const float3 inv_dir_org = -inv_dir * org; // FIXME

      const float3 inv_dir_org(-(float)inv_dir.x() * (float)org.x(),-(float)inv_dir.y() * (float)org.y(),-(float)inv_dir.z() * (float)org.z());
            
      const uint max_uint  = 0xffffffff;  
      const uint mask_uint = 0xfffffff0;

      const char *const bvh_base = (char*)bvh_mem;
      stack_offset[0] = max_uint; // sentinel
      stack_dist[0]   = -(float)INFINITY;
      stack_offset[1] = sizeof(struct gpu::BVHBase); // single node after bvh start 
      stack_dist[1]   = -(float)INFINITY;

      unsigned int sindex = 2; 

      while(1)
	{ 
	  sindex--;

#if STACK_CULLING  == 1    
	  if (stack_dist[sindex] > tfar) continue;
#endif
	  
	  unsigned int cur = stack_offset[sindex]; 

	  while((cur & BVH_LEAF_MASK) == 0) 
	    {
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
	      
	      for (uint i=0;i<popc-1;i++)
		{
		  const int t_max = sg.reduce<int,cl::sycl::intel::maximum>(t); // from larger to smaller distance
		  t = (t == t_max) ? max_uint : t;
		  const uint index = t_max & (~mask_uint);
		  stack_offset[sindex] = sg.broadcast<uint> (offset,index);
		  stack_dist[sindex]   = sg.broadcast<float>(fnear,index);
		  sindex++;
		}
	      const int t_max = sg.reduce<int,cl::sycl::intel::maximum>(t); // from larger to smaller distance
	      cur = sg.broadcast<uint>(offset,t_max & (~mask_uint));
	    }
	  
	  if (cur == max_uint) break; // sentinel reached -> exit

	  const uint numPrims = gpu::getNumLeafPrims(cur);
	  const uint leafOffset = gpu::getLeafOffset(cur);    

	  const gpu::Quad1 *const quads = (gpu::Quad1 *)(bvh_base + leafOffset);
	  hit_tfar = intersectQuad1(sg,quads, numPrims, org, dir, tnear, hit_tfar, local_hit, subgroupLocalID);	  
	  tfar = sg.reduce<float,cl::sycl::intel::minimum>(hit_tfar);	  
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

    void BVHNGPUIntersectorStream::intersect(Accel::Intersectors* This, RayHitN** _inputRays, size_t numRays, IntersectContext* context)
    {
      BVH* __restrict__ bvh = (BVH*) This->ptr;
      
      if (bvh->root == BVH::emptyNode)
	{
	  PRINT(bvh);
	  PRINT(bvh->root);
	  PRINT("empty");
	  exit(0);
	  return;
	}

      
#if defined(EMBREE_DPCPP_SUPPORT)
      gpu::RTCRayHitGPU* inputRays = (gpu::RTCRayHitGPU*)_inputRays;
      void *bvh_mem = (void*)(size_t)(bvh->root);

      DBG(
	  PRINT( sizeof(gpu::RTCRayHitGPU) );
	  PRINT( sizeof(RTCRayHit) );	  
	  );
      assert( sizeof(gpu::RTCRayHitGPU) == sizeof(RTCRayHit) );
      
      DBG(numRays = 1);
      
      DeviceGPU* deviceGPU = (DeviceGPU*)bvh->device;
      cl::sycl::queue &gpu_queue = deviceGPU->getQueue();

      {
	cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {

	    cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
	    const cl::sycl::nd_range<1> nd_range(numRays*cl::sycl::range<1>(BVH_NODE_N),cl::sycl::range<1>(BVH_NODE_N));		  
	    cgh.parallel_for<class trace_ray_stream>(nd_range,[=](cl::sycl::nd_item<1> item) {
		const uint groupID   = item.get_group(0);
		cl::sycl::intel::sub_group sg = item.get_sub_group();
		traceRayBVH16(sg,inputRays[groupID].ray,inputRays[groupID].hit,bvh_mem,out);	      
	      });		  
	  });
	try {
	  gpu_queue.wait_and_throw();
	} catch (cl::sycl::exception const& e) {
	  std::cout << "Caught synchronous SYCL exception:\n"
		    << e.what() << std::endl;
	}
      }

      DBG(exit(0));
#endif      
    }

    void BVHNGPUIntersectorStream::occluded (Accel::Intersectors* This, RayN** inputRays, size_t numRays, IntersectContext* context)
    {
    }


    /*! BVH ray GPU intersectors */
    
    class BVHNGPUIntersector1
    {
    public:
      static void intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context);
      static void occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context);
      static bool pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context);
    };
    
    void BVHNGPUIntersector1::intersect (const Accel::Intersectors* This, RayHit& ray, IntersectContext* context)

    {
      
    }

    void BVHNGPUIntersector1::occluded  (const Accel::Intersectors* This, Ray& ray, IntersectContext* context)

    {
      
    }
    
    bool BVHNGPUIntersector1::pointQuery(const Accel::Intersectors* This, PointQuery* query, PointQueryContext* context)
    {
      return false;
    }
    



    class BVHNGPUIntersector4
    {
    public:
      static void intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context);
      static void occluded (vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context);
    };

    void BVHNGPUIntersector4::intersect(vint<4>* valid, Accel::Intersectors* This, RayHitK<4>& ray, IntersectContext* context)
    {
      
    }
    
    void BVHNGPUIntersector4::occluded(vint<4>* valid, Accel::Intersectors* This, RayK<4>& ray, IntersectContext* context)
    {
      
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// General BVHIntersectorStreamPacketFallback Intersector
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_INTERSECTORN(BVHGPUIntersectorStream,BVHNGPUIntersectorStream);
    DEFINE_INTERSECTOR1(BVHGPUIntersector1,BVHNGPUIntersector1);
    DEFINE_INTERSECTOR4(BVHGPUIntersector4,BVHNGPUIntersector4);    
    
  };
};
