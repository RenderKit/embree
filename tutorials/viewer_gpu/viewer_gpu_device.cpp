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

#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

#if defined(EMBREE_DPCPP_SUPPORT)
#define CL_TARGET_OPENCL_VERSION 220
#define SYCL_SIMPLE_SWIZZLES
#include <CL/sycl.hpp>
#endif

namespace embree {

#define SIMPLE_SHADING 1
#define DBG_PRINT_BUFFER_SIZE 1024*1024
#define DBG_PRINT_LINE_SIZE 512
#define USE_FCT_CALLS 1
  
  extern "C" ISPCScene* g_ispc_scene;
  extern "C" int g_instancing_mode;


/* scene data */
RTCScene g_scene = nullptr;


RTCScene convertScene(ISPCScene* scene_in)
{
  RTCScene scene_out = ConvertScene(g_device, scene_in,RTC_BUILD_QUALITY_MEDIUM);
  //rtcSetSceneBuildQuality(scene_out, RTC_BUILD_QUALITY_HIGH);
  rtcSetSceneBuildQuality(scene_out, RTC_BUILD_QUALITY_MEDIUM);

  /* commit individual objects in case of instancing */
  if (g_instancing_mode != ISPC_INSTANCING_NONE)
  {
    for (unsigned int i=0; i<scene_in->numGeometries; i++) {
      ISPCGeometry* geometry = g_ispc_scene->geometries[i];
      if (geometry->type == GROUP) rtcCommitScene(geometry->scene);
    }
  }

  return scene_out;
}

void postIntersectGeometry(const Ray& ray, DifferentialGeometry& dg, ISPCGeometry* geometry, int& materialID)
{
  if (geometry->type == TRIANGLE_MESH)
  {
    ISPCTriangleMesh* mesh = (ISPCTriangleMesh*) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == QUAD_MESH)
  {
    ISPCQuadMesh* mesh = (ISPCQuadMesh*) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == SUBDIV_MESH)
  {
    ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == CURVES)
  {
    ISPCHairSet* mesh = (ISPCHairSet*) geometry;
    materialID = mesh->geom.materialID;
  }
  else if (geometry->type == GROUP) {
    unsigned int geomID = ray.geomID; {
      postIntersectGeometry(ray,dg,((ISPCGroup*) geometry)->geometries[geomID],materialID);
    }
  }
  else
    assert(false);
}

typedef ISPCInstance* ISPCInstancePtr;

inline int postIntersect(const Ray& ray, DifferentialGeometry& dg)
{
  int materialID = 0;
  unsigned int instID = ray.instID[0]; {
    unsigned int geomID = ray.geomID; {
      ISPCGeometry* geometry = nullptr;
      if (g_instancing_mode != ISPC_INSTANCING_NONE) {
        ISPCInstance* instance = (ISPCInstancePtr) g_ispc_scene->geometries[instID];
        geometry = instance->child;
      } else {
        geometry = g_ispc_scene->geometries[geomID];
      }
      postIntersectGeometry(ray,dg,geometry,materialID);
    }
  }

  return materialID;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* renders a single screen tile */
void renderTileStandard(int taskIndex,
			int threadIndex,
			int* pixels,
			const unsigned int width,
			const unsigned int height,
			const float time,
			const ISPCCamera& camera,
			const int numTilesX,
			const int numTilesY)
{ 
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

#if defined(EMBREE_DPCPP_SUPPORT)

  class NEOGPUDeviceSelector : public cl::sycl::device_selector {
  public:
    int operator()(const cl::sycl::device &Device) const override {
      using namespace cl::sycl::info;

      const std::string DeviceName = Device.get_info<device::name>();
      const std::string DeviceVendor = Device.get_info<device::vendor>();      
      return Device.is_gpu() && DeviceName.find("HD Graphics NEO") ? 1 : -1;
    }
  };

  class CPUDeviceSelector : public cl::sycl::device_selector {
  public:
    int operator()(const cl::sycl::device &Device) const override {
      return Device.is_cpu() ? 1 : -1;
    }
  };

// === create exception handler ===
  
  auto exception_handler = [] (cl::sycl::exception_list exceptions) {
    for (std::exception_ptr const& e : exceptions) {
      try {
	std::rethrow_exception(e);
      } catch(cl::sycl::exception const& e) {
	std::cout << "Caught asynchronous SYCL exception:\n"
	<< e.what() << std::endl;
	FATAL("OpenCL Exception");	
      }
    }
  };


  inline cl::sycl::float3 Vec3fa_to_float3(const Vec3fa& v)
  {
    return cl::sycl::float3(v.x,v.y,v.z);
  }

  inline cl::sycl::float3 normalize(const cl::sycl::float3& v)
  {
    return v * cl::sycl::rsqrt(cl::sycl::dot(v,v));
  }

  cl::sycl::queue   *gpu_queue = nullptr;
  cl::sycl::device  *gpu_device = nullptr;
  
#endif

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
#if defined(EMBREE_DPCPP_SUPPORT)
  {
    using namespace cl::sycl;

    try {
      if (getenv("EMBREE_USE_CPU")) {
        gpu_queue  = new queue(CPUDeviceSelector(), exception_handler);
        gpu_device = new device(CPUDeviceSelector());
      } else {
        gpu_queue = new queue(NEOGPUDeviceSelector(), exception_handler);
        gpu_device = new device(NEOGPUDeviceSelector());
      }
      assert(gpu_queue);
      assert(gpu_device);
    } catch (cl::sycl::invalid_parameter_error &E) {
      std::cout << E.what() << std::endl;
      FATAL("OpenCL Exception");
    }
  }
  
  /* init embree GPU device */
  g_device = rtcNewDeviceGPU(cfg,gpu_device,gpu_queue);
  //g_device = rtcNewDevice(cfg);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
#endif  
}

#if defined(EMBREE_DPCPP_SUPPORT)

SYCL_EXTERNAL uint external_fct(uint a, uint b) { return a + b; }

SYCL_EXTERNAL void rtcIntersectGPUTest(cl::sycl::intel::sub_group &sg,
				       cl::sycl::global_ptr<RTCSceneTy> scene,
				       struct RTCRayHit &rayhit,
				       ulong ext_fct);

#endif

template<typename T>
__forceinline T wg_align(T x, unsigned int alignment)
{
  return ((x + alignment-1)/alignment)*alignment;
}  
  
/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
			       const unsigned int width,
			       const unsigned int height,
			       const float time,
			       const ISPCCamera& camera)
{
  /* create scene */
  if (!g_scene) {
    g_scene = convertScene(g_ispc_scene);
    rtcCommitScene (g_scene);
  }  
#if defined(EMBREE_DPCPP_SUPPORT)

  const unsigned int wg_width  = wg_align(width,16);
  const unsigned int wg_height = wg_align(height,16);
  
  assert(gpu_device);
  assert(gpu_queue);
  
  /* allocate stream of rays in USM */  
  const size_t numRays = width*height;
  RTCRayHit *rtc_rays = (RTCRayHit*)cl::sycl::aligned_alloc(64,sizeof(RTCRayHit)*numRays,*gpu_device,gpu_queue->get_context(),cl::sycl::usm::alloc::shared);
  assert(rtc_rays);

  /* allocate temporary USM frame buffer */  
  const size_t numPixels = width*height;
  int *fb = (int*)cl::sycl::aligned_alloc(64,sizeof(int)*numPixels,*gpu_device,gpu_queue->get_context(),cl::sycl::usm::alloc::shared);
  assert(fb);

  double t0, t1;
  
  /* generate primary ray stream */    
  {
#if USE_FCT_CALLS == 1    
    RTCScene scene = g_scene;
#endif    
    using namespace cl::sycl;	
    const float3 cam_p  = Vec3fa_to_float3(camera.xfm.p);
    const float3 cam_vx = Vec3fa_to_float3(camera.xfm.l.vx);
    const float3 cam_vy = Vec3fa_to_float3(camera.xfm.l.vy);
    const float3 cam_vz = Vec3fa_to_float3(camera.xfm.l.vz);

    t0 = getSeconds();
    cl::sycl::event queue_event = gpu_queue->submit([&](cl::sycl::handler &cgh) {
	const cl::sycl::nd_range<2> nd_range(cl::sycl::range<2>(wg_width,wg_height),cl::sycl::range<2>(16,1));	
	//const cl::sycl::nd_range<2> nd_range(cl::sycl::range<2>(16,1),cl::sycl::range<2>(16,1));
	
	//cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);		
	cgh.parallel_for<class init_rays>(nd_range,[=](cl::sycl::nd_item<2> item) {
	    const uint x = item.get_global_id(0);
	    const uint y = item.get_global_id(1);
	    //if (x < width && y < height) // FIXME: causes 4x slowdown
	      {
		const float3 org = cam_p;
		const float3 dir = normalize((float)x*cam_vx + (float)y*cam_vy + cam_vz);
		RTCRayHit &rh = rtc_rays[y*width+x];
		rh.ray.org_x = org.x();
		rh.ray.org_y = org.y();
		rh.ray.org_z = org.z();
		rh.ray.tnear = 0.0f;
		rh.ray.dir_x = dir.x();
		rh.ray.dir_y = dir.y();
		rh.ray.dir_z = dir.z();
		rh.ray.time  = 0.0f;		
		rh.ray.tfar  = (float)INFINITY;		
		rh.hit.primID = 0;
		rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
#if USE_FCT_CALLS == 1
		cl::sycl::global_ptr<RTCSceneTy> sycl_scene(scene);		
		cl::sycl::intel::sub_group sg = item.get_sub_group();
		ulong ext_fct = reinterpret_cast<ulong>(&external_fct);

		/* test function calls */
		rtcIntersectGPUTest(sg, sycl_scene, rh, ext_fct);
#endif
		
	      }
	  });		  
      });
    try {
      gpu_queue->wait_and_throw();
    } catch (cl::sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
		<< e.what() << std::endl;
      FATAL("OpenCL Exception");      
    }
  }  
  t1 = getSeconds();
  
#if USE_FCT_CALLS == 0
  
  /* trace ray stream */      
  t0 = getSeconds();

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  context.flags = RTC_INTERSECT_CONTEXT_FLAG_GPU;
  rtcIntersect1M(g_scene,&context,rtc_rays,numRays,sizeof(RTCRayHit));

  t1 = getSeconds();
#else
  
#endif
  std::cout << (float)numRays * 0.000001f / (t1 - t0) << " mrays/s" << std::endl;
  
  /* shade stream of rays */
  {
    using namespace cl::sycl;	
    cl::sycl::event queue_event = gpu_queue->submit([&](cl::sycl::handler &cgh) {
	const cl::sycl::nd_range<2> nd_range(cl::sycl::range<2>(wg_width,wg_height),cl::sycl::range<2>(4,4));		  
	cgh.parallel_for<class shade_rays>(nd_range,[=](cl::sycl::nd_item<2> item) {
	    const uint x = item.get_global_id(0);
	    const uint y = item.get_global_id(1);
	    if (x < width && y < height)
	      {
		RTCRayHit &rh = rtc_rays[y*width+x];

		const float3 dir(rh.ray.dir_x,rh.ray.dir_y,rh.ray.dir_z);
		const float3 Ng(rh.hit.Ng_x,rh.hit.Ng_y,rh.hit.Ng_z);
	    
		/* eyelight shading */
		float3 color = float3(0.0f,0.0f,1.0f);
		if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID)
		  color = float3(cl::sycl::abs(dot(dir,normalize(Ng))));

		/* write color to framebuffer */
		const uint r = (uint) (255.0f * clamp((float)color.x(),0.0f,1.0f));
		const uint g = (uint) (255.0f * clamp((float)color.y(),0.0f,1.0f));
		const uint b = (uint) (255.0f * clamp((float)color.z(),0.0f,1.0f));
		fb[y*width+x] = (b << 16) + (g << 8) + r;	      
	      }
	  });		  
      });
    try {
      gpu_queue->wait_and_throw();
    } catch (cl::sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
		<< e.what() << std::endl;
      FATAL("OpenCL Exception");
    }
  }

  /* test */  
  //cl::sycl::free(test,gpu_queue->get_context());
  
  /* copy to real framebuffer */
  memcpy(pixels,fb,sizeof(int)*width*height);
  
  /* free USM allocated temporary framebuffer */  
  cl::sycl::free(fb,gpu_queue->get_context());
  
  /* free USM allocated stream of rays */  
  cl::sycl::free(rtc_rays,gpu_queue->get_context());

#endif  
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
}

} // namespace embree
