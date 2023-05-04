// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <CL/sycl.hpp>
#include "tbb/tbb.h"

#include "../rttrace/rttrace.h"
#include "../rtbuild/rtbuild.h"

#include <vector>
#include <iostream>

void* dispatchGlobalsPtr = nullptr;

void exception_handler(sycl::exception_list exceptions)
{
  for (std::exception_ptr const& e : exceptions) {
    try {
      std::rethrow_exception(e);
    } catch(sycl::exception const& e) {
      std::cout << "Caught asynchronous SYCL exception: " << e.what() << std::endl;
    }
  }
};

inline void fwrite_uchar (unsigned char  v, std::fstream& file) { file.write((const char*)&v,sizeof(v)); }
inline void fwrite_ushort(unsigned short v, std::fstream& file) { file.write((const char*)&v,sizeof(v)); }

void storeTga(uint32_t* pixels, uint32_t width, uint32_t height, const std::string& fileName) try
{
  std::fstream file;
  file.exceptions (std::fstream::failbit | std::fstream::badbit);
  file.open (fileName.c_str(), std::fstream::out | std::fstream::binary);

  fwrite_uchar(0x00, file);
  fwrite_uchar(0x00, file);
  fwrite_uchar(0x02, file);
  fwrite_ushort(0x0000, file);
  fwrite_ushort(0x0000, file);
  fwrite_uchar(0x00, file);
  fwrite_ushort(0x0000, file);
  fwrite_ushort(0x0000, file);
  fwrite_ushort((unsigned short)width , file);
  fwrite_ushort((unsigned short)height, file);
  fwrite_uchar(0x18, file);
  fwrite_uchar(0x20, file);

  for (size_t y=0; y<height; y++) {
    for (size_t x=0; x<width; x++) {
      const uint32_t c = pixels[y*width+x];
      fwrite_uchar((unsigned char)((c>>0)&0xFF), file);
      fwrite_uchar((unsigned char)((c>>8)&0xFF), file);
      fwrite_uchar((unsigned char)((c>>16)&0xFF), file);
    }
  }
}
catch (std::exception const& e) {
  std::cout << "Error: Cannot write file " << fileName << std::endl;
  throw;
}

std::vector<char> readFile(const std::string& fileName) try
{
  std::fstream file;
  file.exceptions (std::fstream::failbit | std::fstream::badbit);
  file.open (fileName.c_str(), std::fstream::in | std::fstream::binary);

  file.seekg (0, std::ios::end);
  std::streampos size = file.tellg();
  std::vector<char> data(size);
  file.seekg (0, std::ios::beg);
  file.read (data.data(), size);
  file.close();

  return data;
}
catch (std::exception const& e) {
  std::cout << "Error: Cannot read file " << fileName << std::endl;
  throw;
}

bool compareTga(const std::string& fileNameA, const std::string& fileNameB)
{
  const std::vector<char> dataA = readFile(fileNameA);
  const std::vector<char> dataB = readFile(fileNameB);
  return dataA == dataB;
}

/* Properly allocates an acceleration structure buffer using ze_raytracing_mem_alloc_ext_desc_t property. */
void* alloc_accel_buffer(size_t bytes, sycl::device device, sycl::context context)
{
  ze_context_handle_t hContext = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(context);
  ze_device_handle_t  hDevice  = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device);

  ze_rtas_device_exp_properties_t rtasProp = { ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXP_PROPERTIES };
  ze_result_t err = zeDeviceGetRTASPropertiesExp(hDevice, &rtasProp );
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("get rtas device properties failed");
  
  ze_raytracing_mem_alloc_ext_desc_t rt_desc;
  rt_desc.stype = ZE_STRUCTURE_TYPE_RAYTRACING_MEM_ALLOC_EXT_DESC;
  rt_desc.pNext = nullptr;
  rt_desc.flags = 0;
    
  ze_device_mem_alloc_desc_t device_desc;
  device_desc.stype = ZE_STRUCTURE_TYPE_DEVICE_MEM_ALLOC_DESC;
  device_desc.pNext = &rt_desc;
  device_desc.flags = ZE_DEVICE_MEM_ALLOC_FLAG_BIAS_CACHED;
  device_desc.ordinal = 0;

  ze_host_mem_alloc_desc_t host_desc;
  host_desc.stype = ZE_STRUCTURE_TYPE_HOST_MEM_ALLOC_DESC;
  host_desc.pNext = nullptr;
  host_desc.flags = ZE_HOST_MEM_ALLOC_FLAG_BIAS_CACHED;
  
  void* ptr = nullptr;
  ze_result_t result = zeMemAllocShared(hContext,&device_desc,&host_desc,bytes,rtasProp.rtasBufferAlignment,hDevice,&ptr);
  if (result != ZE_RESULT_SUCCESS)
    throw std::runtime_error("acceleration buffer allocation failed");

  return ptr;
}

void free_accel_buffer(void* ptr, sycl::context context)
{
  ze_context_handle_t hContext = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(context);
  ze_result_t result = zeMemFree(hContext,ptr);
  if (result != ZE_RESULT_SUCCESS)
    throw std::runtime_error("acceleration buffer free failed");
}


/* dispatch globals allocation is for debugging only */

enum Flags : uint32_t {
  FLAGS_NONE,
  DEPTH_TEST_LESS_EQUAL = 1 << 0  // when set we use <= for depth test, otherwise <
};

struct DispatchGlobals
{
  uint64_t rtMemBasePtr;               // base address of the allocated stack memory
  uint64_t callStackHandlerKSP;             // this is the KSP of the continuation handler that is invoked by BTD when the read KSP is 0
  uint32_t asyncStackSize;             // async-RT stack size in 64 byte blocks
  uint32_t numDSSRTStacks : 16;        // number of stacks per DSS
  uint32_t syncRayQueryCount : 4;      // number of ray queries in the sync-RT stack: 0-15 mapped to: 1-16
  unsigned _reserved_mbz : 12;
  uint32_t maxBVHLevels;               // the maximal number of supported instancing levels (0->8, 1->1, 2->2, ...)
  Flags flags;                         // per context control flags
};

void* allocDispatchGlobals(sycl::device device, sycl::context context)
{
  size_t maxBVHLevels = 2; //RTC_MAX_INSTANCE_LEVEL_COUNT+1;
  
  size_t rtstack_bytes = (64+maxBVHLevels*(64+32)+63)&-64;
  size_t num_rtstacks = 1<<17; // this is sufficiently large also for PVC
  size_t dispatchGlobalSize = 128+num_rtstacks*rtstack_bytes;
  
  void* dispatchGlobalsPtr = alloc_accel_buffer(dispatchGlobalSize,device,context);
  memset(dispatchGlobalsPtr, 0, dispatchGlobalSize);

  DispatchGlobals* dg = (DispatchGlobals*) dispatchGlobalsPtr;
  dg->rtMemBasePtr = (uint64_t) dispatchGlobalsPtr + dispatchGlobalSize;
  dg->callStackHandlerKSP = 0;
  dg->asyncStackSize = 0;
  dg->numDSSRTStacks = 0;
  dg->syncRayQueryCount = 0;
  dg->_reserved_mbz = 0;
  dg->maxBVHLevels = maxBVHLevels;
  dg->flags = DEPTH_TEST_LESS_EQUAL;
  
  return dispatchGlobalsPtr;
}

/* vertex indices for cornell_box model */
ze_rtas_triangle_indices_uint32_exp_t indices[] = {
  { 0, 1, 2 },
  { 0, 2, 3 },
  { 4, 5, 6 },
  { 4, 6, 7 },
  { 8, 9, 10 },
  { 8, 10, 11 },
  { 12, 13, 14 },
  { 12, 14, 15 },
  { 16, 17, 18 },
  { 16, 18, 19 },
  { 20, 21, 22 },
  { 20, 22, 23 },
  { 24, 25, 26 },
  { 24, 26, 27 },
  { 28, 29, 30 },
  { 28, 30, 31 },
  { 32, 33, 34 },
  { 32, 34, 35 },
  { 36, 37, 38 },
  { 36, 38, 39 },
  { 40, 41, 42 },
  { 40, 42, 43 },
  { 44, 45, 46 },
  { 44, 46, 47 },
  { 48, 49, 50 },
  { 48, 50, 51 },
  { 52, 53, 54 },
  { 52, 54, 55 },
  { 56, 57, 58 },
  { 56, 58, 59 },
  { 60, 61, 62 },
  { 60, 62, 63 },
  { 64, 65, 66 },
  { 64, 66, 67 }
};

/* vertex positions for cornell_box model */
ze_rtas_float3_exp_t vertices[] = {
  { 552.8, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 559.2 },
  { 549.6, 0, 559.2 },
  { 290, 0, 114 },
  { 240, 0, 272 },
  { 82, 0, 225 },
  { 130, 0, 65 },
  { 472, 0, 406 },
  { 314, 0, 456 },
  { 265, 0, 296 },
  { 423, 0, 247 },
  { 556, 548.8, 0 },
  { 556, 548.8, 559.2 },
  { 0, 548.8, 559.2 },
  { 0, 548.8, 0 },
  { 549.6, 0, 559.2 },
  { 0, 0, 559.2 },
  { 0, 548.8, 559.2 },
  { 556, 548.8, 559.2 },
  { 0, 0, 559.2 },
  { 0, 0, 0 },
  { 0, 548.8, 0 },
  { 0, 548.8, 559.2 },
  { 552.8, 0, 0 },
  { 549.6, 0, 559.2 },
  { 556, 548.8, 559.2 },
  { 556, 548.8, 0 },
  { 130, 165, 65 },
  { 82, 165, 225 },
  { 240, 165, 272 },
  { 290, 165, 114 },
  { 290, 0, 114 },
  { 290, 165, 114 },
  { 240, 165, 272 },
  { 240, 0, 272 },
  { 130, 0, 65 },
  { 130, 165, 65 },
  { 290, 165, 114 },
  { 290, 0, 114 },
  { 82, 0, 225 },
  { 82, 165, 225 },
  { 130, 165, 65 },
  { 130, 0, 65 },
  { 240, 0, 272 },
  { 240, 165, 272 },
  { 82, 165, 225 },
  { 82, 0, 225 },
  { 423, 330, 247 },
  { 265, 330, 296 },
  { 314, 330, 456 },
  { 472, 330, 406 },
  { 423, 0, 247 },
  { 423, 330, 247 },
  { 472, 330, 406 },
  { 472, 0, 406 },
  { 472, 0, 406 },
  { 472, 330, 406 },
  { 314, 330, 456 },
  { 314, 0, 456 },
  { 314, 0, 456 },
  { 314, 330, 456 },
  { 265, 330, 296 },
  { 265, 0, 296 },
  { 265, 0, 296 },
  { 265, 330, 296 },
  { 423, 330, 247 },
  { 423, 0, 247 },
};

/* builds acceleration structure */
void* build_rtas(sycl::device device, sycl::context context)
{
  /* get L0 handles */
  ze_driver_handle_t hDriver = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device.get_platform());
  ze_device_handle_t hDevice = sycl::get_native<sycl::backend::ext_oneapi_level_zero>(device);
    
  /* create rtas builder object */
  ze_rtas_builder_exp_desc_t builderDesc = { ZE_STRUCTURE_TYPE_RTAS_BUILDER_EXP_DESC };
  ze_rtas_builder_exp_handle_t hBuilder = nullptr;
  ze_result_t err = zeRTASBuilderCreateExp(hDriver, &builderDesc, &hBuilder);
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("ze_rtas_builder creation failed");
    
  /* create geometry descriptor for single triangle mesh */
  ze_rtas_builder_triangles_geometry_info_exp_t mesh = {};
  mesh.geometryType = ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES;
  mesh.geometryFlags = 0;
  mesh.geometryMask = 0xFF;
  
  mesh.triangleFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_TRIANGLE_INDICES_UINT32;
  mesh.triangleCount = sizeof(indices)/sizeof(ze_rtas_triangle_indices_uint32_exp_t);
  mesh.triangleStride = sizeof(ze_rtas_triangle_indices_uint32_exp_t);
  mesh.pTriangleBuffer = indices;

  mesh.vertexFormat = ZE_RTAS_BUILDER_INPUT_DATA_FORMAT_EXP_FLOAT3;
  mesh.vertexCount = sizeof(vertices)/sizeof(ze_rtas_float3_exp_t);
  mesh.vertexStride = sizeof(ze_rtas_float3_exp_t);
  mesh.pVertexBuffer = vertices;

  /* fill geometry descriptor array with pointer to single geometry descriptor */
  std::vector<ze_rtas_builder_geometry_info_exp_t*> descs;
  descs.push_back((ze_rtas_builder_geometry_info_exp_t*)&mesh);
  
  /* get acceleration structure format for this device */
  ze_rtas_device_exp_properties_t rtasProp = { ZE_STRUCTURE_TYPE_RTAS_DEVICE_EXP_PROPERTIES };
  err = zeDeviceGetRTASPropertiesExp(hDevice, &rtasProp );
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("get rtas device properties failed");

  /* create parallel operation for parallel build */
  ze_rtas_parallel_operation_exp_handle_t hParallelOperation = nullptr;
  err = zeRTASParallelOperationCreateExp(hDriver, &hParallelOperation);
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("zeRTASParallelOperationCreateExp failed");

  /* create descriptor of build operation */
  size_t accelBufferBytesOut = 0;
  ze_rtas_aabb_exp_t bounds;
  ze_rtas_builder_build_op_exp_desc_t buildOp = {};
  buildOp.stype = ZE_STRUCTURE_TYPE_RTAS_BUILDER_BUILD_OP_EXP_DESC;
  buildOp.pNext = nullptr;
  buildOp.rtasFormat = rtasProp.rtasFormat;
  buildOp.buildQuality = ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_MEDIUM;
  buildOp.buildFlags = 0;
  buildOp.ppGeometries = (const ze_rtas_builder_geometry_info_exp_t **) descs.data();
  buildOp.numGeometries = descs.size();
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
  buildOp.dispatchGlobalsPtr = dispatchGlobalsPtr;
#endif

  /* query required buffer sizes */
  ze_rtas_builder_exp_properties_t buildProps = { ZE_STRUCTURE_TYPE_RTAS_BUILDER_EXP_PROPERTIES };
  err = zeRTASBuilderGetBuildPropertiesExp(hBuilder,&buildOp,&buildProps);
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("zeRTASBuilderGetBuildPropertiesExp failed");

  /* allocate scratch buffer */
  std::vector<char> scratchBuffer(buildProps.scratchBufferSizeBytes);
  memset(scratchBuffer.data(),0,scratchBuffer.size());

  /* allocate acceleration structure buffer */
  size_t accelBytes = buildProps.rtasBufferSizeBytesMaxRequired;
  void* accel = alloc_accel_buffer(accelBytes,device,context);
  memset(accel,0,accelBytes); // optional
  
  /* build acceleration strucuture multi threaded */
  err = zeRTASBuilderBuildExp(hBuilder,&buildOp,
                                  scratchBuffer.data(),scratchBuffer.size(),
                                  accel, accelBytes,
                                  hParallelOperation,
                                  nullptr, &bounds, &accelBufferBytesOut);
  
  if (err != ZE_RESULT_ERROR_HANDLE_OBJECT_IN_USE)
    throw std::runtime_error("zeRTASBuilderBuildExp failed");

  /* after the build is started one can query number of threads to use for the build */
  ze_rtas_parallel_operation_exp_properties_t prop = { ZE_STRUCTURE_TYPE_RTAS_PARALLEL_OPERATION_EXP_PROPERTIES };
  err = zeRTASParallelOperationGetPropertiesExp(hParallelOperation,&prop);
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("zeRTASParallelOperationGetPropertiesExp failed");

  /* build in parallel using maximal number of build threads */
  tbb::parallel_for(0u, prop.maxConcurrency, 1u, [&](uint32_t) {
    err = zeRTASParallelOperationJoinExp(hParallelOperation);
  });
  
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("zeRTASParallelOperationJoinExp failed");

  /* destroy parallel operation again */
  err = zeRTASParallelOperationDestroyExp(hParallelOperation);
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("zeRTASParallelOperationDestroyExp failed");

  /* destroy rtas builder again */
  err = zeRTASBuilderDestroyExp(hBuilder);
  if (err != ZE_RESULT_SUCCESS)
    throw std::runtime_error("zeRTASBuilderDestroyExp failed");
  
  return accel;
}

/* render using simple UV shading */
void render(unsigned int x, unsigned int y, void* bvh, unsigned int* pixels, unsigned int width, unsigned int height)
{
  /* write zero image if ray tracing extension is not supported */
  intel_raytracing_ext_flag_t flags = intel_get_raytracing_ext_flag();
  if (!(flags & intel_raytracing_ext_flag_ray_query)) {
    pixels[y*width+x] = 0;
    return;
  }
  
  /* fixed camera */
  sycl::float3 vx(-1, -0, -0);
  sycl::float3 vy(-0, -1, -0);
  sycl::float3 vz(32, 32, 95.6379f);
  sycl::float3 p(278, 273, -800);

  /* compute primary ray */
  intel_ray_desc_t ray;
  ray.origin = p;
  ray.direction = float(x)*vx/8.0f + float(y)*vy/8.0f + vz;;
  ray.tmin = 0.0f;
  ray.tmax = INFINITY;
  ray.mask = 0xFF;
  ray.flags = intel_ray_flags_none;

  /* trace ray */
  intel_ray_query_t query = intel_ray_query_init(ray,(intel_raytracing_acceleration_structure_t)bvh);
  intel_ray_query_start_traversal(query);
  intel_ray_query_sync(query);

  /* get UVs of hit point */
  float u = 0, v = 0;
  if (intel_has_committed_hit(query))
  {
    sycl::float2 uv = intel_get_hit_barycentrics( query, intel_hit_type_committed_hit );
    u = uv.x();
    v = uv.y();
  }

  /* write color to framebuffer */
  sycl::float3 color(u,v,1.0f-u-v);
  unsigned int r = (unsigned int) (255.0f * color.x());
  unsigned int g = (unsigned int) (255.0f * color.y());
  unsigned int b = (unsigned int) (255.0f * color.z());
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
}

int main(int argc, char* argv[])
{
  /* use can specify reference image to compare against */
  char* reference_img = NULL;
  if (argc > 2 && std::string(argv[1]) == std::string("--compare"))
    reference_img = argv[2];

  /* create SYCL objects */
  sycl::device device = sycl::device(sycl::gpu_selector_v);
  sycl::queue queue = sycl::queue(device,exception_handler);
  sycl::context context = queue.get_context();

#if defined(ZE_RAYTRACING_RT_SIMULATION)
  RTCore::Init();
  RTCore::SetXeVersion((RTCore::XeVersion)ZE_RAYTRACING_DEVICE);
#endif

  /* initialize L0 ray tracing extension */
  zeRTASInitExp();

#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
  dispatchGlobalsPtr = allocDispatchGlobals(device,context);
#endif

  /* build acceleration structure */
  void* bvh = build_rtas(device,context);

  /* creates framebuffer */
  static const int width = 512;
  static const int height = 512;
  unsigned int* pixels = (unsigned int*) sycl::aligned_alloc(64,width*height*sizeof(unsigned int),device,context,sycl::usm::alloc::shared);
  memset(pixels, 0, width*height*sizeof(uint32_t));

  /* renders image on device */
#if defined(ZE_RAYTRACING_RT_SIMULATION)
  tbb::parallel_for(tbb::blocked_range2d<uint32_t>(0,height,0,width),
     [&](const tbb::blocked_range2d<uint32_t>& r) {
        for (int y=r.rows().begin(); y<r.rows().end(); y++) {
          for (int x=r.cols().begin(); x<r.cols().end(); x++) {
            render(x,y,bvh,pixels,width,height);
          }
        }
     });
#else
  queue.submit([&](sycl::handler& cgh) {
                 const sycl::range<2> range(width,height);
                 cgh.parallel_for(range, [=](sycl::item<2> item) {
                                              const uint32_t x = item.get_id(0);
                                              const uint32_t y = item.get_id(1);
                                              render(x,y,bvh,pixels,width,height);
                                            });
               });
  queue.wait_and_throw();
#endif
  
  /* free acceleration structure again */
  free_accel_buffer(bvh,context);

#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
  free_accel_buffer(dispatchGlobalsPtr, context);
#endif
  
  /* cleanup L0 ray tracing extension */
  zeRTASExitExp();

#if defined(ZE_RAYTRACING_RT_SIMULATION)
  RTCore::Cleanup();
#endif

  /* store image to disk */
  storeTga(pixels,width,height,"cornell_box.tga");
  if (!reference_img) return 0;

  /* compare to reference image */
  const bool ok = compareTga("cornell_box.tga", reference_img);
  std::cout << "cornell_box ";
  if (ok) std::cout << "[PASSED]" << std::endl;
  else    std::cout << "[FAILED]" << std::endl;

  return ok ? 0 : 1;
}
