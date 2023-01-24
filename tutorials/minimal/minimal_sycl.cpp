// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

/*
 * To use the Embree DPC++ API you have to include sycl.hpp before the
 * embree API headers.
 */
#include <sycl/sycl.hpp>
#include <embree4/rtcore.h>

#include <cstdio>
#include <limits>

#include "../common/sycl/util.h"

/*
 * A minimal tutorial. 
 *
 * It demonstrates how to intersect a ray with a single triangle. It is
 * meant to get you started as quickly as possible, and does not output
 * an image. 
 */

/* 
 * This is only required to make the tutorial compile even when
 * a custom namespace is set.
 */
#if defined(RTC_NAMESPACE_USE)
RTC_NAMESPACE_USE
#endif

const sycl::specialization_id<RTCFeatureFlags> feature_mask;
const RTCFeatureFlags required_features = RTC_FEATURE_FLAG_TRIANGLE;

struct Result {
  unsigned geomID;
  unsigned primID; 
  float tfar;
};

/*
 * This function allocated USM memory that is writeable by the device.
 */

template<typename T>
T* alignedSYCLMallocDeviceReadWrite(const sycl::queue& queue, size_t count, size_t align)
{
  if (count == 0)
    return nullptr;

  assert((align & (align - 1)) == 0);
  T *ptr = (T*)sycl::aligned_alloc(align, count * sizeof(T), queue, sycl::usm::alloc::shared);
  if (count != 0 && ptr == nullptr)
    throw std::bad_alloc();

  return ptr;
}

/*
 * This function allocated USM memory that is only readable by the
 * device. Using this mode many small allocations are possible by the
 * application.
 */

template<typename T>
T* alignedSYCLMallocDeviceReadOnly(const sycl::queue& queue, size_t count, size_t align)
{
  if (count == 0)
    return nullptr;

  assert((align & (align - 1)) == 0);
  T *ptr = (T*)sycl::aligned_alloc_shared(align, count * sizeof(T), queue, sycl::ext::oneapi::property::usm::device_read_only());
  if (count != 0 && ptr == nullptr)
    throw std::bad_alloc();

  return ptr;
}

void alignedSYCLFree(const sycl::queue& queue, void* ptr)
{
  if (ptr) sycl::free(ptr, queue);
}

/*
 * We will register this error handler with the device in initializeDevice(),
 * so that we are automatically informed on errors.
 * This is extremely helpful for finding bugs in your code, prevents you
 * from having to add explicit error checking to each Embree API call.
 */
void errorFunction(void* userPtr, enum RTCError error, const char* str)
{
  printf("error %d: %s\n", error, str);
}

/*
 * Embree has a notion of devices, which are entities that can run 
 * raytracing kernels.
 * We initialize our device here, and then register the error handler so that 
 * we don't miss any errors.
 *
 * rtcNewDevice() takes a configuration string as an argument. See the API docs
 * for more information.
 *
 * Note that RTCDevice is reference-counted.
 */
RTCDevice initializeDevice(sycl::context& sycl_context, sycl::device& sycl_device)
{
  RTCDevice device = rtcNewSYCLDevice(sycl_context, "");
  rtcSetDeviceSYCLDevice(device,sycl_device);

  if (!device)
    printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));

  rtcSetDeviceErrorFunction(device, errorFunction, NULL);
  return device;
}

/*
 * Create a scene, which is a collection of geometry objects. Scenes are 
 * what the intersect / occluded functions work on. You can think of a 
 * scene as an acceleration structure, e.g. a bounding-volume hierarchy.
 *
 * Scenes, like devices, are reference-counted.
 */
RTCScene initializeScene(RTCDevice device, const sycl::queue& queue)
{
  RTCScene scene = rtcNewScene(device);

  /* 
   * Create a triangle mesh geometry, and initialize a single triangle.
   * You can look up geometry types in the API documentation to
   * find out which type expects which buffers.
   *
   * We create buffers directly on the device, but you can also use
   * shared buffers. For shared buffers, special care must be taken
   * to ensure proper alignment and padding. This is described in
   * more detail in the API documentation.
   */
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  
  float* vertices = alignedSYCLMallocDeviceReadOnly<float>(queue, 3 * 3, 16);

  rtcSetSharedGeometryBuffer(geom,
                             RTC_BUFFER_TYPE_VERTEX,
                             0,
                             RTC_FORMAT_FLOAT3,
                             vertices,
                             0,
                             3*sizeof(float),
                             3);

  unsigned* indices = alignedSYCLMallocDeviceReadOnly<unsigned>(queue, 3, 16);
  
  rtcSetSharedGeometryBuffer(geom,
                             RTC_BUFFER_TYPE_INDEX,
                             0,
                             RTC_FORMAT_UINT3,
                             indices,
                             0,
                             3*sizeof(unsigned),
                             1);

  if (vertices && indices)
  {
    vertices[0] = 0.f; vertices[1] = 0.f; vertices[2] = 0.f;
    vertices[3] = 1.f; vertices[4] = 0.f; vertices[5] = 0.f;
    vertices[6] = 0.f; vertices[7] = 1.f; vertices[8] = 0.f;

    indices[0] = 0; indices[1] = 1; indices[2] = 2;
  }

  /*
   * You must commit geometry objects when you are done setting them up,
   * or you will not get any intersections.
   */
  rtcCommitGeometry(geom);

  /*
   * In rtcAttachGeometry(...), the scene takes ownership of the geom
   * by increasing its reference count. This means that we don't have
   * to hold on to the geom handle, and may release it. The geom object
   * will be released automatically when the scene is destroyed.
   *
   * rtcAttachGeometry() returns a geometry ID. We could use this to
   * identify intersected objects later on.
   */
  rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);

  /*
   * Like geometry objects, scenes must be committed. This lets
   * Embree know that it may start building an acceleration structure.
   */
  rtcCommitScene(scene);

  return scene;
}

/*
 * Cast a single ray with origin (ox, oy, oz) and direction
 * (dx, dy, dz).
 */

void castRay(sycl::queue& queue, const RTCScene scene, 
             float ox, float oy, float oz,
             float dx, float dy, float dz, Result* result)
{
  queue.submit([=](sycl::handler& cgh)
  {
    cgh.set_specialization_constant<feature_mask>(required_features);
  
    cgh.parallel_for(sycl::range<1>(1),[=](sycl::item<1> item, sycl::kernel_handler kh)
    {
      /*
       * The intersect arguments can be used to pass a feature mask,
       * which improves performance and JIT compile times on the GPU
       */
      RTCIntersectArguments args;
      rtcInitIntersectArguments(&args);

      const RTCFeatureFlags features = kh.get_specialization_constant<feature_mask>();
      args.feature_mask = features;

      /*
       * The ray hit structure holds both the ray and the hit.
       * The user must initialize it properly -- see API documentation
       * for rtcIntersect1() for details.
       */
      struct RTCRayHit rayhit;
      rayhit.ray.org_x = ox;
      rayhit.ray.org_y = oy;
      rayhit.ray.org_z = oz;
      rayhit.ray.dir_x = dx;
      rayhit.ray.dir_y = dy;
      rayhit.ray.dir_z = dz;
      rayhit.ray.tnear = 0;
      rayhit.ray.tfar = std::numeric_limits<float>::infinity();
      rayhit.ray.mask = -1;
      rayhit.ray.flags = 0;
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

      /*
       * There are multiple variants of rtcIntersect. This one
       * intersects a single ray with the scene.
       */
      rtcIntersect1(scene, &rayhit, &args);

      /*
       * write hit result to output buffer
       */
      result->geomID = rayhit.hit.geomID;
      result->primID = rayhit.hit.primID;
      result->tfar = rayhit.ray.tfar;
    });
  });
  queue.wait_and_throw();
  
  printf("%f, %f, %f: ", ox, oy, oz);
  if (result->geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* Note how geomID and primID identify the geometry we just hit.
     * We could use them here to interpolate geometry information,
     * compute shading, etc.
     * Since there is only a single triangle in this scene, we will
     * get geomID=0 / primID=0 for all hits.
     * There is also instID, used for instancing. See
     * the instancing tutorials for more information */
    printf("Found intersection on geometry %d, primitive %d at tfar=%f\n", 
           result->geomID,
           result->primID,
           result->tfar);
  }
  else
    printf("Did not find any intersection.\n");
}

/*
 * Enable persistent JIT compilation caching. These environment
 * variables must be set before the SYCL device creation.
 */

void enablePersistentJITCache()
{
#if defined(_WIN32)
  _putenv_s("SYCL_CACHE_PERSISTENT","1");
  _putenv_s("SYCL_CACHE_DIR","cache");
#else
  setenv("SYCL_CACHE_PERSISTENT","1",1);
  setenv("SYCL_CACHE_DIR","cache",1);
#endif
}

/* -------------------------------------------------------------------------- */

int main()
{
  enablePersistentJITCache();

  /* This will select the first GPU supported by Embree */
  sycl::device sycl_device;
  try {
    sycl_device = sycl::device(rtcSYCLDeviceSelector);
  } catch(std::exception& e) {
    std::cerr << "Caught exception creating sycl::device: " << e.what() << std::endl;
    embree::printAllSYCLDevices();
    return 1;
  }

  sycl::queue sycl_queue(sycl_device); 
  sycl::context sycl_context(sycl_device);
  
  RTCDevice device = initializeDevice(sycl_context,sycl_device);
  RTCScene scene = initializeScene(device, sycl_queue);

  Result* result = alignedSYCLMallocDeviceReadWrite<Result>(sycl_queue, 1, 16);
  
  /* This will hit the triangle at t=1. */
  castRay(sycl_queue, scene, 0.33f, 0.33f, -1, 0, 0, 1, result);

  /* This will not hit anything. */
  castRay(sycl_queue, scene, 1.00f, 1.00f, -1, 0, 0, 1, result);
  
  alignedSYCLFree(sycl_queue, result);

  /* Though not strictly necessary in this example, you should
   * always make sure to release resources allocated through Embree. */
  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
  
  return 0;
}
