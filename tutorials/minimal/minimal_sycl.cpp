// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <CL/sycl.hpp>

#include <cstdio>

#include <embree4/rtcore.h>
#include <cstdio>
#include <limits>

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

struct Result {
  unsigned geomID;
  unsigned primID; 
  float tfar;
};

template<typename T>
T* alignedSYCLMalloc(const sycl::queue& queue, size_t count, size_t align)
{
  if (count == 0)
    return nullptr;

  assert((align & (align - 1)) == 0);
  T *ptr = (T*)sycl::aligned_alloc(align, count * sizeof(T), queue, sycl::usm::alloc::shared);
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
RTCDevice initializeDevice(sycl::context& context, sycl::queue& queue)
{
  RTCDevice device = rtcNewSYCLDevice(&context, &queue, "");

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
  
  float* vertices = alignedSYCLMalloc<float>(queue, 3 * 3, 16);

  rtcSetSharedGeometryBuffer(geom,
                             RTC_BUFFER_TYPE_VERTEX,
                             0,
                             RTC_FORMAT_FLOAT3,
                             vertices,
                             0,
                             3*sizeof(float),
                             3);

  unsigned* indices = alignedSYCLMalloc<unsigned>(queue, 3, 16);
  
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
    const sycl::nd_range<2> nd_range(sycl::range<2>(SYCL_SIMD_WIDTH, 1), 
                                     sycl::range<2>(SYCL_SIMD_WIDTH, 1));
#if __SYCL_COMPILER_VERSION >= 20210801    
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) [[sycl::reqd_sub_group_size(SYCL_SIMD_WIDTH)]]
#else                     
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) [[intel::reqd_sub_group_size(SYCL_SIMD_WIDTH)]]
#endif      
    {
      /*
       * The intersect context can be used to set intersection
       * filters or flags, and it also contains the instance ID stack
       * used in multi-level instancing.
       */
      struct RTCIntersectContext context;
      rtcInitIntersectContext(&context);

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
      rtcIntersect1(scene, &context, &rayhit);

      if (item.get_local_id() == 0) {
        result->geomID = rayhit.hit.geomID;
        result->primID = rayhit.hit.primID;
        result->tfar = rayhit.ray.tfar;
      }
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

/* -------------------------------------------------------------------------- */

int main()
{
  sycl::queue queue(sycl::gpu_selector{}); 
  sycl::context context = queue.get_context();
  
  RTCDevice device = initializeDevice(context,queue);
  RTCScene scene = initializeScene(device, queue);

  Result* result = alignedSYCLMalloc<Result>(queue, 1, 16);
  
  /* This will hit the triangle at t=1. */
  castRay(queue, scene, 0, 0, -1, 0, 0, 1, result);

  /* This will not hit anything. */
  castRay(queue, scene, 1, 1, -1, 0, 0, 1, result);
  
  alignedSYCLFree(queue, result);

  /* Though not strictly necessary in this example, you should
   * always make sure to release resources allocated through Embree. */
  rtcReleaseScene(scene);
  rtcReleaseDevice(device);
  
  return 0;
}
