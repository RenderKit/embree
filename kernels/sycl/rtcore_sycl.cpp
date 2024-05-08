// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#define RTC_EXPORT_API

#include "../common/default.h"
#include "../common/device.h"
#include "../common/scene.h"
#include "../common/context.h"
#include "../geometry/filter.h"
#include "rthwif_embree.h"
using namespace embree;

#define DBG(x)

RTC_NAMESPACE_BEGIN;

RTC_API_EXTERN_C RTCDevice rtcNewSYCLDeviceInternal(sycl::context context, const char* config);

void use_rthwif_embree();
void use_rthwif_production();

/* we define rtcNewSYCLDevice in libembree_sycl.a to avoid drop of rtcore_sycl.o during linking of libembree_sycl.a file */
RTC_API_EXTERN_C RTCDevice rtcNewSYCLDevice(sycl::context context, const char* config)
{
  use_rthwif_embree();     // to avoid drop of rthwif_embree.o during linking of libembree_sycl.a file
#if defined(EMBREE_SYCL_RT_VALIDATION_API)
  use_rthwif_production(); // to avoid drop of rthwif_production.o during linking of libembree_sycl.a file
#endif
  return rtcNewSYCLDeviceInternal(context, config);
}

// This function fetches USM shared allocated data of the scene (e.g, all geometries, BVH memory buffer)
// in a GPU kernel to trigger migration of the memory from CPU to GPU.
RTC_API_EXTERN_C bool prefetchUSMSharedOnGPU(RTCScene hscene)
{
  Scene* scene = (Scene*) hscene;
  //RTC_CATCH_BEGIN;
  //RTC_TRACE(prefetchUSMSharedOnGPU);
  //RTC_VERIFY_HANDLE(hscene);
  //RTC_ENTER_DEVICE(hscene);

  if (!(rtcGetSceneFlags(hscene) & RTC_SCENE_FLAG_PREFETCH_USM_SHARED_ON_GPU)) {
    return false;
  }

  if (!rtcGetDeviceProperty((RTCDevice)scene->device, RTC_DEVICE_PROPERTY_SYCL_DEVICE)) {
    return false;
  }

  auto deviceGPU = reinterpret_cast<DeviceGPU*>(scene->device);
  auto deviceSYCL = deviceGPU->getGPUDevice();

  sycl::queue queue(deviceSYCL);

  const unsigned num_workers = 1024;

  // we accumulate some nonsene data to prevent compiler
  // optimizing away the memory fetches in the GPU kernel
  size_t* result = sycl::malloc_shared<size_t>(num_workers, queue);

  // Use num_workers GPU work items to iterate over all USM shared
  // allocations to trigger USM migration from CPU to GPU
  queue.parallel_for(num_workers, [=](sycl::id<1> idx)
  {
    result[idx] = 0;
    {
      const size_t num_iterations = (scene->geometries.size() + num_workers - 1) / num_workers;
      for (size_t j = 0; j < num_iterations; ++j) {
        const size_t offset = idx * num_iterations + j;
        if (offset >= scene->geometries.size())
          return;

        const Geometry* geom = scene->geometries[offset].ptr;
        if (geom == nullptr) {
          continue;
        }

        result[idx] += (size_t)geom->size();

        // for instances and instance arrays, fetch the
        // transformation data to triggger USM migration
        if (geom->getType() == AccelSet::GTY_INSTANCE_CHEAP ||
            geom->getType() == AccelSet::GTY_INSTANCE_EXPENSIVE)
        {
          Instance* inst = (Instance*)geom;
          AffineSpace3ff t = inst->local2world[0];
          result[idx] += (size_t)(cast_f2i(t.l.vx.x) % 32);
        }
        else if (geom->getType() == AccelSet::GTY_INSTANCE_ARRAY)
        {
          InstanceArray* inst = (InstanceArray*)geom;
          for (int i = 0; i < inst->size(); ++i) {
            AffineSpace3fa t = inst->getLocal2World(i);
            result[idx] += (size_t)(cast_f2i(t.l.vx.x) % 32);
          }
        }
      }
    }

    {
      // iterate over BVH memory buffer in steps of 4KB
      // (page size on Intel Data Center Max GPUs)
      const size_t accel_size = scene->hwaccel.size() / (1 << 12);
      const size_t num_iterations = (accel_size + num_workers - 1) / num_workers;
      for (size_t j = 0; j < num_iterations; ++j) {
        const size_t offset = (idx * num_iterations + j) * (1 << 12);
        if (offset >= accel_size)
          continue;
        result[idx] += ((size_t)scene->hwaccel[offset] % 32);
      }
    }
  });
  queue.wait_and_throw();

  size_t sum = 0;
  for (size_t i = 0; i < num_workers; ++i)
    sum += result[i];

  sycl::free(result, queue);

  // not really usefull result, this is just to prevent compiler
  // optimizing away the memory reads in the GPU kernel
  return sum > 0;

  //RTC_CATCH_END2(scene);
}

#if defined(EMBREE_SYCL_SUPPORT) && (defined(__SYCL_DEVICE_ONLY__) || defined(EMBREE_SYCL_RT_SIMULATION))

SYCL_EXTERNAL __attribute__((always_inline)) void rtcIntersect1(RTCScene hscene, struct RTCRayHit* rayhit, struct RTCIntersectArguments* args)
{
  RTCIntersectArguments default_args;
  if (args == nullptr) {
    rtcInitIntersectArguments(&default_args);
    args = &default_args;
  }
  RTCRayQueryContext* context = args->context;

  RTCRayQueryContext defaultContext;
  if (unlikely(context == nullptr)) {
    rtcInitRayQueryContext(&defaultContext);
    context = &defaultContext;
  }
    
  rtcIntersectRTHW(hscene, context, rayhit, args); 
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcForwardIntersect1(const RTCIntersectFunctionNArguments* args_, RTCScene scene, struct RTCRay* iray, unsigned int instID)
{
  return rtcForwardIntersect1Ex(args_, scene, iray, instID, 0);
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcForwardIntersect1Ex(const RTCIntersectFunctionNArguments* args_, RTCScene scene, struct RTCRay* iray, unsigned int instID, unsigned int instPrimID)
{
  IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_;
  assert(args->N == 1);
  assert(args->forward_scene == nullptr);
  
  Ray* oray = (Ray*)args->rayhit;
  oray->org.x = iray->org_x;
  oray->org.y = iray->org_y;
  oray->org.z = iray->org_z;
  oray->dir.x = iray->dir_x;
  oray->dir.y = iray->dir_y;
  oray->dir.z = iray->dir_z;
  args->forward_scene = scene;
  instance_id_stack::push(args->context, instID, instPrimID);
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcOccluded1(RTCScene hscene, struct RTCRay* ray, struct RTCOccludedArguments* args)
{
  RTCOccludedArguments default_args;
  if (args == nullptr) {
    rtcInitOccludedArguments(&default_args);
    args = &default_args;
  }
  RTCRayQueryContext* context = args->context;

  RTCRayQueryContext defaultContext;
  if (unlikely(context == nullptr)) {
    rtcInitRayQueryContext(&defaultContext);
    context = &defaultContext;
  }
  
  rtcOccludedRTHW(hscene, context, ray, args);
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcForwardOccluded1(const RTCOccludedFunctionNArguments *args_, RTCScene scene, struct RTCRay *iray, unsigned int instID){
  return rtcForwardOccluded1Ex(args_, scene, iray, instID, 0);
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcForwardOccluded1Ex(const RTCOccludedFunctionNArguments *args_, RTCScene scene, struct RTCRay *iray, unsigned int instID, unsigned int instPrimID)
{
  OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_;
  assert(args->N == 1);
  assert(args->forward_scene == nullptr);
   
  Ray* oray = (Ray*)args->ray;
  oray->org.x = iray->org_x;
  oray->org.y = iray->org_y;
  oray->org.z = iray->org_z;
  oray->dir.x = iray->dir_x;
  oray->dir.y = iray->dir_y;
  oray->dir.z = iray->dir_z;
  args->forward_scene = scene;
  instance_id_stack::push(args->context, instID, instPrimID);
}

SYCL_EXTERNAL __attribute__((always_inline)) void* rtcGetGeometryUserDataFromScene (RTCScene hscene, unsigned int geomID)
{
  Scene* scene = (Scene*) hscene;
  //RTC_CATCH_BEGIN;
  //RTC_TRACE(rtcGetGeometryUserDataFromScene);
#if defined(DEBUG)
  //RTC_VERIFY_HANDLE(hscene);
  //RTC_VERIFY_GEOMID(geomID);
#endif
  //RTC_ENTER_DEVICE(hscene); // do not enable for performance reasons
  return scene->get(geomID)->getUserData();
  //RTC_CATCH_END2(scene);
  //return nullptr;
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcGetGeometryTransformFromScene(RTCScene hscene, unsigned int geomID, float time, enum RTCFormat format, void* xfm)
{
  Scene* scene = (Scene*) hscene;
  //RTC_CATCH_BEGIN;
  //RTC_TRACE(rtcGetGeometryTransformFromScene);
  //RTC_ENTER_DEVICE(hscene);
  AffineSpace3fa transform = one;
  Geometry* geom = scene->get(geomID);
  if (geom->getTypeMask() & Geometry::MTY_INSTANCE) {
    Instance* instance = (Instance*) geom;
    if (likely(instance->numTimeSteps <= 1))
      transform = instance->getLocal2World();
    else
      transform = instance->getLocal2World(time);
  }
  storeTransform(transform, format, (float*)xfm);
  //RTC_CATCH_END2(geometry);
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcInvokeIntersectFilterFromGeometry(const RTCIntersectFunctionNArguments* args_i, const RTCFilterFunctionNArguments* filter_args)
{
#if EMBREE_SYCL_GEOMETRY_CALLBACK
  IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_i;
  if (args->geometry->intersectionFilterN)
    args->geometry->intersectionFilterN(filter_args);
#endif
}

SYCL_EXTERNAL __attribute__((always_inline)) void rtcInvokeOccludedFilterFromGeometry(const RTCOccludedFunctionNArguments* args_i, const RTCFilterFunctionNArguments* filter_args)
{
#if EMBREE_SYCL_GEOMETRY_CALLBACK
  OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_i;
  if (args->geometry->occlusionFilterN)
    args->geometry->occlusionFilterN(filter_args);
#endif
}

#endif

RTC_NAMESPACE_END;
