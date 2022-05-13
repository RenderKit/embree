// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#define RTC_EXPORT_API

#include "default.h"
#include "device.h"
#include "scene.h"
#include "context.h"
#include "../rthwif/rthwif_embree.h"
#include "../geometry/filter.h"
using namespace embree;

#define DBG(x)

RTC_NAMESPACE_BEGIN;

RTC_API_EXTERN_C RTCDevice rtcNewSYCLDeviceInternal(sycl::context* sycl_context, sycl::queue* sycl_queue, const char* config);

void use_rthwif_embree();
void use_rthwif_production();

/* we define rtcNewSYCLDevice in libembree_sycl.a to avoid drop of rtcore_sycl.o during linking of libembree_sycl.a file */
RTC_API_EXTERN_C RTCDevice rtcNewSYCLDevice(sycl::context* sycl_context, sycl::queue* sycl_queue, const char* config)
{
  use_rthwif_embree();     // to avoid drop of rthwif_embree.o during linking of libembree_sycl.a file
  use_rthwif_production(); // to avoid drop of rthwif_production.o during linking of libembree_sycl.a file
  return rtcNewSYCLDeviceInternal(sycl_context, sycl_queue, config);
}

#if defined(__SYCL_DEVICE_ONLY__)

SYCL_EXTERNAL void rtcIntersect1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRayHit* rayhit)
{
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcIntersectRTHW(hscene, context, rayhit, &args);
}

SYCL_EXTERNAL void rtcIntersectEx1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRayHit* rayhit, struct RTCIntersectArguments* args)
{
  rtcIntersectRTHW(hscene, context, rayhit, args); 
}

SYCL_EXTERNAL void rtcForwardIntersect1(const RTCIntersectFunctionNArguments* args_, RTCScene scene, struct RTCRay* iray)
{
  assert(args->N == 1);
  assert(args->forward_scene == nullptr);
  IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_;
  
  Ray* oray = (Ray*)args->rayhit;
  oray->org.x = iray->org_x;
  oray->org.y = iray->org_y;
  oray->org.z = iray->org_z;
  oray->dir.x = iray->dir_x;
  oray->dir.y = iray->dir_y;
  oray->dir.z = iray->dir_z;
  oray->tnear() = iray->tnear;
  args->forward_scene = scene;
}

SYCL_EXTERNAL void rtcOccluded1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRay* ray)
{
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcOccludedRTHW(hscene, context, ray, &args);
}

SYCL_EXTERNAL void rtcOccludedEx1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRay* ray, struct RTCIntersectArguments* args)
{
  rtcOccludedRTHW(hscene, context, ray, args);
}

SYCL_EXTERNAL void rtcForwardOccluded1(const RTCOccludedFunctionNArguments* args_, RTCScene scene, struct RTCRay* iray)
{
  assert(args->N == 1);
  assert(args->forward_scene == nullptr);
  OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_;
   
  Ray* oray = (Ray*)args->ray;
  oray->org.x = iray->org_x;
  oray->org.y = iray->org_y;
  oray->org.z = iray->org_z;
  oray->dir.x = iray->dir_x;
  oray->dir.y = iray->dir_y;
  oray->dir.z = iray->dir_z;
  oray->tnear() = iray->tnear;
  args->forward_scene = scene;
}

SYCL_EXTERNAL void* rtcGetGeometryUserDataFromScene (RTCScene hscene, unsigned int geomID)
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

SYCL_EXTERNAL void rtcFilterIntersection(const RTCIntersectFunctionNArguments* args_i, const RTCFilterFunctionNArguments* filter_args)
{
  IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_i;
  isa::reportIntersection1(args, filter_args);
}

SYCL_EXTERNAL void rtcFilterOcclusion(const RTCOccludedFunctionNArguments* args_i, const RTCFilterFunctionNArguments* filter_args)
{
  OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_i;
  isa::reportOcclusion1(args,filter_args);
}

#endif

RTC_NAMESPACE_END;
