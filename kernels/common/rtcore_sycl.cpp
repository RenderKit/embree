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

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcIntersectEx1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRayHit* rayhit, struct RTCIntersectArguments* args)
{
  rtcIntersectRTHW(hscene, context, rayhit, args); 
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcForwardIntersect1(const RTCIntersectFunctionNArguments* args_, RTCScene scene, struct RTCRay* iray)
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

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcOccluded1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRay* ray)
{
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcOccludedRTHW(hscene, context, ray, &args);
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcOccludedEx1(RTCScene hscene, struct RTCIntersectContext* context, struct RTCRay* ray, struct RTCIntersectArguments* args)
{
  rtcOccludedRTHW(hscene, context, ray, args);
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcForwardOccluded1(const RTCOccludedFunctionNArguments* args_, RTCScene scene, struct RTCRay* iray)
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

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcInterpolate(const struct RTCInterpolateArguments* args)
{
  Geometry* geometry = (Geometry*) args->geometry;

  switch (geometry->getType())
  {
  case Geometry::GTY_FLAT_LINEAR_CURVE: 
  case Geometry::GTY_ROUND_LINEAR_CURVE: 
  case Geometry::GTY_ORIENTED_LINEAR_CURVE: 
  case Geometry::GTY_CONE_LINEAR_CURVE: ((LineSegments*)geometry)->interpolate_impl<1>(args); break;
      
  case Geometry::GTY_FLAT_BEZIER_CURVE:
  case Geometry::GTY_ROUND_BEZIER_CURVE:
  case Geometry::GTY_ORIENTED_BEZIER_CURVE: ((isa::CurveGeometryInterface<BezierCurveT>*)geometry)->interpolate_impl<1>(args); break;
      
  case Geometry::GTY_FLAT_BSPLINE_CURVE:
  case Geometry::GTY_ROUND_BSPLINE_CURVE:
  case Geometry::GTY_ORIENTED_BSPLINE_CURVE: ((isa::CurveGeometryInterface<BSplineCurveT>*)geometry)->interpolate_impl<1>(args); break;

  case Geometry::GTY_FLAT_HERMITE_CURVE:
  case Geometry::GTY_ROUND_HERMITE_CURVE:
  case Geometry::GTY_ORIENTED_HERMITE_CURVE: ((isa::HermiteCurveGeometryInterface<HermiteCurveT>*)geometry)->interpolate_impl<1>(args); break;
      
  case Geometry::GTY_FLAT_CATMULL_ROM_CURVE:
  case Geometry::GTY_ROUND_CATMULL_ROM_CURVE:
  case Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE: ((isa::CurveGeometryInterface<CatmullRomCurveT>*)geometry)->interpolate_impl<1>(args); break;

  case Geometry::GTY_TRIANGLE_MESH: ((TriangleMesh*)geometry)->interpolate_impl<1>(args); break;
  case Geometry::GTY_QUAD_MESH: ((QuadMesh*)geometry)->interpolate_impl<1>(args); break;
  case Geometry::GTY_GRID_MESH: ((GridMesh*)geometry)->interpolate_impl<1>(args); break;
  default: break;
  };
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL RTCGeometry rtcGetGeometry (RTCScene hscene, unsigned int geomID)
{
  Scene* scene = (Scene*) hscene;
  //RTC_CATCH_BEGIN;
  //RTC_TRACE(rtcGetGeometry);
#if defined(DEBUG)
  //RTC_VERIFY_HANDLE(hscene);
  //RTC_VERIFY_GEOMID(geomID);
#endif
  //RTC_ENTER_DEVICE(hscene); // do not enable for performance reasons
  return (RTCGeometry) scene->get(geomID);
  //RTC_CATCH_END2(scene);
  //return nullptr;
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void* rtcGetGeometryUserData (RTCGeometry hgeometry)
{
  Geometry* geometry = (Geometry*) hgeometry; // no ref counting here!
  //RTC_CATCH_BEGIN;
  //RTC_TRACE(rtcGetGeometryUserData);
  //RTC_VERIFY_HANDLE(hgeometry);
  ////RTC_ENTER_DEVICE(hgeometry); // do not enable for performance reasons !
  return geometry->getUserData();
  //RTC_CATCH_END2(geometry);
  //return nullptr;
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcFilterIntersection(const RTCIntersectFunctionNArguments* args_i, const RTCFilterFunctionNArguments* filter_args)
{
  IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_i;
  isa::reportIntersection1(args, filter_args);
}

EMBREE_SYCL_SIMD_N SYCL_EXTERNAL void rtcFilterOcclusion(const RTCOccludedFunctionNArguments* args_i, const RTCFilterFunctionNArguments* filter_args)
{
  OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_i;
  isa::reportOcclusion1(args,filter_args);
}

#endif

RTC_NAMESPACE_END;

