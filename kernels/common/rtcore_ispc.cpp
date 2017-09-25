// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "default.h"
#include "scene.h"

DISABLE_DEPRECATED_WARNING;

namespace embree
{
  /* FIXME: why do we need ispcNewScene2? */
  extern "C" RTCScene ispcNewScene2 (RTCDevice device, RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_COHERENT);
    return rtcDeviceNewScene(device,flags,aflags);
  }

  extern "C" void ispcIntersect1 (RTCScene scene, const RTCIntersectContext* context, RTCRay& ray) {
    rtcIntersect1(scene,context,ray);
  }
  
  extern "C" void ispcIntersect4 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay4& ray) {
    rtcIntersect4(valid,scene,context,ray);
  }
  
  extern "C" void ispcIntersect8 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay8& ray) {
    rtcIntersect8(valid,scene,context,ray);
  }
  
  extern "C" void ispcIntersect16 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay16& ray) {
    rtcIntersect16(valid,scene,context,ray);
  }

  extern "C" void ispcIntersect1M (RTCScene scene, const RTCIntersectContext* context, RTCRay* rays, const size_t N, const size_t M, const size_t stride) {
    rtcIntersect1M(scene,context,rays,M,stride);
  }

  extern "C" void ispcIntersect1Mp (RTCScene scene, const RTCIntersectContext* context, RTCRay** rays, const size_t N, const size_t M, const size_t stride) {
    rtcIntersect1Mp(scene,context,rays,M);
  }

  extern "C" void ispcIntersectNM (RTCScene scene, const RTCIntersectContext* context, RTCRayN* rays, const size_t N, const size_t M, const size_t stride) {
    rtcIntersectNM(scene,context,rays,N,M,stride);
  }

  extern "C" void ispcIntersectNp (RTCScene scene, const RTCIntersectContext* context, const RTCRayNp& rays, const  size_t N) {
    rtcIntersectNp(scene,context,rays,N);
  }
  
  extern "C" void ispcOccluded1 (RTCScene scene, const RTCIntersectContext* context, RTCRay& ray) {
    rtcOccluded1(scene,context,ray);
  }
  
  extern "C" void ispcOccluded4 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay4& ray) {
    rtcOccluded4(valid,scene,context,ray);
  }
  
  extern "C" void ispcOccluded8 (const void* valid, RTCScene scene, const RTCIntersectContext* context,  RTCRay8& ray) {
    rtcOccluded8(valid,scene,context,ray);
  }
  
  extern "C" void ispcOccluded16 (const void* valid, RTCScene scene, const RTCIntersectContext* context, RTCRay16& ray) {
    rtcOccluded16(valid,scene,context,ray);
  }

  extern "C" void ispcOccluded1M (RTCScene scene, const RTCIntersectContext* context, RTCRay* rays, const size_t N, const size_t M, const size_t stride) {
    rtcOccluded1M(scene,context,rays,M,stride);
  }

  extern "C" void ispcOccluded1Mp (RTCScene scene, const RTCIntersectContext* context, RTCRay** rays, const size_t N, const size_t M, const size_t stride) {
    rtcOccluded1Mp(scene,context,rays,M);
  }

  extern "C" void ispcOccludedNM (RTCScene scene, const RTCIntersectContext* context, RTCRayN* rays, const size_t N, const  size_t M, const  size_t stride) {
    rtcOccludedNM(scene,context,(RTCRayN*)rays,N,M,stride);
  }

  extern "C" void ispcOccludedNp (RTCScene scene, const RTCIntersectContext* context, const RTCRayNp& rays, const  size_t N) {
    rtcOccludedNp(scene,context,rays,N);
  }
  
  extern "C" void ispcSetIntersectFunction1 (RTCGeometry hgeometry, RTCIntersectFunc intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction1);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunction(intersect,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetIntersectFunction4 (RTCGeometry hgeometry, RTCIntersectFunc4 intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction4);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunction4(intersect,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetIntersectFunction8 (RTCGeometry hgeometry, RTCIntersectFunc8 intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction8);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunction8(intersect,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetIntersectFunction16 (RTCGeometry hgeometry, RTCIntersectFunc16 intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction16);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunction16(intersect,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetOccludedFunction1 (RTCGeometry hgeometry, RTCOccludedFunc occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction1);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunction(occluded,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetOccludedFunction4 (RTCGeometry hgeometry, RTCOccludedFunc4 occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction4);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunction4(occluded,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetOccludedFunction8 (RTCGeometry hgeometry, RTCOccludedFunc8 occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction8);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunction8(occluded,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetOccludedFunction16 (RTCGeometry hgeometry, RTCOccludedFunc16 occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction16);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunction16(occluded,true);
    RTCORE_CATCH_END2(geometry);
  }
  
  extern "C" void ispcSetIntersectionFilterFunction (RTCGeometry hgeometry, RTCFilterFuncN filter) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectionFilterFunctionN(filter);
    RTCORE_CATCH_END2(geometry);
  }
  
  // extern "C" void ispcSetIntersectionFilterFunction4 (RTCGeometry hgeometry, RTCFilterFunc4 filter) 
  // {
  //   Ref<Geometry> geometry = (Geometry*) hgeometry;
  //   RTCORE_CATCH_BEGIN;
  //   RTCORE_TRACE(rtcSetIntersectionFilterFunction4);
  //   RTCORE_VERIFY_HANDLE(hgeometry);
  //   geometry->setIntersectionFilterFunction4(filter,true);
  //   RTCORE_CATCH_END2(geometry);
  // }
  
  // extern "C" void ispcSetIntersectionFilterFunction8 (RTCGeometry hgeometry, RTCFilterFunc8 filter) 
  // {
  //   Ref<Geometry> geometry = (Geometry*) hgeometry;
  //   RTCORE_CATCH_BEGIN;
  //   RTCORE_TRACE(rtcSetIntersectionFilterFunction8);
  //   RTCORE_VERIFY_HANDLE(hgeometry);
  //   geometry->setIntersectionFilterFunction8(filter,true);
  //   RTCORE_CATCH_END2(geometry);
  // }
  
  // extern "C" void ispcSetIntersectionFilterFunction16 (RTCGeometry hgeometry, RTCFilterFunc16 filter) 
  // {
  //   Ref<Geometry> geometry = (Geometry*) hgeometry;
  //   RTCORE_CATCH_BEGIN;
  //   RTCORE_TRACE(rtcSetIntersectionFilterFunction16);
  //   RTCORE_VERIFY_HANDLE(hgeometry);
  //   geometry->setIntersectionFilterFunction16(filter,true);
  //   RTCORE_CATCH_END2(geometry);
  // }

  extern "C" void ispcSetOcclusionFilterFunction (RTCGeometry hgeometry, RTCFilterFuncN filter) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOcclusionFilterFunctionN(filter);
    RTCORE_CATCH_END2(geometry);
  }
  
  // extern "C" void ispcSetOcclusionFilterFunction4 (RTCGeometry hgeometry, RTCFilterFunc4 filter) 
  // {
  //   Ref<Geometry> geometry = (Geometry*) hgeometry;
  //   RTCORE_CATCH_BEGIN;
  //   RTCORE_TRACE(rtcSetOcclusionFilterFunction4);
  //   RTCORE_VERIFY_HANDLE(hgeometry);
  //   geometry->setOcclusionFilterFunction4(filter,true);
  //   RTCORE_CATCH_END2(geometry);
  // }
  
  // extern "C" void ispcSetOcclusionFilterFunction8 (RTCGeometry hgeometry, RTCFilterFunc8 filter) 
  // {
  //   Ref<Geometry> geometry = (Geometry*) hgeometry;
  //   RTCORE_CATCH_BEGIN;
  //   RTCORE_TRACE(rtcSetOcclusionFilterFunction8);
  //   RTCORE_VERIFY_HANDLE(hgeometry);
  //   geometry->setOcclusionFilterFunction8(filter,true);
  //   RTCORE_CATCH_END2(geometry);
  // }
  
  // extern "C" void ispcSetOcclusionFilterFunction16 (RTCGeometry hgeometry, RTCFilterFunc16 filter) 
  // {
  //   Ref<Geometry> geometry = (Geometry*) hgeometry;
  //   RTCORE_CATCH_BEGIN;
  //   RTCORE_TRACE(rtcSetOcclusionFilterFunction16);
  //   RTCORE_VERIFY_HANDLE(hgeometry);
  //   geometry->setOcclusionFilterFunction16(filter,true);
  //   RTCORE_CATCH_END2(geometry);
  // }

  extern "C" void ispcInterpolateN(RTCScene scene, unsigned int geomID, 
                                   const void* valid, const unsigned int* primIDs, const float* u, const float* v, size_t numUVs, 
                                   RTCBufferType buffer, 
                                   float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    rtcInterpolateN(scene,geomID,valid,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,numFloats);
  }

  extern "C" void ispcInterpolateN2(RTCScene scene, unsigned int geomID, 
                                    const void* valid, const unsigned int* primIDs, const float* u, const float* v, size_t numUVs, 
                                    RTCBufferType buffer, 
                                    float* P, float* dPdu, float* dPdv,
                                    float* ddPdudu, float* ddPdvdv, float* ddPdudv,
                                    size_t numFloats)
  {
    rtcInterpolateN2(scene,geomID,valid,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats);
  }
}
