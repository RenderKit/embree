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
  
  
  extern "C" void ispcSetIntersectFunction (RTCGeometry hgeometry, RTCIntersectFuncN intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunctionN(intersect);
    RTCORE_CATCH_END2(geometry);
  }
    
  extern "C" void ispcSetOccludedFunction (RTCGeometry hgeometry, RTCOccludedFuncN occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunctionN(occluded);
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
  
  extern "C" void ispcSetOcclusionFilterFunction (RTCGeometry hgeometry, RTCFilterFuncN filter) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOcclusionFilterFunctionN(filter);
    RTCORE_CATCH_END2(geometry);
  }
  
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
