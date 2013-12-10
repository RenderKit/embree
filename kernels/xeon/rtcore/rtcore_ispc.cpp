// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "common/default.h"
#include "scene.h"

namespace embree
{
#define size_t int  // FIXME: workaround of ISPC bug

#define CATCH_BEGIN try {
#define CATCH_END                                                       \
  } catch (std::bad_alloc&) {                                         \
  if (VERBOSE) std::cerr << "RTCore: Out of memory" << std::endl;     \
  recordError(RTC_OUT_OF_MEMORY);                                       \
 } catch (std::exception& e) {                                          \
  if (VERBOSE) std::cerr << "RTCore: " << e.what() << std::endl;      \
  recordError(RTC_UNKNOWN_ERROR);                                       \
 } catch (...) {                                                        \
  if (VERBOSE) std::cerr << "RTCore: Unknown exception caught." << std::endl; \
  recordError(RTC_UNKNOWN_ERROR);                                       \
 }

#define VERIFY_HANDLE(handle) \
  if (handle == NULL) {                                                 \
    if (VERBOSE) std::cerr << "RTCore: invalid argument" << std::endl; \
    recordError(RTC_INVALID_ARGUMENT);                                  \
  }

#define VERIFY_GEOMID(id) \
  if (id == -1) {                                                 \
    if (VERBOSE) std::cerr << "RTCore: invalid argument" << std::endl; \
    recordError(RTC_INVALID_ARGUMENT);                                  \
  }

#define TRACE(x) //std::cout << #x << std::endl;

  extern "C" void ispcInit(const char* cfg) {
    rtcInit(cfg);
  }
  
  extern "C" void ispcExit() {
    rtcExit();
  }
  
  extern "C" RTCError ispcGetError() {
    return rtcGetError();
  }
  
  extern "C" void ispcDebug() {
    rtcDebug();
  }
  
  extern "C" RTCScene ispcNewScene (RTCFlags flags, RTCAlgorithmFlags aflags) 
  {
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCFlags(flags | RTC_COHERENT);
    return rtcNewScene(flags,aflags);
  }
  
  extern "C" void ispcCommitScene (RTCScene scene) {
    return rtcCommit(scene);
  }
  
  extern "C" void ispcIntersect1 (RTCScene scene, RTCRay& ray) {
    rtcIntersect(scene,ray);
  }
  
  extern "C" void ispcIntersect4 (const void* valid, RTCScene scene, RTCRay4& ray) {
    rtcIntersect4(valid,scene,ray);
  }
  
  extern "C" void ispcIntersect8 (const void* valid, RTCScene scene, RTCRay8& ray) {
    rtcIntersect8(valid,scene,ray);
  }
  
  extern "C" void ispcIntersect16 (const void* valid, RTCScene scene, RTCRay16& ray) {
    rtcIntersect16(valid,scene,ray);
  }
  
  extern "C" void ispcOccluded1 (RTCScene scene, RTCRay& ray) {
    rtcOccluded(scene,ray);
  }
  
  extern "C" void ispcOccluded4 (const void* valid, RTCScene scene, RTCRay4& ray) {
    rtcOccluded4(valid,scene,ray);
  }
  
  extern "C" void ispcOccluded8 (const void* valid, RTCScene scene, RTCRay8& ray) {
    rtcOccluded8(valid,scene,ray);
  }
  
  extern "C" void ispcOccluded16 (const void* valid, RTCScene scene, RTCRay16& ray) {
    rtcOccluded16(valid,scene,ray);
  }
  
  extern "C" void ispcDeleteScene (RTCScene scene) {
    rtcDeleteScene(scene);
  }
  
  extern "C" unsigned ispcNewInstance (RTCScene target, RTCScene source) {
    return rtcNewInstance(target,source);
  }
  
  extern "C" void ispcSetTransform (RTCScene scene, unsigned geomID, RTCMatrixType layout, float* xfm) {
    return rtcSetTransform(scene,geomID,layout,xfm);
  }
  
  extern "C" unsigned ispcNewUserGeometry (RTCScene scene) {
    return rtcNewUserGeometry(scene);
  }
  
  extern "C" unsigned ispcNewTriangleMesh (RTCScene scene, RTCFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCFlags(flags | RTC_COHERENT);
    return rtcNewTriangleMesh((RTCScene)scene,flags,numTriangles,numVertices,numTimeSteps);
  }
  
/*  extern "C" unsigned ispcNewQuadraticBezierCurves (RTCScene scene, RTCFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCFlags(flags | RTC_COHERENT);
    return rtcNewQuadraticBezierCurves(scene,flags,numCurves,numVertices,numTimeSteps);
    }*/
  
  extern "C" void ispcSetRayMask (RTCScene scene, unsigned geomID, int mask) {
    rtcSetMask(scene,geomID,mask);
  }
  
  extern "C" void* ispcMapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) {
    return rtcMapBuffer(scene,geomID,type);
  }
  
  extern "C" void ispcUnmapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) {
    rtcUnmapBuffer(scene,geomID,type);
  }
  
  extern "C" void ispcEnable (RTCScene scene, unsigned geomID) {
    rtcEnable(scene,geomID);
  }
  
  extern "C" void ispcModified (RTCScene scene, unsigned geomID) {
    rtcUpdate(scene,geomID);
  }
  
  extern "C" void ispcDisable (RTCScene scene, unsigned geomID) {
    rtcDisable(scene,geomID);
  }
  
  extern "C" void ispcDeleteGeometry (RTCScene scene, unsigned geomID) {
    rtcDeleteGeometry(scene,geomID);
  }
  
  extern "C" void ispcSetBounds (RTCScene scene, unsigned geomID, 
                                 float lower_x, float lower_y, float lower_z,
                                 float upper_x, float upper_y, float upper_z)
  {
    rtcSetBounds(scene,geomID,lower_x,lower_y,lower_z,upper_x,upper_y,upper_z);
  }
  
  extern "C" void ispcSetUserData (RTCScene scene, unsigned geomID, void* ptr) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetUserData);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setUserData(ptr,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetIntersectFunction1 (RTCScene scene, unsigned geomID, RTCIntersectFunc intersect) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction1);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setIntersectFunction(intersect,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetIntersectFunction4 (RTCScene scene, unsigned geomID, RTCIntersectFunc4 intersect) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction4);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setIntersectFunction4(intersect,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetIntersectFunction8 (RTCScene scene, unsigned geomID, RTCIntersectFunc8 intersect) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction8);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setIntersectFunction8(intersect,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetIntersectFunction16 (RTCScene scene, unsigned geomID, RTCIntersectFunc16 intersect) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetIntersectFunction16);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setIntersectFunction16(intersect,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetOccludedFunction1 (RTCScene scene, unsigned geomID, RTCOccludedFunc occluded) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction1);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setOccludedFunction(occluded,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetOccludedFunction4 (RTCScene scene, unsigned geomID, RTCOccludedFunc4 occluded) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction4);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setOccludedFunction4(occluded,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetOccludedFunction8 (RTCScene scene, unsigned geomID, RTCOccludedFunc8 occluded) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction8);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setOccludedFunction8(occluded,true);
    CATCH_END;
  }
  
  extern "C" void ispcSetOccludedFunction16 (RTCScene scene, unsigned geomID, RTCOccludedFunc16 occluded) 
  {
    CATCH_BEGIN;
    TRACE(rtcSetOccludedFunction16);
    VERIFY_HANDLE(scene);
    VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->setOccludedFunction16(occluded,true);
    CATCH_END;
  }
}
