// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

namespace embree
{
#define size_t int  // FIXME: workaround for ISPC bug
#define ssize_t int  // FIXME: workaround for ISPC bug

  extern "C" RTCDevice ispcNewDevice(const char* cfg) {
    return rtcNewDevice(cfg);
  }

  extern "C" void ispcDeleteDevice(RTCDevice device) {
    rtcDeleteDevice(device);
  }

  extern "C" void ispcInit(const char* cfg) {
    rtcInit(cfg);
  }
  
  extern "C" void ispcExit() {
    rtcExit();
  }

  extern "C" void ispcSetParameter1i(const RTCParameter parm, ssize_t val) {
    rtcSetParameter1i(parm,val);
  }

  extern "C" void ispcDeviceSetParameter1i(RTCDevice device, const RTCParameter parm, ssize_t val) {
    rtcDeviceSetParameter1i(device,parm,val);
  }

  extern "C" RTCError ispcGetError() {
    return rtcGetError();
  }

  extern "C" RTCError ispcDeviceGetError(RTCDevice device) {
    return rtcDeviceGetError(device);
  }

  extern "C" void ispcSetErrorFunction(void* f) {
    return rtcSetErrorFunction((RTCErrorFunc)f);
  }

  extern "C" void ispcDeviceSetErrorFunction(RTCDevice device, void* f) {
    return rtcDeviceSetErrorFunction(device,(RTCErrorFunc)f);
  }

  extern "C" void ispcSetMemoryMonitorFunction(void* f) {
    return rtcSetMemoryMonitorFunction((RTCMemoryMonitorFunc)f);
  }

  extern "C" void ispcDeviceSetMemoryMonitorFunction(RTCDevice device, void* f) {
    return rtcDeviceSetMemoryMonitorFunction(device,(RTCMemoryMonitorFunc)f);
  }

  extern "C" void ispcDebug() {
    rtcDebug();
  }
  
  extern "C" RTCScene ispcNewScene (RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_COHERENT);
    return rtcNewScene(flags,aflags);
  }
 
  extern "C" RTCScene ispcNewScene2 (RTCDevice device, RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_COHERENT);
    return rtcDeviceNewScene(device,flags,aflags);
  }

  extern "C" void ispcSetProgressMonitorFunction(RTCScene scene, void* func, void* ptr) {
    return rtcSetProgressMonitorFunction(scene,(RTCProgressMonitorFunc)func,ptr);
  }

  extern "C" void ispcCommitScene (RTCScene scene) {
    return rtcCommit(scene);
  }

  extern "C" void ispcCommitSceneThread (RTCScene scene, unsigned int threadID, unsigned int numThreads) {
    return rtcCommitThread(scene,threadID,numThreads);
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
  
  extern "C" void ispcSetTransform (RTCScene scene, unsigned geomID, RTCMatrixType layout, const float* xfm) {
    return rtcSetTransform(scene,geomID,layout,xfm);
  }
  
  extern "C" unsigned ispcNewUserGeometry (RTCScene scene, size_t numItems) {
    return rtcNewUserGeometry(scene,numItems);
  }
  
  extern "C" unsigned ispcNewTriangleMesh (RTCScene scene, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) {
    return rtcNewTriangleMesh((RTCScene)scene,flags,numTriangles,numVertices,numTimeSteps);
  }
  
  extern "C" unsigned ispcNewBezierCurves (RTCScene scene, RTCGeometryFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) {
    return rtcNewHairGeometry(scene,flags,numCurves,numVertices,numTimeSteps);
  }

  extern "C" unsigned ispcNewSubdivisionMesh (RTCScene scene, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, 
                                              size_t numVertices, size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps) 
  {
    return rtcNewSubdivisionMesh(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
  }

  extern "C" void ispcSetRayMask (RTCScene scene, unsigned geomID, int mask) {
    rtcSetMask(scene,geomID,mask);
  }

  extern "C" void ispcSetBoundaryMode (RTCScene scene, unsigned geomID, RTCBoundaryMode mode) {
    rtcSetBoundaryMode(scene,geomID,mode);
  }
  
  extern "C" void* ispcMapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) {
    return rtcMapBuffer(scene,geomID,type);
  }
  
  extern "C" void ispcUnmapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) {
    rtcUnmapBuffer(scene,geomID,type);
  }

  extern "C" void ispcSetBuffer(RTCScene scene, unsigned geomID, RTCBufferType type, void* ptr, size_t offset, size_t stride) {
    rtcSetBuffer(scene,geomID,type,ptr,offset,stride);
  }

  extern "C" void ispcEnable (RTCScene scene, unsigned geomID) {
    rtcEnable(scene,geomID);
  }
  
  extern "C" void ispcUpdate (RTCScene scene, unsigned geomID) {
    rtcUpdate(scene,geomID);
  }

  extern "C" void ispcUpdateBuffer (RTCScene scene, unsigned geomID, RTCBufferType type) {
    rtcUpdateBuffer(scene,geomID,type);
  }
  
  extern "C" void ispcDisable (RTCScene scene, unsigned geomID) {
    rtcDisable(scene,geomID);
  }
  
  extern "C" void ispcDeleteGeometry (RTCScene scene, unsigned geomID) {
    rtcDeleteGeometry(scene,geomID);
  }
    
  extern "C" void ispcSetUserData (RTCScene hscene, unsigned geomID, void* ptr) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetUserData);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setUserData(ptr);
    RTCORE_CATCH_END(scene->device);
  }

  extern "C" void* ispcGetUserData (RTCScene hscene, unsigned geomID)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetUserData);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    return scene->get(geomID)->getUserData(); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->device);
    return nullptr;
  }

  extern "C" void ispcSetBoundsFunction (RTCScene scene, unsigned geomID, RTCBoundsFunc bounds) {
    rtcSetBoundsFunction(scene,geomID,bounds);
  }

  extern "C" void ispcSetIntersectFunction1 (RTCScene hscene, unsigned geomID, RTCIntersectFunc intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction1);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction(intersect,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetIntersectFunction4 (RTCScene hscene, unsigned geomID, RTCIntersectFunc4 intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction4(intersect,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetIntersectFunction8 (RTCScene hscene, unsigned geomID, RTCIntersectFunc8 intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction8(intersect,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetIntersectFunction16 (RTCScene hscene, unsigned geomID, RTCIntersectFunc16 intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction16(intersect,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOccludedFunction1 (RTCScene hscene, unsigned geomID, RTCOccludedFunc occluded) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction1);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction(occluded,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOccludedFunction4 (RTCScene hscene, unsigned geomID, RTCOccludedFunc4 occluded) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction4(occluded,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOccludedFunction8 (RTCScene hscene, unsigned geomID, RTCOccludedFunc8 occluded) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction8(occluded,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOccludedFunction16 (RTCScene hscene, unsigned geomID, RTCOccludedFunc16 occluded) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction16(occluded,true);
    RTCORE_CATCH_END(scene->device);
  }

  extern "C" void ispcSetIntersectionFilterFunction1 (RTCScene hscene, unsigned geomID, RTCFilterFunc filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction1);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction(filter,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetIntersectionFilterFunction4 (RTCScene hscene, unsigned geomID, RTCFilterFunc4 filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction4(filter,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetIntersectionFilterFunction8 (RTCScene hscene, unsigned geomID, RTCFilterFunc8 filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction8(filter,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetIntersectionFilterFunction16 (RTCScene hscene, unsigned geomID, RTCFilterFunc16 filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction16(filter,true);
    RTCORE_CATCH_END(scene->device);
  }

  extern "C" void ispcSetOcclusionFilterFunction1 (RTCScene hscene, unsigned geomID, RTCFilterFunc filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction1);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction(filter,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOcclusionFilterFunction4 (RTCScene hscene, unsigned geomID, RTCFilterFunc4 filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction4(filter,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOcclusionFilterFunction8 (RTCScene hscene, unsigned geomID, RTCFilterFunc8 filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction8(filter,true);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcSetOcclusionFilterFunction16 (RTCScene hscene, unsigned geomID, RTCFilterFunc16 filter) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction16(filter,true);
    RTCORE_CATCH_END(scene->device);
  }

  extern "C" void ispcSetDisplacementFunction (RTCScene hscene, unsigned int geomID, void* func, RTCBounds* bounds)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDisplacementFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setDisplacementFunction((RTCDisplacementFunc)func,bounds);
    RTCORE_CATCH_END(scene->device);
  }
  
  extern "C" void ispcInterpolateN(RTCScene scene, unsigned int geomID, 
                                   const void* valid, const unsigned int* primIDs, const float* u, const float* v, size_t numUVs, 
                                   RTCBufferType buffer, 
                                   float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    rtcInterpolateN(scene,geomID,valid,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,numFloats);
  }
}


