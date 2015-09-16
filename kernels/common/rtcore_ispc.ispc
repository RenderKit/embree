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

#include "../../include/embree2/rtcore.isph" 

#define size_tt int // FIXME: workaround of ISPC bug
#define ssize_tt int // FIXME: workaround of ISPC bug

extern "C" RTCDevice ispcNewDevice(const uniform int8* uniform cfg);
extern "C" void ispcDeleteDevice(RTCDevice device);
extern "C" void ispcInit(const uniform int8* uniform cfg);
extern "C" void ispcExit();
extern "C" void ispcSetParameter1i(const uniform RTCParameter parm, uniform ssize_tt val);
extern "C" void ispcDeviceSetParameter1i(RTCDevice device, const uniform RTCParameter parm, uniform ssize_tt val);
extern "C" uniform RTCError ispcGetError ();
extern "C" uniform RTCError ispcDeviceGetError (RTCDevice device);
extern "C" void ispcSetErrorFunction (void* uniform ptr);
extern "C" void ispcDeviceSetErrorFunction (RTCDevice device, void* uniform ptr);
extern "C" void ispcSetMemoryMonitorFunction (void* uniform ptr);
extern "C" void ispcDeviceSetMemoryMonitorFunction (RTCDevice device, void* uniform ptr);
extern "C" void ispcDebug();
extern "C" RTCScene ispcNewScene (uniform RTCSceneFlags flags, uniform RTCAlgorithmFlags aflags);
extern "C" RTCScene ispcNewScene2 (RTCDevice device, uniform RTCSceneFlags flags, uniform RTCAlgorithmFlags aflags);
extern "C" void ispcSetProgressMonitorFunction (RTCScene scene, void* uniform func, void* uniform ptr);
extern "C" void ispcCommitScene (RTCScene scene);
extern "C" void ispcCommitSceneThread (RTCScene scene, uniform unsigned int threadID, uniform unsigned int numThreads);
extern "C" void ispcIntersect1 (RTCScene scene, uniform RTCRay1& ray);
extern "C" void ispcIntersect4 (void* uniform valid, RTCScene scene, void* uniform ray);
extern "C" void ispcIntersect8 (void* uniform valid, RTCScene scene, void* uniform ray);
extern "C" void ispcIntersect16 (void* uniform valid, RTCScene scene, void* uniform ray);
extern "C" void ispcOccluded1 (RTCScene scene, uniform RTCRay1& ray);
extern "C" void ispcOccluded4 (void* uniform valid, RTCScene scene, void* uniform ray);
extern "C" void ispcOccluded8 (void* uniform valid, RTCScene scene, void* uniform ray);
extern "C" void ispcOccluded16 (void* uniform valid, RTCScene scene, void* uniform ray);
extern "C" void ispcDeleteScene (RTCScene scene);
extern "C" uniform unsigned int ispcNewInstance (RTCScene target, RTCScene source);
extern "C" void ispcSetTransform (RTCScene scene, uniform unsigned int geomID, uniform RTCMatrixType layout, const uniform float* uniform xfm);
extern "C" uniform unsigned int ispcNewUserGeometry (RTCScene scene, uniform size_tt numItems);
extern "C" uniform unsigned int ispcNewTriangleMesh (RTCScene scene,
                                                 uniform RTCGeometryFlags flags,
                                                 uniform size_tt numTriangles,
                                                 uniform size_tt numVertices,
                                                 uniform size_tt numTimeSteps);
extern "C" uniform unsigned int ispcNewBezierCurves (RTCScene scene,
                                                              uniform RTCGeometryFlags flags,
                                                              uniform size_tt numCurves,
                                                              uniform size_tt numVertices,
                                                              uniform size_tt numTimeSteps);

extern "C" uniform unsigned int ispcNewSubdivisionMesh (RTCScene scene,
                                                        uniform RTCGeometryFlags flags,
                                                        uniform size_tt numFaces,
                                                        uniform size_tt numEdges,
                                                        uniform size_tt numVertices,
                                                        uniform size_tt numCreases,
                                                        uniform size_tt numCorners,
                                                        uniform size_tt numHoles,
                                                        uniform size_tt numTimeSteps);

extern "C" void ispcSetRayMask (RTCScene scene, uniform unsigned int geomID, uniform int mask);
extern "C" void ispcSetBoundaryMode(RTCScene scene, uniform unsigned int geomID, uniform size_tt mask);
extern "C" void* uniform ispcMapBuffer(RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type);
extern "C" void ispcUnmapBuffer(RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type);
extern "C" void ispcSetBuffer(RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type, void* uniform ptr, uniform size_tt offset, uniform size_tt stride);
extern "C" void ispcEnable (RTCScene scene, uniform unsigned int geomID);
extern "C" void ispcUpdate (RTCScene scene, uniform unsigned int geomID);
extern "C" void ispcUpdateBuffer (RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type);
extern "C" void ispcDisable (RTCScene scene, uniform unsigned int geomID);
extern "C" void ispcDeleteGeometry (RTCScene scene, uniform unsigned int geomID);

extern "C" void ispcSetBoundsFunction (RTCScene scene, uniform unsigned int geomID, void* uniform bounds);
extern "C" void ispcSetUserData (RTCScene scene, uniform unsigned int geomID, void* uniform ptr);
extern "C" void* uniform ispcGetUserData (RTCScene scene, uniform unsigned int geomID);

extern "C" void ispcSetIntersectFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect);
extern "C" void ispcSetIntersectFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect); 
extern "C" void ispcSetIntersectFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect);
extern "C" void ispcSetIntersectFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform intersect); 

extern "C" void ispcSetOccludedFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);
extern "C" void ispcSetOccludedFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform occluded);

extern "C" void ispcSetIntersectionFilterFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetIntersectionFilterFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);

extern "C" void ispcSetOcclusionFilterFunction1 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunction4 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunction8 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);
extern "C" void ispcSetOcclusionFilterFunction16 (RTCScene scene, uniform unsigned int geomID, void* uniform filter);

extern "C" void ispcSetDisplacementFunction (RTCScene scene, uniform unsigned int geomID, void *uniform func, uniform RTCBounds* uniform bounds);

extern "C" void ispcInterpolateN(RTCScene scene, uniform unsigned int geomID, 
                                 const void* uniform valid, const uniform unsigned int* uniform primIDs, const uniform float* uniform u, const uniform float* uniform v, uniform size_tt numUVs, 
                                 uniform RTCBufferType buffer, 
                                 uniform float* uniform P, uniform float* uniform dPdu, uniform float* uniform dPdv, uniform size_tt numFloats);

RTCDevice rtcNewDevice(const uniform int8* uniform cfg) {
  return ispcNewDevice(cfg);
}

void rtcDeleteDevice(RTCDevice device) {
  ispcDeleteDevice(device);
}

void rtcInit(const uniform int8* uniform cfg) {
  ispcInit(cfg);
}

void rtcExit() {
  ispcExit();
}

void rtcSetParameter1i(const uniform RTCParameter parm, uniform size_t val) {
  ispcSetParameter1i(parm,val);
}

void rtcDeviceSetParameter1i(RTCDevice device, const uniform RTCParameter parm, uniform size_t val) {
  ispcDeviceSetParameter1i(device,parm,val);
}

uniform RTCError rtcGetError() {
  return ispcGetError();
}

uniform RTCError rtcDeviceGetError(RTCDevice device) {
  return ispcDeviceGetError(device);
}

void rtcSetErrorFunction(uniform RTC_ERROR_FUNCTION func) {
  ispcSetErrorFunction(func);
}

void rtcDeviceSetErrorFunction(RTCDevice device, uniform RTC_ERROR_FUNCTION func) {
  ispcDeviceSetErrorFunction(device,func);
}

void rtcSetMemoryMonitorFunction(uniform RTC_MEMORY_MONITOR_FUNCTION func) {
  ispcSetMemoryMonitorFunction(func);
}

void rtcDeviceSetMemoryMonitorFunction(RTCDevice device, uniform RTC_MEMORY_MONITOR_FUNCTION func) {
  ispcDeviceSetMemoryMonitorFunction(device,func);
}

void rtcDebug() {
  ispcDebug();
}

RTCScene rtcNewScene (uniform RTCSceneFlags flags, uniform RTCAlgorithmFlags aflags) 
{  
  if (aflags & RTC_INTERSECT_VARYING) {
    if (sizeof(varying float) == 16)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 1));
    else if (sizeof(varying float) == 32)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 2));
    else if (sizeof(varying float) == 64)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 3));
  }
  return ispcNewScene(flags,aflags);
}

RTCScene rtcDeviceNewScene (RTCDevice device, uniform RTCSceneFlags flags, uniform RTCAlgorithmFlags aflags) 
{  
  if (aflags & RTC_INTERSECT_VARYING) {
    if (sizeof(varying float) == 16)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 1));
    else if (sizeof(varying float) == 32)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 2));
    else if (sizeof(varying float) == 64)
      aflags = (uniform RTCAlgorithmFlags) ((aflags & ~RTC_INTERSECT_VARYING) | (1 << 3));
  }
  return ispcNewScene2(device,flags,aflags);
}

void rtcSetProgressMonitorFunction(RTCScene scene, uniform RTC_PROGRESS_MONITOR_FUNCTION func, void* uniform ptr) {
  ispcSetProgressMonitorFunction(scene,func,ptr);
}

void rtcCommit (RTCScene scene) {
  ispcCommitScene(scene);
}

void rtcCommitThread (RTCScene scene, uniform unsigned int threadID, uniform unsigned int numThreads) {
  ispcCommitSceneThread(scene,threadID,numThreads);
}

void rtcIntersect1 (RTCScene scene, uniform RTCRay1& ray) {
  ispcIntersect1(scene,ray);
}

void rtcIntersect (RTCScene scene, varying RTCRay& ray) 
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }

  if (sizeof(varying float) == 16)
    ispcIntersect4(&imask,scene,&ray);
  else if (sizeof(varying float) == 32)
    ispcIntersect8(&imask,scene,&ray);
  else if (sizeof(varying float) == 64)
    ispcIntersect16(&imask,scene,&ray);
}

void rtcOccluded1 (RTCScene scene, uniform RTCRay1& ray) {
  ispcOccluded1(scene,ray);
}

void rtcOccluded (RTCScene scene, varying RTCRay& ray) 
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }

  if (sizeof(varying float) == 16)
    ispcOccluded4(&imask,scene,&ray);
  else if (sizeof(varying float) == 32)
    ispcOccluded8(&imask,scene,&ray);
  else if (sizeof(varying float) == 64)
    ispcOccluded16(&imask,scene,&ray);
}

void rtcDeleteScene (RTCScene scene) {
  ispcDeleteScene(scene);
}

uniform unsigned int rtcNewInstance (RTCScene target, RTCScene source) {
  return ispcNewInstance(target,source);
}

void rtcSetTransform (RTCScene scene, uniform unsigned int geomID, uniform RTCMatrixType layout, const uniform float* uniform xfm) {
  ispcSetTransform(scene,geomID,layout,xfm);
}

uniform unsigned int rtcNewUserGeometry (RTCScene scene, uniform size_t numItems) {
  return ispcNewUserGeometry(scene,numItems);
}

uniform unsigned int rtcNewTriangleMesh (RTCScene scene,
                                         uniform RTCGeometryFlags flags,
                                         uniform size_t numTriangles,
                                         uniform size_t numVertices,
                                         uniform size_t numTimeSteps)
{
  return ispcNewTriangleMesh(scene,flags,numTriangles,numVertices,numTimeSteps);
}

uniform unsigned int rtcNewHairGeometry (RTCScene scene,
                                                  uniform RTCGeometryFlags flags,
                                                  uniform size_t numCurves,
                                                  uniform size_t numVertices,
                                                  uniform size_t numTimeSteps)
{
  return ispcNewBezierCurves (scene,flags,numCurves,numVertices,numTimeSteps);
}

uniform unsigned int rtcNewSubdivisionMesh (RTCScene scene,
                                            uniform RTCGeometryFlags flags,
                                            uniform size_t numFaces,
                                            uniform size_t numEdges,
                                            uniform size_t numVertices,
                                            uniform size_t numEdgeCreases, 
                                            uniform size_t numVertexCreases, 
                                            uniform size_t numHoles,
                                            uniform size_t numTimeSteps)
{
  return ispcNewSubdivisionMesh(scene,flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
}


void rtcSetMask (RTCScene scene, uniform unsigned int geomID, uniform int mask) {
  ispcSetRayMask(scene,geomID,mask);
}

void rtcSetBoundaryMode(RTCScene scene, uniform unsigned int geomID, uniform RTCBoundaryMode mode) {
  ispcSetBoundaryMode(scene,geomID,mode);
}

void* uniform rtcMapBuffer(RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type) {
  return ispcMapBuffer(scene,geomID,type);
}

void rtcUnmapBuffer(RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type) {
  ispcUnmapBuffer(scene,geomID,type);
}

void rtcSetBuffer(RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type, void* uniform ptr, uniform size_t offset, uniform size_t stride) {
  ispcSetBuffer(scene,geomID,type,ptr,offset,stride);
}

void rtcEnable (RTCScene scene, uniform unsigned int geomID) {
  ispcEnable(scene,geomID);
}

void rtcUpdate (RTCScene scene, uniform unsigned int geomID) {
  ispcUpdate(scene,geomID);
}

void rtcUpdateBuffer (RTCScene scene, uniform unsigned int geomID, uniform RTCBufferType type) {
  ispcUpdateBuffer(scene,geomID,type);
}

void rtcDisable (RTCScene scene, uniform unsigned int geomID) {
  ispcDisable(scene,geomID);
}

void rtcDeleteGeometry (RTCScene scene, uniform unsigned int geomID) {
  ispcDeleteGeometry(scene,geomID);
}

void rtcSetUserData (RTCScene scene, uniform unsigned int geomID, void* uniform ptr) {
  ispcSetUserData(scene,geomID,ptr);
}

void* uniform rtcGetUserData (RTCScene scene, uniform unsigned int geomID) {
  return ispcGetUserData(scene,geomID);
}

void rtcSetBoundsFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCBoundsFunc bounds) {
  ispcSetBoundsFunction(scene,geomID,bounds);
}

void rtcSetIntersectFunction1 (RTCScene scene, uniform unsigned int geomID, uniform RTCIntersectFuncUniform intersect) {
  ispcSetIntersectFunction1(scene,geomID,intersect);
}

void rtcSetIntersectFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCIntersectFuncVarying intersect)
{
  if (sizeof(varying float) == 16) 
    ispcSetIntersectFunction4(scene,geomID,intersect);
  else if (sizeof(varying float) == 32) 
    ispcSetIntersectFunction8(scene,geomID,intersect);
  else if (sizeof(varying float) == 64) 
    ispcSetIntersectFunction16(scene,geomID,intersect);
}

void rtcSetOccludedFunction1 (RTCScene scene, uniform unsigned int geomID, uniform RTCOccludedFuncUniform occluded) {
  ispcSetOccludedFunction1(scene,geomID,occluded);
}

void rtcSetOccludedFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCOccludedFuncVarying occluded)
{
  if (sizeof(varying float) == 16)
    ispcSetOccludedFunction4(scene,geomID,occluded);
  else if (sizeof(varying float) == 32)
    ispcSetOccludedFunction8(scene,geomID,occluded);
  else if (sizeof(varying float) == 64)
    ispcSetOccludedFunction16(scene,geomID,occluded);
}

void rtcSetIntersectionFilterFunction1 (RTCScene scene, uniform unsigned int geomID, uniform RTCFilterFuncUniform filter) {
  ispcSetIntersectionFilterFunction1(scene,geomID,filter);
}

void rtcSetIntersectionFilterFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCFilterFuncVarying filter)
{
  if (sizeof(varying float) == 16)
    ispcSetIntersectionFilterFunction4(scene,geomID,filter);
  else if (sizeof(varying float) == 32)
    ispcSetIntersectionFilterFunction8(scene,geomID,filter);
  else if (sizeof(varying float) == 64)
    ispcSetIntersectionFilterFunction16(scene,geomID,filter);
}

void rtcSetOcclusionFilterFunction1 (RTCScene scene, uniform unsigned int geomID, uniform RTCFilterFuncUniform filter) {
  ispcSetOcclusionFilterFunction1(scene,geomID,filter);
}

void rtcSetOcclusionFilterFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCFilterFuncVarying filter)
{
  if (sizeof(varying float) == 16)
    ispcSetOcclusionFilterFunction4(scene,geomID,filter);
  else if (sizeof(varying float) == 32)
    ispcSetOcclusionFilterFunction8(scene,geomID,filter);
  else if (sizeof(varying float) == 64)
    ispcSetOcclusionFilterFunction16(scene,geomID,filter);
}

void rtcSetDisplacementFunction (RTCScene scene, uniform unsigned int geomID, uniform RTCDisplacementFunc func, uniform RTCBounds* uniform bounds)
{
  ispcSetDisplacementFunction(scene,geomID,func,bounds);
}

void rtcInterpolate(RTCScene scene, uniform unsigned int geomID, varying unsigned int primID, varying float u, varying float v, 
                    uniform RTCBufferType buffer,
                    varying float* uniform P, varying float* uniform dPdu, varying float* uniform dPdv, uniform size_t numFloats)
{
  varying bool mask = __mask;
  unmasked {
    varying int imask = mask ? -1 : 0;
  }
  ispcInterpolateN(scene,geomID,(const void* uniform)&imask,(const uniform unsigned int* uniform)&primID,
                   (const uniform float* uniform)&u,(const uniform float* uniform)&v,sizeof(varying float)/4,buffer,
                   (uniform float* uniform)P,(uniform float* uniform)dPdu,(uniform float* uniform)dPdv,numFloats);
}
