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

#ifdef _WIN32
#  define RTCORE_API extern "C" __declspec(dllexport)
#else
#  define RTCORE_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "device.h"
#include "scene.h"
#include "raystream_log.h"

namespace embree
{  
  RTCORE_API RTCDevice rtcNewDevice(const char* cfg)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewDevice);
    return (RTCDevice) new Device(cfg,false);
    RTCORE_CATCH_END_NOREPORT;
    return (RTCDevice) nullptr;
  }

  RTCORE_API void rtcDeleteDevice(RTCDevice device) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteDevice);
    RTCORE_VERIFY_HANDLE(device);
    delete (Device*) device;
    RTCORE_CATCH_END_NOREPORT;
  }

  /* mutex to make API thread safe */
  static MutexSys g_mutex;

  /* global device for compatibility with old rtcInit / rtcExit scheme */
  Device* g_device = nullptr;

  RTCORE_API void rtcInit(const char* cfg) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInit);
    Lock<MutexSys> lock(g_mutex);
    if (g_device) throw_RTCError(RTC_INVALID_OPERATION,"already initialized");
    g_device = new Device(cfg,true);
    RTCORE_CATCH_END(g_device);
  }
  
  RTCORE_API void rtcExit() 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcExit);
    Lock<MutexSys> lock(g_mutex);
    if (!g_device) throw_RTCError(RTC_INVALID_OPERATION,"rtcInit has to get called before rtcExit");
    delete g_device; g_device = nullptr;
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcSetParameter1i(const RTCParameter parm, ssize_t val)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetParameter1i);
    if (g_device) g_device->setParameter1i(parm,val);
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcDeviceSetParameter1i(RTCDevice hdevice, const RTCParameter parm, ssize_t val)
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetParameter1i);
    RTCORE_VERIFY_HANDLE(hdevice);
    device->setParameter1i(parm,val);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API RTCError rtcGetError()
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetError);
    if (g_device == nullptr) return RTC_UNKNOWN_ERROR;
    RTCError* stored_error = g_device->error();
    RTCError error = *stored_error;
    *stored_error = RTC_NO_ERROR;
    return error;
    RTCORE_CATCH_END(g_device);
    return RTC_UNKNOWN_ERROR;
  }

  RTCORE_API RTCError rtcDeviceGetError(RTCDevice hdevice)
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceGetError);
    RTCORE_VERIFY_HANDLE(hdevice);
    RTCError* stored_error = device->error();
    RTCError error = *stored_error;
    *stored_error = RTC_NO_ERROR;
    return error;
    RTCORE_CATCH_END(device);
    return RTC_UNKNOWN_ERROR;
  }

  RTCORE_API void rtcSetErrorFunction(RTCErrorFunc func) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetErrorFunction);
    assert(g_device);
    if (g_device) g_device->error_function = func;
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcDeviceSetErrorFunction(RTCDevice hdevice, RTCErrorFunc func) 
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetErrorFunction);
    RTCORE_VERIFY_HANDLE(hdevice);
    device->error_function = func;
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcSetMemoryMonitorFunction(RTCMemoryMonitorFunc func) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetMemoryMonitorFunction);
    assert(g_device);
    if (g_device) g_device->memory_monitor_function = func;
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcDeviceSetMemoryMonitorFunction(RTCDevice hdevice, RTCMemoryMonitorFunc func) 
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetMemoryMonitorFunction);
    device->memory_monitor_function = func;
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcDebug() 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDebug);

#if defined(RTCORE_STAT_COUNTERS)
    Stat::print(std::cout);
    Stat::clear();
#endif

#if defined(DEBUG) && 1
    extern void printTessCacheStats();
    printTessCacheStats();
#endif
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API RTCScene rtcNewScene (RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewScene);
    assert(g_device);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_INCOHERENT);
    return (RTCScene) new Scene(g_device,flags,aflags);
    RTCORE_CATCH_END(g_device);
    return nullptr;
  }

  RTCORE_API RTCScene rtcDeviceNewScene (RTCDevice device, RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceNewScene);
    RTCORE_VERIFY_HANDLE(device);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_INCOHERENT);
    return (RTCScene) new Scene((Device*)device,flags,aflags);
    RTCORE_CATCH_END((Device*)device);
    return nullptr;
  }

  RTCORE_API void rtcSetProgressMonitorFunction(RTCScene hscene, RTCProgressMonitorFunc func, void* ptr) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetProgressMonitorFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->setProgressMonitorFunction(func,ptr);
    RTCORE_CATCH_END(scene->device);
  }
  
  RTCORE_API void rtcCommit (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommit);
    RTCORE_VERIFY_HANDLE(hscene);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.dumpGeometry(scene);
#endif

    /* perform scene build */
    scene->build(0,0);
    
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcCommitThread(RTCScene hscene, unsigned int threadID, unsigned int numThreads) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitThread);
    RTCORE_VERIFY_HANDLE(hscene);

    if (unlikely(numThreads == 0)) 
      throw_RTCError(RTC_INVALID_OPERATION,"invalid number of threads specified");

#if defined(__MIC__)
    if (unlikely(numThreads % 4 != 0 && numThreads != 1)) 
      throw_RTCError(RTC_INVALID_OPERATION,"MIC requires numThreads % 4 == 0 in rtcCommitThread");
#endif
    
    /* for best performance set FTZ and DAZ flags in the MXCSR control and status register */
#if !defined(__MIC__)
    unsigned int mxcsr = _mm_getcsr();
    _mm_setcsr(mxcsr | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));
#endif
    
     /* perform scene build */
    scene->build(threadID,numThreads);

 /* reset MXCSR register again */
#if !defined(__MIC__)
    _mm_setcsr(mxcsr);
#endif

    RTCORE_CATCH_END(scene->device);
  }
  
  RTCORE_API void rtcIntersect (RTCScene hscene, RTCRay& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay old_ray = ray;
#endif

    STAT3(normal.travs,1,1,1);
    scene->intersect(ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay1Intersect(scene,old_ray,ray);
#endif
    RTCORE_CATCH_END(scene->device);
  }

#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcIntersect4 (const void* valid, RTCScene hscene, RTCRay4& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect4);
#if !defined(__TARGET_SIMD4__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 not supported");    
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,4);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay4 old_ray = ray;
#endif

    scene->intersect4(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay4Intersect(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END(scene->device);
  }
#endif
  
#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcIntersect8 (const void* valid, RTCScene hscene, RTCRay8& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect8);
#if !defined(__TARGET_SIMD8__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 not supported");                                    
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,8);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay8 old_ray = ray;
#endif

    scene->intersect8(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay8Intersect(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END(scene->device);
  }
#endif
  
#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcIntersect16 (const void* valid, RTCScene hscene, RTCRay16& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect16);
#if !defined(__TARGET_SIMD16__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,16);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay16 old_ray = ray;
#endif

    scene->intersect16(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay16Intersect(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END(scene->device);
  }
#endif
  
  RTCORE_API void rtcOccluded (RTCScene hscene, RTCRay& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded);
    STAT3(shadow.travs,1,1,1);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay old_ray = ray;
#endif

    scene->occluded(ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay1Occluded(scene,old_ray,ray);
#endif
    RTCORE_CATCH_END(scene->device);
  }
  
#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcOccluded4 (const void* valid, RTCScene hscene, RTCRay4& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded4);
#if !defined(__TARGET_SIMD4__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded4 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,4);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay4 old_ray = ray;
#endif

    scene->occluded4(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay4Occluded(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END(scene->device);
  }
#endif
  
#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcOccluded8 (const void* valid, RTCScene hscene, RTCRay8& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded8);
#if !defined(__TARGET_SIMD8__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded8 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,8);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay8 old_ray = ray;
#endif

    scene->occluded8(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay8Occluded(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END(scene->device);
  }
#endif
  
#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcOccluded16 (const void* valid, RTCScene hscene, RTCRay16& ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded16);
#if !defined(__TARGET_SIMD16__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded16 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,16);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay16 old_ray = ray;
#endif

    scene->occluded16(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay16Occluded(valid,scene,old_ray,ray);
#endif
    
#endif
    RTCORE_CATCH_END(scene->device);
  }
#endif
  
  RTCORE_API void rtcDeleteScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    Device* device = scene ? scene->device : nullptr;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteScene);
    RTCORE_VERIFY_HANDLE(hscene);
    delete scene;
    RTCORE_CATCH_END(device);
  }

  RTCORE_API unsigned rtcNewInstance (RTCScene htarget, RTCScene hsource) 
  {
    Scene* target = (Scene*) htarget;
    Scene* source = (Scene*) hsource;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewInstance);
    RTCORE_VERIFY_HANDLE(htarget);
    RTCORE_VERIFY_HANDLE(hsource);
    if (target->device != source->device) throw_RTCError(RTC_INVALID_OPERATION,"scenes do not belong to the same device");
    return target->newInstance(source);
    RTCORE_CATCH_END(target->device);
    return -1;
  }

  RTCORE_API void rtcSetTransform (RTCScene hscene, unsigned geomID, RTCMatrixType layout, const float* xfm) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetTransform);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    RTCORE_VERIFY_HANDLE(xfm);

    AffineSpace3fa transform = one;
    switch (layout) 
    {
    case RTC_MATRIX_ROW_MAJOR:
      transform = AffineSpace3fa(Vec3fa(xfm[ 0],xfm[ 4],xfm[ 8]),
                                 Vec3fa(xfm[ 1],xfm[ 5],xfm[ 9]),
                                 Vec3fa(xfm[ 2],xfm[ 6],xfm[10]),
                                 Vec3fa(xfm[ 3],xfm[ 7],xfm[11]));
      break;

    case RTC_MATRIX_COLUMN_MAJOR:
      transform = AffineSpace3fa(Vec3fa(xfm[ 0],xfm[ 1],xfm[ 2]),
                                 Vec3fa(xfm[ 3],xfm[ 4],xfm[ 5]),
                                 Vec3fa(xfm[ 6],xfm[ 7],xfm[ 8]),
                                 Vec3fa(xfm[ 9],xfm[10],xfm[11]));
      break;

    case RTC_MATRIX_COLUMN_MAJOR_ALIGNED16:
      transform = AffineSpace3fa(Vec3fa(xfm[ 0],xfm[ 1],xfm[ 2]),
                                 Vec3fa(xfm[ 4],xfm[ 5],xfm[ 6]),
                                 Vec3fa(xfm[ 8],xfm[ 9],xfm[10]),
                                 Vec3fa(xfm[12],xfm[13],xfm[14]));
      break;

    default: 
      throw_RTCError(RTC_INVALID_OPERATION,"Unknown matrix type");
      break;
    }
    ((Scene*) scene)->get_locked(geomID)->setTransform(transform);

    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API unsigned rtcNewUserGeometry (RTCScene hscene, size_t numItems) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewUserGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    return scene->newUserGeometry(numItems);
    RTCORE_CATCH_END(scene->device);
    return -1;
  }

  RTCORE_API unsigned rtcNewTriangleMesh (RTCScene hscene, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewTriangleMesh);
    RTCORE_VERIFY_HANDLE(hscene);
    return scene->newTriangleMesh(flags,numTriangles,numVertices,numTimeSteps);
    RTCORE_CATCH_END(scene->device);
    return -1;
  }

  RTCORE_API unsigned rtcNewHairGeometry (RTCScene hscene, RTCGeometryFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewHairGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    return scene->newBezierCurves(flags,numCurves,numVertices,numTimeSteps);
    RTCORE_CATCH_END(scene->device);
    return -1;
  }

  RTCORE_API unsigned rtcNewSubdivisionMesh (RTCScene hscene, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, 
                                             size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewSubdivisionMesh);
    RTCORE_VERIFY_HANDLE(hscene);
    return scene->newSubdivisionMesh(flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
    RTCORE_CATCH_END(scene->device);
    return -1;
  }

  RTCORE_API void rtcSetMask (RTCScene hscene, unsigned geomID, int mask) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetMask);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setMask(mask);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetBoundaryMode (RTCScene hscene, unsigned geomID, RTCBoundaryMode mode) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundaryMode);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setBoundaryMode(mode);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void* rtcMapBuffer(RTCScene hscene, unsigned geomID, RTCBufferType type) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcMapBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    return scene->get_locked(geomID)->map(type);
    RTCORE_CATCH_END(scene->device);
    return nullptr;
  }

  RTCORE_API void rtcUnmapBuffer(RTCScene hscene, unsigned geomID, RTCBufferType type) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUnmapBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->unmap(type);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetBuffer(RTCScene hscene, unsigned geomID, RTCBufferType type, void* ptr, size_t offset, size_t stride)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setBuffer(type,ptr,offset,stride);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcEnable (RTCScene hscene, unsigned geomID) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcEnable);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->enable();
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcUpdate (RTCScene hscene, unsigned geomID) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdate);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->update();
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcUpdateBuffer (RTCScene hscene, unsigned geomID, RTCBufferType type) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdateBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->updateBuffer(type);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcDisable (RTCScene hscene, unsigned geomID) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDisable);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->disable();
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcDeleteGeometry (RTCScene hscene, unsigned geomID) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->erase();
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetUserData (RTCScene hscene, unsigned geomID, void* ptr) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetUserData);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setUserData(ptr);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void* rtcGetUserData (RTCScene hscene, unsigned geomID)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetUserData);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    return scene->get(geomID)->getUserData(); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->device);
    return nullptr;
  }

  RTCORE_API void rtcSetBoundsFunction (RTCScene hscene, unsigned geomID, RTCBoundsFunc bounds)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundsFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setBoundsFunction(bounds);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetDisplacementFunction (RTCScene hscene, unsigned geomID, RTCDisplacementFunc func, RTCBounds* bounds)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDisplacementFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setDisplacementFunction(func,bounds);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetIntersectFunction (RTCScene hscene, unsigned geomID, RTCIntersectFunc intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction(intersect);
    RTCORE_CATCH_END(scene->device);
  }

#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcSetIntersectFunction4 (RTCScene hscene, unsigned geomID, RTCIntersectFunc4 intersect4) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction4(intersect4);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetIntersectFunction8 (RTCScene hscene, unsigned geomID, RTCIntersectFunc8 intersect8) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction8(intersect8);
    RTCORE_CATCH_END(scene->device);
  }
  
  RTCORE_API void rtcSetIntersectFunction16 (RTCScene hscene, unsigned geomID, RTCIntersectFunc16 intersect16) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectFunction16(intersect16);
    RTCORE_CATCH_END(scene->device);
  }
#endif

  RTCORE_API void rtcSetOccludedFunction (RTCScene hscene, unsigned geomID, RTCOccludedFunc occluded) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOccludedFunction(occluded);
    RTCORE_CATCH_END(scene->device);
  }

#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcSetOccludedFunction4 (RTCScene hscene, unsigned geomID, RTCOccludedFunc4 occluded4) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOccludedFunction4(occluded4);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetOccludedFunction8 (RTCScene hscene, unsigned geomID, RTCOccludedFunc8 occluded8) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOccludedFunction8(occluded8);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetOccludedFunction16 (RTCScene hscene, unsigned geomID, RTCOccludedFunc16 occluded16) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOccludedFunction16(occluded16);
    RTCORE_CATCH_END(scene->device);
  }
#endif

  RTCORE_API void rtcSetIntersectionFilterFunction (RTCScene hscene, unsigned geomID, RTCFilterFunc intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectionFilterFunction(intersect);
    RTCORE_CATCH_END(scene->device);
  }

#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcSetIntersectionFilterFunction4 (RTCScene hscene, unsigned geomID, RTCFilterFunc4 filter4) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectionFilterFunction4(filter4);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetIntersectionFilterFunction8 (RTCScene hscene, unsigned geomID, RTCFilterFunc8 filter8) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectionFilterFunction8(filter8);
    RTCORE_CATCH_END(scene->device);
  }
  
  RTCORE_API void rtcSetIntersectionFilterFunction16 (RTCScene hscene, unsigned geomID, RTCFilterFunc16 filter16) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setIntersectionFilterFunction16(filter16);
    RTCORE_CATCH_END(scene->device);
  }
#endif

  RTCORE_API void rtcSetOcclusionFilterFunction (RTCScene hscene, unsigned geomID, RTCFilterFunc intersect) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOcclusionFilterFunction(intersect);
    RTCORE_CATCH_END(scene->device);
  }

#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcSetOcclusionFilterFunction4 (RTCScene hscene, unsigned geomID, RTCFilterFunc4 filter4) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOcclusionFilterFunction4(filter4);
    RTCORE_CATCH_END(scene->device);
  }

  RTCORE_API void rtcSetOcclusionFilterFunction8 (RTCScene hscene, unsigned geomID, RTCFilterFunc8 filter8) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOcclusionFilterFunction8(filter8);
    RTCORE_CATCH_END(scene->device);
  }
  
  RTCORE_API void rtcSetOcclusionFilterFunction16 (RTCScene hscene, unsigned geomID, RTCFilterFunc16 filter16) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get_locked(geomID)->setOcclusionFilterFunction16(filter16);
    RTCORE_CATCH_END(scene->device);
  }
#endif

  RTCORE_API void rtcInterpolate(RTCScene hscene, unsigned geomID, unsigned primID, float u, float v, 
                                 RTCBufferType buffer,
                                 float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolate);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get(geomID)->interpolate(primID,u,v,buffer,P,dPdu,dPdv,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->device);
  }

#if defined (RTCORE_RAY_PACKETS)
  RTCORE_API void rtcInterpolateN(RTCScene hscene, unsigned geomID, 
                                  const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                                  RTCBufferType buffer,
                                  float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolateN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->get(geomID)->interpolateN(valid_i,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->device);
  }
#endif
}
