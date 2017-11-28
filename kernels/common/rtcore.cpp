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

#ifdef _WIN32
#  define RTCORE_API extern "C" __declspec(dllexport)
#else
#  define RTCORE_API extern "C" __attribute__ ((visibility ("default")))
#endif

#include "default.h"
#include "device.h"
#include "scene.h"
#include "context.h"
#include "../../include/embree3/rtcore_ray.h"

namespace embree
{  
  /* mutex to make API thread safe */
  static MutexSys g_mutex;

  RTCORE_API RTCDevice rtcNewDevice(const char* cfg)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewDevice);
    Lock<MutexSys> lock(g_mutex);
    Device* device = new Device(cfg,false);
    return (RTCDevice) device->refInc();
    RTCORE_CATCH_END(nullptr);
    return (RTCDevice) nullptr;
  }

  RTCORE_API void rtcReleaseDevice(RTCDevice hdevice) 
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcReleaseDevice);
    RTCORE_VERIFY_HANDLE(hdevice);
    Lock<MutexSys> lock(g_mutex);
    device->refDec();
    RTCORE_CATCH_END(nullptr);
  }
  
  RTCORE_API void rtcSetDeviceProperty(RTCDevice hdevice, const RTCDeviceProperty prop, ssize_t val)
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDeviceProperty);
    const bool internal_prop = (size_t)prop >= 1000000 && (size_t)prop < 1000004;
    if (!internal_prop) RTCORE_VERIFY_HANDLE(hdevice); // allow NULL device for special internal settings
    Lock<MutexSys> lock(g_mutex);
    device->setProperty(prop,val);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API ssize_t rtcGetDeviceProperty(RTCDevice hdevice, const RTCDeviceProperty prop)
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetDeviceProperty);
    RTCORE_VERIFY_HANDLE(hdevice);
    Lock<MutexSys> lock(g_mutex);
    return device->getProperty(prop);
    RTCORE_CATCH_END(device);
    return 0;
  }

  RTCORE_API RTCError rtcGetDeviceError(RTCDevice hdevice)
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetDeviceError);
    if (device == nullptr) return Device::getThreadErrorCode();
    else                   return device->getDeviceErrorCode();
    RTCORE_CATCH_END(device);
    return RTC_ERROR_UNKNOWN;
  }

  RTCORE_API void rtcSetDeviceErrorFunction(RTCDevice hdevice, RTCErrorFunction func, void* userPtr) 
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDeviceErrorFunction);
    RTCORE_VERIFY_HANDLE(hdevice);
    device->setErrorFunction(func,userPtr);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcSetDeviceMemoryMonitorFunction(RTCDevice hdevice, RTCMemoryMonitorFunction func, void* userPtr) 
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDeviceMemoryMonitorFunction);
    device->setMemoryMonitorFunction(func,userPtr);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API RTCScene rtcNewScene (RTCDevice device) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewScene);
    RTCORE_VERIFY_HANDLE(device);
    Scene* scene = new Scene((Device*)device);
    return (RTCScene) scene->refInc();
    RTCORE_CATCH_END((Device*)device);
    return nullptr;
  }

  RTCORE_API void rtcSetSceneProgressMonitorFunction(RTCScene hscene, RTCProgressMonitorFunction func, void* ptr) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetSceneProgressMonitorFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->setProgressMonitorFunction(func,ptr);
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcSetSceneBuildQuality (RTCScene hscene, RTCBuildQuality quality) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetSceneBuildQuality);
    RTCORE_VERIFY_HANDLE(hscene);
    if (quality != RTC_BUILD_QUALITY_LOW &&
        quality != RTC_BUILD_QUALITY_MEDIUM &&
        quality != RTC_BUILD_QUALITY_HIGH)
      throw std::runtime_error("invalid build quality");
    scene->setBuildQuality(quality);
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API RTCBuildQuality rtcGetSceneBuildQuality(RTCScene hscene)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetSceneBuildQuality);
    RTCORE_VERIFY_HANDLE(hscene);
    return scene->getBuildQuality();
    RTCORE_CATCH_END2(scene);
    return RTC_BUILD_QUALITY_MEDIUM;
  }

  RTCORE_API void rtcSetSceneFlags (RTCScene hscene, RTCSceneFlags sflags) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetSceneFlags);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->setSceneFlags(sflags);
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API RTCSceneFlags rtcGetSceneFlags(RTCScene hscene)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetSceneFlags);
    RTCORE_VERIFY_HANDLE(hscene);
    return scene->getSceneFlags();
    RTCORE_CATCH_END2(scene);
    return RTC_SCENE_FLAG_NONE;
  }
  
  RTCORE_API void rtcCommitScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitScene);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->commit(false);
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcCommitJoinScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitJoinScene);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->commit(true);
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcGetSceneBounds(RTCScene hscene, RTCBounds* bounds_o)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetSceneBounds);
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    BBox3fa bounds = scene->bounds.bounds();
    bounds_o->lower_x = bounds.lower.x;
    bounds_o->lower_y = bounds.lower.y;
    bounds_o->lower_z = bounds.lower.z;
    bounds_o->align0  = 0;
    bounds_o->upper_x = bounds.upper.x;
    bounds_o->upper_y = bounds.upper.y;
    bounds_o->upper_z = bounds.upper.z;
    bounds_o->align1  = 0;
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcGetSceneLinearBounds(RTCScene hscene, RTCBounds* bounds_o)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetSceneBounds);
    RTCORE_VERIFY_HANDLE(hscene);
    if (bounds_o == nullptr)
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid destination pointer");
    if (scene->isModified())
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    
    bounds_o[0].lower_x = scene->bounds.bounds0.lower.x;
    bounds_o[0].lower_y = scene->bounds.bounds0.lower.y;
    bounds_o[0].lower_z = scene->bounds.bounds0.lower.z;
    bounds_o[0].align0  = 0;
    bounds_o[0].upper_x = scene->bounds.bounds0.upper.x;
    bounds_o[0].upper_y = scene->bounds.bounds0.upper.y;
    bounds_o[0].upper_z = scene->bounds.bounds0.upper.z;
    bounds_o[0].align1  = 0;
    bounds_o[1].lower_x = scene->bounds.bounds1.lower.x;
    bounds_o[1].lower_y = scene->bounds.bounds1.lower.y;
    bounds_o[1].lower_z = scene->bounds.bounds1.lower.z;
    bounds_o[1].align0  = 0;
    bounds_o[1].upper_x = scene->bounds.bounds1.upper.x;
    bounds_o[1].upper_y = scene->bounds.bounds1.upper.y;
    bounds_o[1].upper_z = scene->bounds.bounds1.upper.z;
    bounds_o[1].align1  = 0;
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcIntersect1 (RTCScene hscene, RTCIntersectContext* user_context, RTCRay* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect1);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT3(normal.travs,1,1,1);
    IntersectContext context(scene,user_context);
    scene->intersectors.intersect(*ray,&context);
#if defined(DEBUG)
    ((Ray*)ray)->verifyHit();
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcIntersect4 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay4* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect4);

#if defined(EMBREE_TARGET_SIMD4) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)ray)   & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,cnt,cnt,cnt);
    IntersectContext context(scene,user_context);
    scene->intersectors.intersect4(valid,*ray,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect4 not supported");  
#endif
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcIntersect8 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay8* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect8);

#if defined(EMBREE_TARGET_SIMD8) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)ray)   & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
    scene->intersectors.intersect8(valid,*ray,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect8 not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcIntersect16 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay16* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect16);

#if defined(EMBREE_TARGET_SIMD16) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)ray)   & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
    scene->intersectors.intersect16(valid,*ray,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect16 not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcIntersect1M (RTCScene hscene, RTCIntersectContext* user_context, RTCRay* rays, const unsigned int M, const size_t stride) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect1M);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays ) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for single rays */
    if (likely(M == 1)) {
      if (likely(rays->tnear <= rays->tfar)) 
        scene->intersectors.intersect(*rays,&context);
    } 

    /* codepath for streams */
    else {
      scene->device->rayStreamFilters.filterAOS(scene,rays,M,stride,&context,true);   
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect1M not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcIntersect1Mp (RTCScene hscene, RTCIntersectContext* user_context, RTCRay** rays, const unsigned int M) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect1Mp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for single rays */
    if (likely(M == 1)) {
      if (likely(rays[0]->tnear <= rays[0]->tfar)) 
        scene->intersectors.intersect(*rays[0],&context);
    } 

    /* codepath for streams */
    else {
      scene->device->rayStreamFilters.filterAOP(scene,rays,M,&context,true);   
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersect1Mp not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcIntersectNM (RTCScene hscene, RTCIntersectContext* user_context, struct RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersectNM);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,N*M,N*M,N*M);
    IntersectContext context(scene,user_context);

    /* code path for single ray streams */
    if (likely(N == 1))
    {
      /* fast code path for streams of size 1 */
      if (likely(M == 1)) {
        if (likely(((RTCRay*)rays)->tnear <= ((RTCRay*)rays)->tfar))
          scene->intersectors.intersect(*(RTCRay*)rays,&context);
      } 
      /* normal codepath for single ray streams */
      else {
        scene->device->rayStreamFilters.filterAOS(scene,(RTCRay*)rays,M,stride,&context,true);
      }
    }
    /* code path for ray packet streams */
    else {
      scene->device->rayStreamFilters.filterSOA(scene,(char*)rays,N,M,stride,&context,true);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersectNM not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcIntersectNp (RTCScene hscene, RTCIntersectContext* user_context, const RTCRayNp* rays, const unsigned int N) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersectNp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays->org_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->org_x not aligned to 4 bytes");   
    if (((size_t)rays->org_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->org_y not aligned to 4 bytes");   
    if (((size_t)rays->org_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->org_z not aligned to 4 bytes");   
    if (((size_t)rays->dir_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_x not aligned to 4 bytes");   
    if (((size_t)rays->dir_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_y not aligned to 4 bytes");   
    if (((size_t)rays->dir_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_z not aligned to 4 bytes");   
    if (((size_t)rays->tnear ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_x not aligned to 4 bytes");   
    if (((size_t)rays->tfar  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->tnear not aligned to 4 bytes");   
    if (((size_t)rays->time  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->time not aligned to 4 bytes");   
    if (((size_t)rays->mask  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->mask not aligned to 4 bytes");   
    if (((size_t)rays->Ng_x  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->Ng_x not aligned to 4 bytes");   
    if (((size_t)rays->Ng_y  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->Ng_y not aligned to 4 bytes");   
    if (((size_t)rays->Ng_z  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->Ng_z not aligned to 4 bytes");   
    if (((size_t)rays->u     ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->u not aligned to 4 bytes");   
    if (((size_t)rays->v     ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->v not aligned to 4 bytes");   
    if (((size_t)rays->geomID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->geomID not aligned to 4 bytes");   
    if (((size_t)rays->primID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->primID not aligned to 4 bytes");   
    if (((size_t)rays->instID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->instID not aligned to 4 bytes");   
#endif
    STAT3(normal.travs,N,N,N);
    IntersectContext context(scene,user_context);
    scene->device->rayStreamFilters.filterSOP(scene,*rays,N,&context,true);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcIntersectNp not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcOccluded1 (RTCScene hscene, RTCIntersectContext* user_context, RTCRay* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded1);
    STAT3(shadow.travs,1,1,1);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)ray) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    IntersectContext context(scene,user_context);
    scene->intersectors.occluded(*ray,&context);
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcOccluded4 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay4* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded4);

#if defined(EMBREE_TARGET_SIMD4) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)ray)   & 0x0F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,cnt,cnt,cnt);
    IntersectContext context(scene,user_context);
    scene->intersectors.occluded4(valid,*ray,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded4 not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }
 
  RTCORE_API void rtcOccluded8 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay8* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded8);

#if defined(EMBREE_TARGET_SIMD8) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)ray)   & 0x1F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
    scene->intersectors.occluded8(valid,*ray,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded8 not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcOccluded16 (const int* valid, RTCScene hscene, RTCIntersectContext* user_context, RTCRay16* ray) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded16);

#if defined(EMBREE_TARGET_SIMD16) && defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)ray)   & 0x3F) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,cnt,cnt,cnt);

    IntersectContext context(scene,user_context);
    scene->intersectors.occluded16(valid,*ray,&context);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded16 not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcOccluded1M(RTCScene hscene, RTCIntersectContext* user_context, RTCRay* rays, const unsigned int M, const size_t stride) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded1M);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for streams of size 1 */
    if (likely(M == 1)) {
      if (likely(rays->tnear <= rays->tfar)) 
        scene->intersectors.occluded (*rays,&context);
    } 
    /* codepath for normal streams */
    else {
      scene->device->rayStreamFilters.filterAOS(scene,rays,M,stride,&context,false);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded1M not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcOccluded1Mp(RTCScene hscene, RTCIntersectContext* user_context, RTCRay** rays, const unsigned int M) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded1Mp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,M,M,M);
    IntersectContext context(scene,user_context);

    /* fast codepath for streams of size 1 */
    if (likely(M == 1)) {
      if (likely(rays[0]->tnear <= rays[0]->tfar)) 
        scene->intersectors.occluded (*rays[0],&context);
    } 
    /* codepath for normal streams */
    else {
      scene->device->rayStreamFilters.filterAOP(scene,rays,M,&context,false);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccluded1Mp not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcOccludedNM(RTCScene hscene, RTCIntersectContext* user_context, RTCRayN* rays, const unsigned int N, const unsigned int M, const size_t stride) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccludedNM);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (stride < sizeof(RTCRay)) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"stride too small");
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays) & 0x03) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "ray not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,N*M,N*N,N*N);
    IntersectContext context(scene,user_context);

    /* codepath for single rays */
    if (likely(N == 1))
    {
      /* fast path for streams of size 1 */
      if (likely(M == 1)) {
        if (likely(((RTCRay*)rays)->tnear <= ((RTCRay*)rays)->tfar))
          scene->intersectors.occluded (*(RTCRay*)rays,&context);
      } 
      /* codepath for normal ray streams */
      else {
        scene->device->rayStreamFilters.filterAOS(scene,(RTCRay*)rays,M,stride,&context,false);
      }
    }
    /* code path for ray packet streams */
    else {
      scene->device->rayStreamFilters.filterSOA(scene,(char*)rays,N,M,stride,&context,false);
    }
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccludedNM not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcOccludedNp(RTCScene hscene, RTCIntersectContext* user_context, const RTCRayNp* rays, const unsigned int N) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccludedNp);

#if defined (EMBREE_RAY_PACKETS)
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    if (scene->isModified()) throw_RTCError(RTC_ERROR_INVALID_OPERATION,"scene got not committed");
    if (((size_t)rays->org_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->org_x not aligned to 4 bytes");   
    if (((size_t)rays->org_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->org_y not aligned to 4 bytes");   
    if (((size_t)rays->org_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->org_z not aligned to 4 bytes");   
    if (((size_t)rays->dir_x ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_x not aligned to 4 bytes");   
    if (((size_t)rays->dir_y ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_y not aligned to 4 bytes");   
    if (((size_t)rays->dir_z ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_z not aligned to 4 bytes");   
    if (((size_t)rays->tnear ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->dir_x not aligned to 4 bytes");   
    if (((size_t)rays->tfar  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->tnear not aligned to 4 bytes");   
    if (((size_t)rays->time  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->time not aligned to 4 bytes");   
    if (((size_t)rays->mask  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->mask not aligned to 4 bytes");   
    if (((size_t)rays->Ng_x  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->Ng_x not aligned to 4 bytes");   
    if (((size_t)rays->Ng_y  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->Ng_y not aligned to 4 bytes");   
    if (((size_t)rays->Ng_z  ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->Ng_z not aligned to 4 bytes");   
    if (((size_t)rays->u     ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->u not aligned to 4 bytes");   
    if (((size_t)rays->v     ) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->v not aligned to 4 bytes");   
    if (((size_t)rays->geomID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->geomID not aligned to 4 bytes");   
    if (((size_t)rays->primID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->primID not aligned to 4 bytes");   
    if (((size_t)rays->instID) & 0x03 ) throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "rays->instID not aligned to 4 bytes");   
#endif
    STAT3(shadow.travs,N,N,N);
    IntersectContext context(scene,user_context);
    scene->device->rayStreamFilters.filterSOP(scene,*rays,N,&context,false);
#else
    throw_RTCError(RTC_ERROR_INVALID_OPERATION,"rtcOccludedNp not supported");
#endif
    RTCORE_CATCH_END2(scene);
  }
  
  RTCORE_API void rtcReleaseScene (RTCScene hscene) 
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcReleaseScene);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->refDec();
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API RTCGeometry rtcNewInstance (RTCDevice hdevice, RTCScene hsource, unsigned int numTimeSteps)
  {
    Device* device = (Device*) hdevice;
    Scene* source = (Scene*) hsource;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewInstance);
    RTCORE_VERIFY_HANDLE(hdevice);
    RTCORE_VERIFY_HANDLE(hsource);
#if defined(EMBREE_GEOMETRY_USER)
    Geometry* geom = Instance::create(device,source,numTimeSteps);
    return (RTCGeometry) geom->refInc();
#else
    throw_RTCError(RTC_ERROR_UNKNOWN,"rtcNewInstance is not supported");
#endif
    RTCORE_CATCH_END(device);
    return nullptr;
  }

  RTCORE_API RTCGeometry rtcNewGeometryInstance (RTCDevice hdevice,  RTCScene hscene, unsigned geomID) 
  {
    Device* device = (Device*) hdevice;
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewGeometryInstance);
    RTCORE_VERIFY_HANDLE(hdevice);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    Geometry* geom = new GeometryInstance(device,scene->get_locked(geomID));
    return (RTCGeometry) geom->refInc();
    RTCORE_CATCH_END(device);
    return nullptr;
  }

  RTCORE_API RTCGeometry rtcNewGeometryGroup (RTCDevice hdevice, RTCScene hscene, unsigned* geomIDs, unsigned int N) 
  {
    Device* device = (Device*) hdevice;
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewGeometryGroup);
    RTCORE_VERIFY_HANDLE(hdevice);
    RTCORE_VERIFY_HANDLE(hscene);
    if (N) RTCORE_VERIFY_HANDLE(geomIDs);

    std::vector<Ref<Geometry>> geometries(N);
    for (size_t i=0; i<N; i++) {
      RTCORE_VERIFY_GEOMID(geomIDs[i]);
      geometries[i] = scene->get_locked(geomIDs[i]);
      if (geometries[i]->getType() == Geometry::GROUP)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"geometry groups cannot contain other geometry groups");
      if (geometries[i]->getType() != geometries[0]->getType())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"geometries inside group have to be of same type");
    }
    Geometry* geom = new GeometryGroup(device,geometries);
    return (RTCGeometry) geom->refInc();
    RTCORE_CATCH_END(device);
    return nullptr;
  }

  AffineSpace3fa convertTransform(RTCMatrixType layout, const float* xfm)
  {
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
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"Unknown matrix type");
      break;
    }
    return transform;
  }

  RTCORE_API void rtcSetGeometryTransform (RTCGeometry hgeometry, RTCMatrixType layout, const float* xfm, unsigned int timeStep) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryTransform);
    RTCORE_VERIFY_HANDLE(hgeometry);
    RTCORE_VERIFY_HANDLE(xfm);
    const AffineSpace3fa transform = convertTransform(layout,xfm);
    geometry->setTransform(transform,timeStep);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcReportIntersection(const struct RTCIntersectFunctionNArguments* const args_i, const struct RTCFilterFunctionNArguments* filter_args)
  {
    IntersectFunctionNArguments* args = (IntersectFunctionNArguments*) args_i;
    args->report(args,filter_args);
  }

  RTCORE_API void rtcReportOcclusion(const struct RTCOccludedFunctionNArguments* const args_i, const struct RTCFilterFunctionNArguments* filter_args)
  {
    OccludedFunctionNArguments* args = (OccludedFunctionNArguments*) args_i;
    args->report(args,filter_args);
  }
  
  RTCORE_API RTCGeometry rtcNewGeometry (RTCDevice hdevice, RTCGeometryType type)
  {
    Device* device = (Device*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewGeometry);
    RTCORE_VERIFY_HANDLE(hdevice);
    
    switch (type)
    {
    case RTC_GEOMETRY_TYPE_TRIANGLE:
      {
  #if defined(EMBREE_GEOMETRY_TRIANGLES)
        createTriangleMeshTy createTriangleMesh = nullptr;
        SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createTriangleMesh);
        Geometry* geom = createTriangleMesh(device);
        return (RTCGeometry) geom->refInc();
  #else
        throw_RTCError(RTC_ERROR_UNKNOWN,"rtcNewTriangleMesh is not supported");
  #endif
      }
      break;

    case RTC_GEOMETRY_TYPE_QUAD:
      {
#if defined(EMBREE_GEOMETRY_QUADS)
        createQuadMeshTy createQuadMesh = nullptr;
        SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createQuadMesh);
        Geometry* geom = createQuadMesh(device);
        return (RTCGeometry) geom->refInc();
#else
        throw_RTCError(RTC_ERROR_UNKNOWN,"rtcNewQuadMesh is not supported");
#endif
      }
      break;

    case RTC_GEOMETRY_TYPE_CURVE_LINEAR:
    case RTC_GEOMETRY_TYPE_CURVE_BEZIER:
    case RTC_GEOMETRY_TYPE_CURVE_BSPLINE:
      {
#if defined(EMBREE_GEOMETRY_HAIR)
        createLineSegmentsTy createLineSegments = nullptr;
        SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createLineSegments);
        createCurvesBezierTy createCurvesBezier = nullptr;
        SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createCurvesBezier);
        createCurvesBSplineTy createCurvesBSpline = nullptr;
        SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createCurvesBSpline);

        Geometry* geom;
        switch (type) {
        case RTC_GEOMETRY_TYPE_CURVE_LINEAR : geom = createLineSegments (device); break;
        case RTC_GEOMETRY_TYPE_CURVE_BEZIER : geom = createCurvesBezier (device,type,RTC_GEOMETRY_SUBTYPE_SURFACE); break;
        case RTC_GEOMETRY_TYPE_CURVE_BSPLINE: geom = createCurvesBSpline(device,type,RTC_GEOMETRY_SUBTYPE_SURFACE); break;
        default:                              geom = nullptr; break;
        }
        return (RTCGeometry) geom->refInc();
#else
        throw_RTCError(RTC_ERROR_UNKNOWN,"rtcNewCurveGeometry is not supported");
#endif
      }
      break;

    case RTC_GEOMETRY_TYPE_SUBDIVISION:
      {
#if defined(EMBREE_GEOMETRY_SUBDIV)
        createSubdivMeshTy createSubdivMesh = nullptr;
        SELECT_SYMBOL_DEFAULT_AVX(device->enabled_cpu_features,createSubdivMesh);
        Geometry* geom = createSubdivMesh(device);
        return (RTCGeometry) geom->refInc();
#else
        throw_RTCError(RTC_ERROR_UNKNOWN,"rtcNewSubdivisionMesh is not supported");
#endif
      }
      break;

    case RTC_GEOMETRY_TYPE_USER:
      {
#if defined(EMBREE_GEOMETRY_USER)
        Geometry* geom = new UserGeometry(device,0,1);
        return (RTCGeometry) geom->refInc();
#else
        throw_RTCError(RTC_ERROR_UNKNOWN,"rtcNewUserGeometry is not supported");
#endif
      }
      break;

    default:
      throw_RTCError(RTC_ERROR_UNKNOWN,"invalid geometry type");
    }

    RTCORE_CATCH_END(device);
    return nullptr;
  }

  RTCORE_API void rtcSetGeometryNumPrimitives(RTCGeometry hgeometry, unsigned int N)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryNumPrimitives);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setNumPrimitives(N);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryNumTimeSteps(RTCGeometry hgeometry, unsigned int N)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryNumTimeSteps);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setNumTimeSteps(N);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometrySubtype(RTCGeometry hgeometry, RTCGeometrySubtype type)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometrySubtype);
    RTCORE_VERIFY_HANDLE(hgeometry);
    if (type != RTC_GEOMETRY_SUBTYPE_RIBBON && type != RTC_GEOMETRY_SUBTYPE_SURFACE)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"invalid geometry subtype");
    geometry->setSubtype(type);
    RTCORE_CATCH_END2(geometry);
  }

  /*! sets the build quality of the geometry */
  RTCORE_API void rtcSetGeometryBuildQuality (RTCGeometry hgeometry, RTCBuildQuality quality) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryBuildQuality);
    RTCORE_VERIFY_HANDLE(hgeometry);
    if (quality != RTC_BUILD_QUALITY_LOW &&
        quality != RTC_BUILD_QUALITY_MEDIUM &&
        quality != RTC_BUILD_QUALITY_HIGH &&
        quality != RTC_BUILD_QUALITY_REFIT)
      throw std::runtime_error("invalid build quality");
    geometry->setBuildQuality(quality);
    RTCORE_CATCH_END2(geometry);
  }
  
  RTCORE_API void rtcSetGeometryMask (RTCGeometry hgeometry, int mask) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryMask);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setMask(mask);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometrySubdivisionMode (RTCGeometry hgeometry, unsigned topologyID, RTCSubdivisionMode mode) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometrySubdivisionMode);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setSubdivisionMode(topologyID,mode);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryIndexBuffer (RTCGeometry hgeometry, RTCBufferType vertexBuffer, RTCBufferType indexBuffer) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryIndexBuffer);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIndexBuffer(vertexBuffer,indexBuffer);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void* rtcNewBuffer(RTCGeometry hgeometry, RTCBufferType type, size_t byteStride, unsigned int numItems) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewBuffer);
    RTCORE_VERIFY_HANDLE(hgeometry);
    return geometry->newBuffer(type,byteStride,numItems);
    RTCORE_CATCH_END2(geometry);
    return nullptr;
  }

  RTCORE_API void rtcSetBuffer(RTCGeometry hgeometry, RTCBufferType type, const void* ptr, size_t offset, size_t stride, unsigned int size)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBuffer);
    RTCORE_VERIFY_HANDLE(hgeometry);
    RTCORE_VERIFY_UPPER(stride,unsigned(inf));
    geometry->setBuffer(type,(void*)ptr,offset,stride,size);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void* rtcGetBuffer(RTCGeometry hgeometry, RTCBufferType type) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetBuffer);
    RTCORE_VERIFY_HANDLE(hgeometry);
    return geometry->getBuffer(type);
    RTCORE_CATCH_END2(geometry);
    return nullptr;
  }
  
  RTCORE_API void rtcEnableGeometry (RTCGeometry hgeometry) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcEnableGeometry);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->enable();
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcUpdateBuffer (RTCGeometry hgeometry, RTCBufferType type) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdateBuffer);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->updateBuffer(type);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcDisableGeometry (RTCGeometry hgeometry) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDisableGeometry);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->disable();
    RTCORE_CATCH_END2(geometry);
  }

    RTCORE_API void rtcSetGeometryTessellationRate (RTCGeometry hgeometry, float tessellationRate)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryTessellationRate);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setTessellationRate(tessellationRate);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryUserData (RTCGeometry hgeometry, void* ptr) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryUserData);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setUserData(ptr);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void* rtcGetGeometryUserData (RTCGeometry hgeometry)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetGeometryUserData);
    RTCORE_VERIFY_HANDLE(hgeometry);
    return geometry->getUserData();
    RTCORE_CATCH_END2(geometry);
    return nullptr;
  }

  RTCORE_API void rtcSetGeometryBoundsFunction (RTCGeometry hgeometry, RTCBoundsFunction bounds, void* userPtr)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryBoundsFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setBoundsFunction(bounds,userPtr);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryDisplacementFunction (RTCGeometry hgeometry, RTCDisplacementFunction func, RTCBounds* bounds)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryDisplacementFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setDisplacementFunction(func,bounds);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryIntersectFunction (RTCGeometry hgeometry, RTCIntersectFunctionN intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryIntersectFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectFunctionN(intersect);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryOccludedFunction (RTCGeometry hgeometry, RTCOccludedFunctionN occluded) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunctionN);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOccludedFunctionN(occluded);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryIntersectFilterFunction (RTCGeometry hgeometry, RTCFilterFunctionN intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryIntersectFilterFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setIntersectionFilterFunctionN(intersect);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcSetGeometryOccludedFilterFunction (RTCGeometry hgeometry, RTCFilterFunctionN intersect) 
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetGeometryOccludedFilterFunction);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->setOcclusionFilterFunctionN(intersect);
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API void rtcInterpolate(RTCGeometry hgeometry, unsigned primID, float u, float v, 
                                 RTCBufferType buffer,
                                 float* P, float* dPdu, float* dPdv, 
                                 float* ddPdudu, float* ddPdvdv, float* ddPdudv, 
                                 unsigned int numFloats)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolate);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->interpolate(primID,u,v,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END2(geometry);
  }

#if defined (EMBREE_RAY_PACKETS)
  RTCORE_API void rtcInterpolateN(RTCGeometry hgeometry,
                                  const void* valid_i, const unsigned* primIDs, const float* u, const float* v, unsigned int numUVs, 
                                  RTCBufferType buffer,
                                  float* P, float* dPdu, float* dPdv, 
                                  float* ddPdudu, float* ddPdvdv, float* ddPdudv, 
                                  unsigned int numFloats)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolateN);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->interpolateN(valid_i,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END2(geometry);
  }
#endif

  RTCORE_API void rtcCommitGeometry (RTCGeometry hgeometry)
  {
    Geometry* geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitGeometry);
    RTCORE_VERIFY_HANDLE(hgeometry);
    return geometry->commit();
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API unsigned int rtcAttachGeometry (RTCScene hscene, RTCGeometry hgeometry)
  {
    Scene* scene = (Scene*) hscene;
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcAttachGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_HANDLE(hgeometry);
    if (scene->device != geometry->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    return scene->bind(RTC_INVALID_GEOMETRY_ID,geometry);
    RTCORE_CATCH_END2(scene);
    return -1;
  }

  RTCORE_API unsigned int rtcAttachAndReleaseGeometry (RTCScene hscene, RTCGeometry hgeometry)
  {
    Scene* scene = (Scene*) hscene;
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcAttachAndReleasGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_HANDLE(hgeometry);
    if (scene->device != geometry->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    unsigned int geomID = scene->bind(RTC_INVALID_GEOMETRY_ID,geometry);
    geometry->refDec();
    return geomID;
    RTCORE_CATCH_END2(scene);
    return -1;
  }

  RTCORE_API unsigned int rtcAttachGeometryByID (RTCScene hscene, RTCGeometry hgeometry, unsigned int geomID)
  {
    Scene* scene = (Scene*) hscene;
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcAttachGeometryByID);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_HANDLE(hgeometry);
    RTCORE_VERIFY_GEOMID(geomID);
    if (scene->device != geometry->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    return scene->bind(geomID,geometry);
    RTCORE_CATCH_END2(scene);
    return -1;
  }

  RTCORE_API unsigned int rtcAttachAndReleaseGeometryByID (RTCScene hscene, RTCGeometry hgeometry, unsigned int geomID_in)
  {
    Scene* scene = (Scene*) hscene;
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcAttachAndReleaseGeometryByID);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_HANDLE(hgeometry);
    RTCORE_VERIFY_GEOMID(geomID_in);
    if (scene->device != geometry->device)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"inputs are from different devices");
    unsigned int geomID = scene->bind(geomID_in,geometry);
    geometry->refDec();
    return geomID;
    RTCORE_CATCH_END2(scene);
    return -1;
  }
  
  RTCORE_API void rtcDetachGeometry (RTCScene hscene, unsigned int geomID)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDetachGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->detachGeometry(geomID);
    RTCORE_CATCH_END2(scene);
  }

  RTCORE_API void rtcReleaseGeometry (RTCGeometry hgeometry)
  {
    Ref<Geometry> geometry = (Geometry*) hgeometry;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcReleaseGeometry);
    RTCORE_VERIFY_HANDLE(hgeometry);
    geometry->refDec();
    RTCORE_CATCH_END2(geometry);
  }

  RTCORE_API RTCGeometry rtcGetGeometry (RTCScene hscene, unsigned int geomID)
  {
    Scene* scene = (Scene*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetGeometry);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
#endif
    return (RTCGeometry) scene->get(geomID);
    RTCORE_CATCH_END2(scene);
    return nullptr;
  }
}
