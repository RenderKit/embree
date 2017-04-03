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
#include "../../include/embree2/rtcore_ray.h"
#include "../../common/sys/library.h"

namespace embree
{  
  MutexSys g_api_mutex;

  static ErrorHandler g_errorHandler;
  
  void setThreadErrorCode(RTCError error)
  {
    RTCError* stored_error = g_errorHandler.error();
    if (*stored_error == RTC_NO_ERROR)
      *stored_error = error;
  }

  RTCError getThreadErrorCode()
  {
    RTCError* stored_error = g_errorHandler.error();
    RTCError error = *stored_error;
    *stored_error = RTC_NO_ERROR;
    return error;
  }
  
  void process_error(DeviceInterface* device, RTCError error, const char* str)
  {
    /* store global error code when device construction failed */
    if (!device) return setThreadErrorCode(error);
    else         return device->processError(error,str);
  }

  static std::map<DeviceInterface*,size_t> g_cache_size_map;
  static std::map<DeviceInterface*,size_t> g_num_threads_map;

  /*! some global variables that can be set via rtcSetParameter1i for debugging purposes */
  ssize_t DeviceInterface::debug_int0 = 0;
  ssize_t DeviceInterface::debug_int1 = 0;
  ssize_t DeviceInterface::debug_int2 = 0;
  ssize_t DeviceInterface::debug_int3 = 0;

  size_t DeviceInterface::getMaxNumThreads()
  {
    size_t maxNumThreads = 0;
    for (std::map<DeviceInterface*,size_t>::iterator i=g_num_threads_map.begin(); i != g_num_threads_map.end(); i++)
      maxNumThreads = max(maxNumThreads, (*i).second);
    if (maxNumThreads == 0)
      maxNumThreads = std::numeric_limits<size_t>::max();
    return maxNumThreads;
  }

  size_t DeviceInterface::getMaxCacheSize()
  {
    size_t maxCacheSize = 0;
    for (std::map<DeviceInterface*,size_t>::iterator i=g_cache_size_map.begin(); i!= g_cache_size_map.end(); i++)
      maxCacheSize = max(maxCacheSize, (*i).second);
    return maxCacheSize;
  }
 
  void DeviceInterface::setCacheSize(DeviceInterface* device, size_t bytes) 
  {
    if (bytes == 0) g_cache_size_map.erase(device);
    else            g_cache_size_map[device] = bytes;
  }

  void DeviceInterface::setNumThreads(DeviceInterface* device, size_t numThreads) 
  {
    if (numThreads == 0) 
      g_num_threads_map[device] = std::numeric_limits<size_t>::max();
    else 
      g_num_threads_map[device] = numThreads;
  }

  bool DeviceInterface::unsetNumThreads(DeviceInterface* device) {
    g_num_threads_map.erase(device);
    return g_num_threads_map.size() == 0;
  }

  namespace sse2      { extern DeviceInterface* createDevice(const State& state); }
  namespace sse42     { extern DeviceInterface* createDevice(const State& state); }
  namespace avx       { extern DeviceInterface* createDevice(const State& state); }
  namespace avx2      { extern DeviceInterface* createDevice(const State& state); }
  namespace avx512knl { extern DeviceInterface* createDevice(const State& state); }
  namespace avx512skx { extern DeviceInterface* createDevice(const State& state); }

  DeviceInterface* createDevice(const char* cfg, bool single)
  {
    const State state(cfg,single);
    DeviceInterface* (*newDevice) (const State& state) = nullptr;
#if defined(__TARGET_SSE2__)
    if (state.hasISA(SSE2)) newDevice = &sse2::createDevice;
#endif
#if defined(__TARGET_SSE42__)
    if (state.hasISA(SSE42)) newDevice = &sse42::createDevice;
#endif
#if defined(__TARGET_AVX__)
    if (state.hasISA(AVX)) newDevice = &avx::createDevice;
#endif
#if defined(__TARGET_AVX2__)
    if (state.hasISA(AVX2)) newDevice = &avx2::createDevice;
#endif
#if defined(__TARGET_AVX512KNL__)
    if (state.hasISA(AVX512KNL)) newDevice = &avx512knl::createDevice;
#endif
#if defined(__TARGET_AVX512SKX__)
    if (state.hasISA(AVX512SKX)) newDevice = &avx512skx::createDevice;
#endif
    if (newDevice == nullptr) 
      throw_RTCError(RTC_UNKNOWN_ERROR,"cannot create device");
    
    return newDevice(state);
  }

  RTCORE_API RTCDevice rtcNewDevice(const char* cfg)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewDevice);
    Lock<MutexSys> lock(g_api_mutex);
    return (RTCDevice) createDevice(cfg,false);
    RTCORE_CATCH_END(nullptr);
    return (RTCDevice) nullptr;
  }

  RTCORE_API void rtcDeleteDevice(RTCDevice device) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteDevice);
    RTCORE_VERIFY_HANDLE(device);
    Lock<MutexSys> lock(g_api_mutex);
    delete (DeviceInterface*) device;
    RTCORE_CATCH_END(nullptr);
  }

  /* global device for compatibility with old rtcInit / rtcExit scheme */
  static DeviceInterface* g_device = nullptr;

  RTCORE_API void rtcInit(const char* cfg) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInit);
    Lock<MutexSys> lock(g_api_mutex);
    if (g_device) throw_RTCError(RTC_INVALID_OPERATION,"already initialized");
    g_device = createDevice(cfg,true);
    RTCORE_CATCH_END(g_device);
  }
  
  RTCORE_API void rtcExit() 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcExit);
    Lock<MutexSys> lock(g_api_mutex);
    if (!g_device) throw_RTCError(RTC_INVALID_OPERATION,"rtcInit has to get called before rtcExit");
    delete g_device; g_device = nullptr;
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcSetParameter1i(const RTCParameter parm, ssize_t val)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetParameter1i);
    Lock<MutexSys> lock(g_api_mutex);
    if (g_device) g_device->setParameter1i(parm,val);
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API ssize_t rtcGetParameter1i(const RTCParameter parm)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetParameter1i);
    assert(g_device);
    Lock<MutexSys> lock(g_api_mutex);
    return g_device->getParameter1i(parm);
    RTCORE_CATCH_END(g_device);
    return 0;
  }

  RTCORE_API void rtcDeviceSetParameter1i(RTCDevice hdevice, const RTCParameter parm, ssize_t val)
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetParameter1i);
    const bool internal_parm = (size_t)parm >= 1000000 && (size_t)parm < 1000004;
    if (!internal_parm) RTCORE_VERIFY_HANDLE(hdevice); // allow NULL device for special internal settings
    Lock<MutexSys> lock(g_api_mutex);
    if (device) device->setParameter1i(parm,val);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API ssize_t rtcDeviceGetParameter1i(RTCDevice hdevice, const RTCParameter parm)
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceGetParameter1i);
    RTCORE_VERIFY_HANDLE(hdevice);
    Lock<MutexSys> lock(g_api_mutex);

    size_t iparm = (size_t)parm;

    /* get name of internal regression test */
    if (iparm >= 2000000 && iparm < 3000000)
    {
      RegressionTest* test = getRegressionTest(iparm-2000000);
      if (test) return (ssize_t) test->name.c_str();
      else      return 0;
    }

    /* run internal regression test */
    if (iparm >= 3000000 && iparm < 4000000)
    {
      RegressionTest* test = getRegressionTest(iparm-3000000);
      if (test) return test->run();
      else      return 0;
    }

    if (device)
      return device->getParameter1i(parm);

    RTCORE_CATCH_END(device);
    return 0;
  }

  RTCORE_API RTCError rtcGetError()
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetError);
    if (g_device == nullptr) return getThreadErrorCode();
    else                     return g_device->getDeviceErrorCode();
    RTCORE_CATCH_END(g_device);
    return RTC_UNKNOWN_ERROR;
  }

  RTCORE_API RTCError rtcDeviceGetError(RTCDevice hdevice)
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceGetError);
    if (device == nullptr) return getThreadErrorCode();
    else                   return device->getDeviceErrorCode();
    RTCORE_CATCH_END(device);
    return RTC_UNKNOWN_ERROR;
  }

  RTCORE_API void rtcSetErrorFunction(RTCErrorFunc func) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetErrorFunction);
    assert(g_device);
    if (g_device) g_device->setErrorFunction(func);
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcDeviceSetErrorFunction(RTCDevice hdevice, RTCErrorFunc func) 
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetErrorFunction);
    RTCORE_VERIFY_HANDLE(hdevice);
    device->setErrorFunction(func);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcDeviceSetErrorFunction2(RTCDevice hdevice, RTCErrorFunc2 func, void* userPtr) 
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetErrorFunction2);
    RTCORE_VERIFY_HANDLE(hdevice);
    device->setErrorFunction(func,userPtr);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcSetMemoryMonitorFunction(RTCMemoryMonitorFunc func) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetMemoryMonitorFunction);
    assert(g_device);
    if (g_device) g_device->setMemoryMonitorFunction(func);
    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API void rtcDeviceSetMemoryMonitorFunction(RTCDevice hdevice, RTCMemoryMonitorFunc func) 
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetMemoryMonitorFunction);
    device->setMemoryMonitorFunction(func);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcDeviceSetMemoryMonitorFunction2(RTCDevice hdevice, RTCMemoryMonitorFunc2 func, void* userPtr) 
  {
    DeviceInterface* device = (DeviceInterface*) hdevice;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceSetMemoryMonitorFunction2);
    device->setMemoryMonitorFunction(func,userPtr);
    RTCORE_CATCH_END(device);
  }

  RTCORE_API void rtcDebug() 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDebug);

/*#if defined(EMBREE_STAT_COUNTERS)
    Stat::print(std::cout);
    Stat::clear();
#endif*/

    RTCORE_CATCH_END(g_device);
  }

  RTCORE_API RTCScene rtcNewScene (RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewScene);
    assert(g_device);
    //if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_INCOHERENT);
    return (RTCScene) g_device->newScene(flags,aflags);
    RTCORE_CATCH_END(g_device);
    return nullptr;
  }

  RTCORE_API RTCScene rtcDeviceNewScene (RTCDevice device, RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeviceNewScene);
    RTCORE_VERIFY_HANDLE(device);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_INCOHERENT);
    return (RTCScene) ((DeviceInterface*)device)->newScene(flags,aflags);
    RTCORE_CATCH_END((DeviceInterface*)device);
    return nullptr;
  }

  RTCORE_API void rtcSetProgressMonitorFunction(RTCScene hscene, RTCProgressMonitorFunc func, void* ptr) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetProgressMonitorFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    Lock<MutexSys> lock(g_api_mutex);
    scene->setProgressMonitorFunction(func,ptr);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcCommit (RTCScene hscene) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommit);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->commit(0,0,true);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcCommitJoin (RTCScene hscene) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitJoin);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->commit(0,0,false);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcCommitThread(RTCScene hscene, unsigned int threadID, unsigned int numThreads) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitThread);
    RTCORE_VERIFY_HANDLE(hscene);

    if (unlikely(numThreads == 0)) 
      throw_RTCError(RTC_INVALID_OPERATION,"invalid number of threads specified");

    /* for best performance set FTZ and DAZ flags in the MXCSR control and status register */
    unsigned int mxcsr = _mm_getcsr();
    _mm_setcsr(mxcsr | /* FTZ */ (1<<15) | /* DAZ */ (1<<6));
    
    /* perform scene build */
    scene->commit(threadID,numThreads,false);

    /* reset MXCSR register again */
    _mm_setcsr(mxcsr);
    
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcGetBounds(RTCScene hscene, RTCBounds& bounds_o)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetBounds);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->rtcGetBounds(bounds_o);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcGetLinearBounds(RTCScene hscene, RTCBounds* bounds_o)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetBounds);
    RTCORE_VERIFY_HANDLE(hscene);
    scene->rtcGetLinearBounds(bounds_o);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcIntersect (RTCScene hscene, RTCRay& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect(ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect1Ex (RTCScene hscene, const RTCIntersectContext* user_context, RTCRay& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect1Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect1Ex(user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect4 (const void* valid, RTCScene hscene, RTCRay4& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect4);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect4(valid,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect4Ex (const void* valid, RTCScene hscene, const RTCIntersectContext* user_context, RTCRay4& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect4Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect4Ex(valid,user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcIntersect8 (const void* valid, RTCScene hscene, RTCRay8& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect8);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect8(valid,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect8Ex (const void* valid, RTCScene hscene, const RTCIntersectContext* user_context, RTCRay8& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect8Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect8Ex(valid,user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcIntersect16 (const void* valid, RTCScene hscene, RTCRay16& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect16);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect16(valid,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect16Ex (const void* valid, RTCScene hscene, const RTCIntersectContext* user_context, RTCRay16& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect16Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect16Ex(valid,user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect1M (RTCScene hscene, const RTCIntersectContext* user_context, RTCRay* rays, const size_t M, const size_t stride) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect1M);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect1M(user_context,rays,M,stride);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersect1Mp (RTCScene hscene, const RTCIntersectContext* user_context, RTCRay** rays, const size_t M) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect1Mp);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersect1Mp(user_context,rays,M);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersectNM (RTCScene hscene, const RTCIntersectContext* user_context, struct RTCRayN* rays, const size_t N, const size_t M, const size_t stride) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersectNM);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersectNM(user_context,rays,N,M,stride);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcIntersectNp (RTCScene hscene, const RTCIntersectContext* user_context, const RTCRayNp& rays, const size_t N) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersectNp);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcIntersectNp(user_context,rays,N);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcOccluded (RTCScene hscene, RTCRay& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded(ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccluded1Ex (RTCScene hscene, const RTCIntersectContext* user_context, RTCRay& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded1Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded1Ex(user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcOccluded4 (const void* valid, RTCScene hscene, RTCRay4& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded4);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded4(valid,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccluded4Ex (const void* valid, RTCScene hscene, const RTCIntersectContext* user_context, RTCRay4& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded4Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded4Ex(valid,user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }
 
  RTCORE_API void rtcOccluded8 (const void* valid, RTCScene hscene, RTCRay8& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded8);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded8(valid,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccluded8Ex (const void* valid, RTCScene hscene, const RTCIntersectContext* user_context, RTCRay8& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded8Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded8Ex(valid,user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcOccluded16 (const void* valid, RTCScene hscene, RTCRay16& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded16);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded16(valid,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccluded16Ex (const void* valid, RTCScene hscene, const RTCIntersectContext* user_context, RTCRay16& ray) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded16Ex);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded16Ex(valid,user_context,ray);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcOccluded1M(RTCScene hscene, const RTCIntersectContext* user_context, RTCRay* rays, const size_t M, const size_t stride) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded1M);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded1M(user_context,rays,M,stride);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccluded1Mp(RTCScene hscene, const RTCIntersectContext* user_context, RTCRay** rays, const size_t M) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded1Mp);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccluded1Mp(user_context,rays,M);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccludedNM(RTCScene hscene, const RTCIntersectContext* user_context, RTCRayN* rays, const size_t N, const size_t M, const size_t stride) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccludedNM);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccludedNM (user_context,rays,N,M,stride);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcOccludedNp(RTCScene hscene, const RTCIntersectContext* user_context, const RTCRayNp& rays, const size_t N) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccludedNp);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(hscene);
#endif
    scene->rtcOccludedNp(user_context,rays,N);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcDeleteScene (RTCScene hscene) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    DeviceInterface* device = scene ? scene->getDevice() : nullptr;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteScene);
    RTCORE_VERIFY_HANDLE(hscene);
    delete scene;
    RTCORE_CATCH_END(device);
  }

  RTCORE_API unsigned rtcNewInstance (RTCScene htarget, RTCScene hsource) 
  {
    SceneInterface* target = (SceneInterface*) htarget;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewInstance);
    RTCORE_VERIFY_HANDLE(htarget);
    RTCORE_VERIFY_HANDLE(hsource);
#if defined(EMBREE_GEOMETRY_USER)
    SceneInterface* source = (SceneInterface*) hsource;
    if (target->getDevice() != source->getDevice()) throw_RTCError(RTC_INVALID_OPERATION,"scenes do not belong to the same device");
    return target->newInstance(source,1);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewInstance is not supported");
#endif
    RTCORE_CATCH_END(target->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewInstance2 (RTCScene htarget, RTCScene hsource, size_t numTimeSteps) 
  {
    SceneInterface* target = (SceneInterface*) htarget;
    SceneInterface* source = (SceneInterface*) hsource;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewInstance2);
    RTCORE_VERIFY_HANDLE(htarget);
    RTCORE_VERIFY_HANDLE(hsource);
#if defined(EMBREE_GEOMETRY_USER)
    if (target->getDevice() != source->getDevice()) throw_RTCError(RTC_INVALID_OPERATION,"scenes do not belong to the same device");
    return target->newInstance(source,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewInstance2 is not supported");
#endif
    RTCORE_CATCH_END(target->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewGeometryInstance (RTCScene hscene, unsigned geomID) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewGeometryInstance);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    return scene->newGeometryInstance(geomID);
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewGeometryGroup (RTCScene hscene, RTCGeometryFlags flags, unsigned* geomIDs, size_t N) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewGeometryGroup);
    RTCORE_VERIFY_HANDLE(hscene);
    for (size_t i=0; i<N; i++) RTCORE_VERIFY_GEOMID(geomIDs[i]);
    return scene->newGeometryGroup(flags,geomIDs,N);
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
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
      throw_RTCError(RTC_INVALID_OPERATION,"Unknown matrix type");
      break;
    }
    return transform;
  }

  RTCORE_API void rtcSetTransform (RTCScene hscene, unsigned geomID, RTCMatrixType layout, const float* xfm) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetTransform);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    RTCORE_VERIFY_HANDLE(xfm);
    const AffineSpace3fa transform = convertTransform(layout,xfm);
    ((SceneInterface*) scene)->getGeometryLocked(geomID)->setTransform(transform,0);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetTransform2 (RTCScene hscene, unsigned geomID, RTCMatrixType layout, const float* xfm, size_t timeStep) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetTransform);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    RTCORE_VERIFY_HANDLE(xfm);
    const AffineSpace3fa transform = convertTransform(layout,xfm);
    ((SceneInterface*) scene)->getGeometryLocked(geomID)->setTransform(transform,timeStep);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API unsigned rtcNewUserGeometry (RTCScene hscene, size_t numItems) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewUserGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_USER)
    return scene->newUserGeometry(RTC_GEOMETRY_STATIC,numItems,1);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewUserGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewUserGeometry2 (RTCScene hscene, size_t numItems, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewUserGeometry2);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_USER)
    return scene->newUserGeometry(RTC_GEOMETRY_STATIC,numItems,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewUserGeometry2 is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewUserGeometry3 (RTCScene hscene, RTCGeometryFlags gflags, size_t numItems, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewUserGeometry2);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_USER)
    return scene->newUserGeometry(gflags,numItems,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewUserGeometry3 is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewTriangleMesh (RTCScene hscene, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewTriangleMesh);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_TRIANGLES)
    return scene->newTriangleMesh(flags,numTriangles,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewTriangleMesh is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewQuadMesh (RTCScene hscene, RTCGeometryFlags flags, size_t numQuads, size_t numVertices, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewQuadMesh);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_QUADS)
    return scene->newQuadMesh(flags,numQuads,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewQuadMesh is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewHairGeometry (RTCScene hscene, RTCGeometryFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewHairGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_HAIR)
    return scene->newCurves(isa::NativeCurves::HAIR,isa::NativeCurves::BEZIER,flags,numCurves,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewHairGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewBezierHairGeometry (RTCScene hscene, RTCGeometryFlags flags, unsigned int numCurves, unsigned int numVertices, unsigned int numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewBezierHairGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_HAIR)
    return scene->newCurves(isa::NativeCurves::HAIR,isa::NativeCurves::BEZIER,flags,numCurves,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewBezierHairGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }
  
  RTCORE_API unsigned rtcNewBSplineHairGeometry (RTCScene hscene, RTCGeometryFlags flags, unsigned int numCurves, unsigned int numVertices, unsigned int numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewBSplineHairGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_HAIR)
    return scene->newCurves(isa::NativeCurves::HAIR,isa::NativeCurves::BSPLINE,flags,numCurves,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewBSplineHairGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewCurveGeometry (RTCScene hscene, RTCGeometryFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewCurveGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_HAIR)
    return scene->newCurves(isa::NativeCurves::SURFACE,isa::NativeCurves::BEZIER,flags,numCurves,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewCurveGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewBezierCurveGeometry (RTCScene hscene, RTCGeometryFlags flags, unsigned int numCurves, unsigned int numVertices, unsigned int numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewBezierCurveGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_HAIR)
    return scene->newCurves(isa::NativeCurves::SURFACE,isa::NativeCurves::BEZIER,flags,numCurves,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewBezierCurveGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewBSplineCurveGeometry (RTCScene hscene, RTCGeometryFlags flags, unsigned int numCurves, unsigned int numVertices, unsigned int numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewBSplineCurveGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_HAIR)
    return scene->newCurves(isa::NativeCurves::SURFACE,isa::NativeCurves::BSPLINE,flags,numCurves,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewBSplineCurveGeometry is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewLineSegments (RTCScene hscene, RTCGeometryFlags flags, size_t numSegments, size_t numVertices, size_t numTimeSteps)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewLineSegments);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_LINES)
    return scene->newLineSegments(flags,numSegments,numVertices,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewLineSegments is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API unsigned rtcNewSubdivisionMesh (RTCScene hscene, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, 
                                             size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewSubdivisionMesh);
    RTCORE_VERIFY_HANDLE(hscene);
#if defined(EMBREE_GEOMETRY_SUBDIV)
    return scene->newSubdivisionMesh(flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
#else
    throw_RTCError(RTC_UNKNOWN_ERROR,"rtcNewSubdivisionMesh is not supported");
#endif
    RTCORE_CATCH_END(scene->getDevice());
    return -1;
  }

  RTCORE_API void rtcSetMask (RTCScene hscene, unsigned geomID, int mask) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetMask);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setMask(mask);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetBoundaryMode (RTCScene hscene, unsigned geomID, RTCBoundaryMode mode) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundaryMode);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setSubdivisionMode(0,(RTCSubdivisionMode)mode);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetSubdivisionMode (RTCScene hscene, unsigned geomID, unsigned topologyID, RTCSubdivisionMode mode) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetSubdivisionMode);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setSubdivisionMode(topologyID,mode);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIndexBuffer (RTCScene hscene, unsigned geomID, RTCBufferType vertexBuffer, RTCBufferType indexBuffer) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIndexBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIndexBuffer(vertexBuffer,indexBuffer);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void* rtcMapBuffer(RTCScene hscene, unsigned geomID, RTCBufferType type) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcMapBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    return scene->getGeometryLocked(geomID)->map(type);
    RTCORE_CATCH_END(scene->getDevice());
    return nullptr;
  }

  RTCORE_API void rtcUnmapBuffer(RTCScene hscene, unsigned geomID, RTCBufferType type) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUnmapBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->unmap(type);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetBuffer(RTCScene hscene, unsigned geomID, RTCBufferType type, const void* ptr, size_t offset, size_t stride)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setBuffer(type,(void*)ptr,offset,stride,-1);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetBuffer2(RTCScene hscene, unsigned geomID, RTCBufferType type, const void* ptr, size_t offset, size_t stride, size_t size)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBuffer2);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setBuffer(type,(void*)ptr,offset,stride,size);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcEnable (RTCScene hscene, unsigned geomID) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcEnable);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->enable();
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcUpdate (RTCScene hscene, unsigned geomID) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdate);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->update();
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcUpdateBuffer (RTCScene hscene, unsigned geomID, RTCBufferType type) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdateBuffer);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->updateBuffer(type);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcDisable (RTCScene hscene, unsigned geomID) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDisable);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->disable();
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcDeleteGeometry (RTCScene hscene, unsigned geomID) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteGeometry);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->deleteGeometry(geomID);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetTessellationRate (RTCScene hscene, unsigned geomID, float tessellationRate)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetTessellationRate);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setTessellationRate(tessellationRate);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetUserData (RTCScene hscene, unsigned geomID, void* ptr) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetUserData);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setUserData(ptr);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void* rtcGetUserData (RTCScene hscene, unsigned geomID)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetUserData);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    return scene->getGeometry(geomID)->getUserData(); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->getDevice());
    return nullptr;
  }

  RTCORE_API void rtcSetBoundsFunction (RTCScene hscene, unsigned geomID, RTCBoundsFunc bounds)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundsFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setBoundsFunction(bounds);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetBoundsFunction2 (RTCScene hscene, unsigned geomID, RTCBoundsFunc2 bounds, void* userPtr)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundsFunction2);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setBoundsFunction2(bounds,userPtr);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetBoundsFunction3 (RTCScene hscene, unsigned geomID, RTCBoundsFunc3 bounds, void* userPtr)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundsFunction3);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setBoundsFunction3(bounds,userPtr);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetDisplacementFunction (RTCScene hscene, unsigned geomID, RTCDisplacementFunc func, RTCBounds* bounds)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDisplacementFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setDisplacementFunction(func,bounds);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetDisplacementFunction2 (RTCScene hscene, unsigned geomID, RTCDisplacementFunc2 func, RTCBounds* bounds)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDisplacementFunction2);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setDisplacementFunction2(func,bounds);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectFunction (RTCScene hscene, unsigned geomID, RTCIntersectFunc intersect) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectFunction(intersect);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectFunction4 (RTCScene hscene, unsigned geomID, RTCIntersectFunc4 intersect4) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectFunction4(intersect4);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectFunction8 (RTCScene hscene, unsigned geomID, RTCIntersectFunc8 intersect8) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectFunction8(intersect8);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcSetIntersectFunction16 (RTCScene hscene, unsigned geomID, RTCIntersectFunc16 intersect16) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectFunction16(intersect16);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectFunction1Mp (RTCScene hscene, unsigned geomID, RTCIntersectFunc1Mp intersect) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction1Mp);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectFunction1Mp(intersect);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectFunctionN (RTCScene hscene, unsigned geomID, RTCIntersectFuncN intersect) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunctionN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectFunctionN(intersect);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOccludedFunction (RTCScene hscene, unsigned geomID, RTCOccludedFunc occluded) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOccludedFunction(occluded);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOccludedFunction4 (RTCScene hscene, unsigned geomID, RTCOccludedFunc4 occluded4) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOccludedFunction4(occluded4);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOccludedFunction8 (RTCScene hscene, unsigned geomID, RTCOccludedFunc8 occluded8) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOccludedFunction8(occluded8);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOccludedFunction16 (RTCScene hscene, unsigned geomID, RTCOccludedFunc16 occluded16) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOccludedFunction16(occluded16);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOccludedFunction1Mp (RTCScene hscene, unsigned geomID, RTCOccludedFunc1Mp occluded) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction1Mp);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOccludedFunction1Mp(occluded);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOccludedFunctionN (RTCScene hscene, unsigned geomID, RTCOccludedFuncN occluded) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunctionN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOccludedFunctionN(occluded);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectionFilterFunction (RTCScene hscene, unsigned geomID, RTCFilterFunc intersect) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectionFilterFunction(intersect);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectionFilterFunction4 (RTCScene hscene, unsigned geomID, RTCFilterFunc4 filter4) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectionFilterFunction4(filter4);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectionFilterFunction8 (RTCScene hscene, unsigned geomID, RTCFilterFunc8 filter8) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectionFilterFunction8(filter8);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcSetIntersectionFilterFunction16 (RTCScene hscene, unsigned geomID, RTCFilterFunc16 filter16) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectionFilterFunction16(filter16);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetIntersectionFilterFunctionN (RTCScene hscene, unsigned geomID, RTCFilterFuncN filterN) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunctionN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setIntersectionFilterFunctionN(filterN);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOcclusionFilterFunction (RTCScene hscene, unsigned geomID, RTCFilterFunc intersect) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOcclusionFilterFunction(intersect);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOcclusionFilterFunction4 (RTCScene hscene, unsigned geomID, RTCFilterFunc4 filter4) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction4);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOcclusionFilterFunction4(filter4);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOcclusionFilterFunction8 (RTCScene hscene, unsigned geomID, RTCFilterFunc8 filter8) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction8);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOcclusionFilterFunction8(filter8);
    RTCORE_CATCH_END(scene->getDevice());
  }
  
  RTCORE_API void rtcSetOcclusionFilterFunction16 (RTCScene hscene, unsigned geomID, RTCFilterFunc16 filter16) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction16);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOcclusionFilterFunction16(filter16);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcSetOcclusionFilterFunctionN (RTCScene hscene, unsigned geomID, RTCFilterFuncN filterN) 
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunctionN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometryLocked(geomID)->setOcclusionFilterFunctionN(filterN);
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcInterpolate(RTCScene hscene, unsigned geomID, unsigned primID, float u, float v, 
                                 RTCBufferType buffer,
                                 float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolate);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometry(geomID)->interpolate(primID,u,v,buffer,P,dPdu,dPdv,nullptr,nullptr,nullptr,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->getDevice());
  }

  RTCORE_API void rtcInterpolate2(RTCScene hscene, unsigned geomID, unsigned primID, float u, float v, 
                                  RTCBufferType buffer,
                                  float* P, float* dPdu, float* dPdv, 
                                  float* ddPdudu, float* ddPdvdv, float* ddPdudv, 
                                  size_t numFloats)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolate);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometry(geomID)->interpolate(primID,u,v,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->getDevice());
  }


#if defined (EMBREE_RAY_PACKETS)
  RTCORE_API void rtcInterpolateN(RTCScene hscene, unsigned geomID, 
                                  const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                                  RTCBufferType buffer,
                                  float* P, float* dPdu, float* dPdv, size_t numFloats)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolateN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometry(geomID)->interpolateN(valid_i,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,nullptr,nullptr,nullptr,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->getDevice());
  }
#endif

#if defined (EMBREE_RAY_PACKETS)
  RTCORE_API void rtcInterpolateN2(RTCScene hscene, unsigned geomID, 
                                   const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                                   RTCBufferType buffer,
                                   float* P, float* dPdu, float* dPdv, 
                                   float* ddPdudu, float* ddPdvdv, float* ddPdudv, 
                                   size_t numFloats)
  {
    SceneInterface* scene = (SceneInterface*) hscene;
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolateN);
    RTCORE_VERIFY_HANDLE(hscene);
    RTCORE_VERIFY_GEOMID(geomID);
    scene->getGeometry(geomID)->interpolateN(valid_i,primIDs,u,v,numUVs,buffer,P,dPdu,dPdv,ddPdudu,ddPdvdv,ddPdudv,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END(scene->getDevice());
  }
#endif
}
