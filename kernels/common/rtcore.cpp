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

#include "default.h"
#include "version.h"
#include "scene.h"
#include "raystream_log.h"

#if !defined(_MM_SET_DENORMALS_ZERO_MODE)
#define _MM_DENORMALS_ZERO_ON   (0x0040)
#define _MM_DENORMALS_ZERO_OFF  (0x0000)
#define _MM_DENORMALS_ZERO_MASK (0x0040)
#define _MM_SET_DENORMALS_ZERO_MODE(x) (_mm_setcsr((_mm_getcsr() & ~_MM_DENORMALS_ZERO_MASK) | (x)))
#endif

#if defined(TASKING_LOCKSTEP)
#  include "tasking/taskscheduler_mic.h"
#elif defined(TASKING_TBB_INTERNAL)
#  include "tasking/taskscheduler_tbb.h"
#endif

namespace embree
{  
  /* functions to initialize global state */
  void init_globals();

  /* register functions for accels */
  void BVH4Register();
  void BVH8Register();

#if defined(__MIC__)
  void BVH4iRegister();
  void BVH4MBRegister();
  void BVH4HairRegister();
#endif

  /*! intersector registration functions */
  DECLARE_SYMBOL(RTCBoundsFunc,InstanceBoundsFunc);
  DECLARE_SYMBOL(AccelSet::Intersector1,InstanceIntersector1);
  DECLARE_SYMBOL(AccelSet::Intersector4,InstanceIntersector4);
  DECLARE_SYMBOL(AccelSet::Intersector8,InstanceIntersector8);
  DECLARE_SYMBOL(AccelSet::Intersector16,InstanceIntersector16);
  
#if defined(TASKING_TBB)
  bool g_tbb_threads_initialized = false;
  tbb::task_scheduler_init tbb_threads(tbb::task_scheduler_init::deferred);

  class TBBAffinity: public tbb::task_scheduler_observer
  {
    void on_scheduler_entry( bool ) {
      setAffinity(TaskSchedulerTBB::threadIndex());
    }
  } tbb_affinity;
#endif

  void memoryMonitor(ssize_t bytes, bool post)
  {
    if (State::instance()->g_memory_monitor_function && bytes != 0) {
      if (!State::instance()->g_memory_monitor_function(bytes,post)) {
#if !defined(TASKING_LOCKSTEP) && !defined(TASKING_TBB_INTERNAL)
        if (bytes > 0) { // only throw exception when we allocate memory to never throw inside a destructor
          throw_RTCError(RTC_OUT_OF_MEMORY,"memory monitor forced termination");
        }
#endif
      }
    }
  }

  void process_error(RTCError error, const char* str)
  { 
    /* print error when in verbose mode */
    if (State::instance()->verbosity(1)) 
    {
      switch (error) {
      case RTC_NO_ERROR         : std::cerr << "Embree: No error"; break;
      case RTC_UNKNOWN_ERROR    : std::cerr << "Embree: Unknown error"; break;
      case RTC_INVALID_ARGUMENT : std::cerr << "Embree: Invalid argument"; break;
      case RTC_INVALID_OPERATION: std::cerr << "Embree: Invalid operation"; break;
      case RTC_OUT_OF_MEMORY    : std::cerr << "Embree: Out of memory"; break;
      case RTC_UNSUPPORTED_CPU  : std::cerr << "Embree: Unsupported CPU"; break;
      default                   : std::cerr << "Embree: Invalid error code"; break;                   
      };
      if (str) std::cerr << ", (" << str << ")";
      std::cerr << std::endl;
    }

    /* call user specified error callback */
    if (State::instance()->g_error_function) 
      State::instance()->g_error_function(error,str); 

    /* record error code */
    RTCError* stored_error = State::error();
    if (*stored_error == RTC_NO_ERROR)
      *stored_error = error;
  }

  /* mutex to make API thread safe */
  static MutexSys g_mutex;

  /* set if embree got initialized */
  static bool g_initialized = false;

  void InstanceIntersectorsRegister ()
  {
    int features = getCPUFeatures();
#if defined(__MIC__)
    SELECT_SYMBOL_KNC(features,InstanceBoundsFunc);
    SELECT_SYMBOL_KNC(features,InstanceIntersector1);
    SELECT_SYMBOL_KNC(features,InstanceIntersector16);
#else
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceBoundsFunc);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceIntersector1);
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceIntersector4);
    SELECT_SYMBOL_AVX_AVX2(features,InstanceIntersector8);
#endif
  }

#if defined(TASKING_LOCKSTEP)

  LockStepTaskScheduler regression_task_scheduler;

  void task_regression_testing(void* This, size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* taskGroup) 
  {
    LockStepTaskScheduler::setInstance(&regression_task_scheduler);
    LockStepTaskScheduler::Init init(threadIndex,threadCount,&regression_task_scheduler);
    if (threadIndex != 0) return;
    runRegressionTests();
  }
#endif

  void print_info()
  {
    std::cout << "Embree Ray Tracing Kernels " << __EMBREE_VERSION__ << " (" << __DATE__ << ")" << std::endl;
    std::cout << "  Compiler : " << getCompilerName() << std::endl;
    std::cout << "  Platform : " << getPlatformName() << std::endl;
    std::cout << "  CPU      : " << stringOfCPUModel(getCPUModel()) << " (" << getCPUVendor() << ")" << std::endl;
    std::cout << "  ISA      : " << stringOfCPUFeatures(getCPUFeatures()) << std::endl;
#if !defined(__MIC__)
    const bool hasFTZ = _mm_getcsr() & _MM_FLUSH_ZERO_ON;
    const bool hasDAZ = _mm_getcsr() & _MM_DENORMALS_ZERO_ON;
    std::cout << "  MXCSR    : " << "FTZ=" << hasFTZ << ", DAZ=" << hasDAZ << std::endl;
#endif
    std::cout << "  Config   : ";
#if defined(TASKING_TBB)
    std::cout << "TBB ";
#endif
#if defined(__TARGET_SSE41__)
    std::cout << "SSE4.1 ";
#endif
#if defined(__TARGET_SSE42__)
    std::cout << "SSE4.2 ";
#endif
#if defined(__TARGET_AVX__)
    std::cout << "AVX ";
#endif
#if defined(__TARGET_AVX2__)
    std::cout << "AVX2 ";
#endif
#if defined(__TARGET_AVX512__)
    std::cout << "AVX512 ";
#endif
#if defined(TASKING_TBB_INTERNAL)
    std::cout << "internal_tasking_system ";
#endif
#if defined(TASKING_LOCKSTEP)
    std::cout << "internal_tasking_system ";
#endif
#if defined(RTCORE_RAY_MASK)
    std::cout << "raymasks ";
#endif
#if defined (RTCORE_BACKFACE_CULLING)
    std::cout << "backfaceculling ";
#endif
#if defined(RTCORE_INTERSECTION_FILTER)
    std::cout << "intersection_filter ";
#endif
#if defined(RTCORE_BUFFER_STRIDE)
    std::cout << "bufferstride ";
#endif
    std::cout << std::endl;

    /* check of FTZ and DAZ flags are set in CSR */
#if !defined(__MIC__)
    if (!hasFTZ || !hasDAZ) {
#if !defined(_DEBUG)
      if (State::instance()->verbosity(1)) 
#endif
      {
        std::cout << std::endl;
        std::cout << "================================================================================" << std::endl;
        std::cout << "WARNING: \"Flush to Zero\" or \"Denormals are Zero\" mode not enabled " << std::endl 
                  << "         in the MXCSR control and status register. This can have a severe " << std::endl
                  << "         performance impact. Please enable these modes for each application " << std::endl
                  << "         thread the following way:" << std::endl
                  << std::endl 
                  << "           #include \"xmmintrin.h\"" << std::endl 
                  << "           #include \"pmmintrin.h\"" << std::endl 
                  << std::endl 
                  << "           _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);" << std::endl 
                  << "           _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);" << std::endl;
        std::cout << "================================================================================" << std::endl;
        std::cout << std::endl;
      }
    }
#endif

#if defined (__MIC__) && defined(RTCORE_BUFFER_STRIDE)
    if (State::instance()->verbosity(1))
      std::cout << "  WARNING: enabled 'bufferstride' support will lower BVH build performance" << std::endl;
#endif
  }

  RTCORE_API void rtcInit(const char* cfg) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInit);

    Lock<MutexSys> lock(g_mutex);
    if (g_initialized)
      throw_RTCError(RTC_INVALID_OPERATION,"already initialized");

    g_initialized = true;

    /* initialize global state */
    init_globals();
    State::instance()->clear();
    State::instance()->parseString(cfg);
    State::instance()->parseFile(FileName::executableFolder()+FileName(".embree" TOSTRING(__EMBREE_VERSION_MAJOR__)));
    if (FileName::homeFolder() != FileName("")) // home folder is not available on KNC
      State::instance()->parseFile(FileName::homeFolder()+FileName(".embree" TOSTRING(__EMBREE_VERSION_MAJOR__)));
    
    if (State::instance()->tessellation_cache_size)
      resizeTessellationCache( State::instance()->tessellation_cache_size );

    /*! enable some floating point exceptions to catch bugs */
    if (State::instance()->float_exceptions)
    {
      int exceptions = _MM_MASK_MASK;
      //exceptions &= ~_MM_MASK_INVALID;
      exceptions &= ~_MM_MASK_DENORM;
      exceptions &= ~_MM_MASK_DIV_ZERO;
      //exceptions &= ~_MM_MASK_OVERFLOW;
      //exceptions &= ~_MM_MASK_UNDERFLOW;
      //exceptions &= ~_MM_MASK_INEXACT;
      _MM_SET_EXCEPTION_MASK(exceptions);
    }

#if defined(__MIC__) // FIXME: put into State::verify function
    if (!(g_numThreads == 1 || (g_numThreads % 4) == 0))
      throw_RTCError(RTC_INVALID_OPERATION,"Xeon Phi supports only number of threads % 4 == 0, or threads == 1");
#endif

    /* print info header */
    if (State::instance()->verbosity(1))
      print_info();

    /* CPU has to support at least SSE2 */
#if !defined (__MIC__)
    if (!hasISA(SSE2)) 
      throw_RTCError(RTC_UNSUPPORTED_CPU,"CPU does not support SSE2");
#endif

#if !defined(__MIC__)
    BVH4Register();
#else
    BVH4iRegister();
    BVH4MBRegister();
    BVH4HairRegister();

#endif 
#if defined(__TARGET_AVX__)
    if (hasISA(AVX)) {
      BVH8Register();
    }
#endif
    
    InstanceIntersectorsRegister();

    if (State::instance()->verbosity(2)) 
      State::instance()->print();

#if defined(TASKING_LOCKSTEP)
    TaskScheduler::create(g_numThreads);
#endif

#if defined(TASKING_TBB_INTERNAL)
    TaskSchedulerTBB::create(g_numThreads);
#endif

#if defined(TASKING_TBB)
    if (g_numThreads == 0) {
      g_tbb_threads_initialized = false;
      g_numThreads = tbb::task_scheduler_init::default_num_threads();
    } else {
      g_tbb_threads_initialized = true;
      tbb_threads.initialize(g_numThreads);
      tbb_affinity.observe(true); 
    }
#endif

    /* execute regression tests */
    if (State::instance()->regression_testing) 
    {
#if defined(TASKING_LOCKSTEP)
      TaskScheduler::EventSync event;
      TaskScheduler::Task task(&event,task_regression_testing,nullptr,TaskScheduler::getNumThreads(),nullptr,nullptr,"regression_testing");
      TaskScheduler::addTask(-1,TaskScheduler::GLOBAL_FRONT,&task);
      event.sync();
#else
      runRegressionTests();
#endif
    }

    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcExit() 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcExit);
    
    Lock<MutexSys> lock(g_mutex);
    if (!g_initialized)
      throw_RTCError(RTC_INVALID_OPERATION,"rtcInit has to get called before rtcExit");

#if defined(TASKING_LOCKSTEP)
    TaskScheduler::destroy();
#endif

#if defined(TASKING_TBB_INTERNAL)
    TaskSchedulerTBB::destroy();
#endif

#if defined(TASKING_TBB)
    if (g_tbb_threads_initialized)
      tbb_threads.terminate();
#endif
    State::instance()->clear();
    g_initialized = false;
    RTCORE_CATCH_END;
  }

  RTCORE_API RTCError rtcGetError() 
  {
    RTCORE_TRACE(rtcGetError);
    RTCError* stored_error = State::error();
    RTCError error = *stored_error;
    *stored_error = RTC_NO_ERROR;
    return error;
  }

  RTCORE_API void rtcSetErrorFunction(RTCErrorFunc func) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetErrorFunction);
    State::instance()->g_error_function = func;
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetMemoryMonitorFunction(RTCMemoryMonitorFunc func) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetMemoryMonitorFunction);
    State::instance()->g_memory_monitor_function = func;
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcDebug()
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDebug);

#if defined(RTCORE_STAT_COUNTERS)
    Stat::print(std::cout);
    Stat::clear();
#endif

#if defined(DEBUG) && 0
    extern void printTessCacheStats();
    printTessCacheStats();
#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API RTCScene rtcNewScene (RTCSceneFlags flags, RTCAlgorithmFlags aflags) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewScene);
    if (!isCoherent(flags) && !isIncoherent(flags)) flags = RTCSceneFlags(flags | RTC_SCENE_INCOHERENT);
    return (RTCScene) new Scene(flags,aflags);
    RTCORE_CATCH_END;
    return nullptr;
  }

  RTCORE_API void rtcSetProgressMonitorFunction(RTCScene scene, RTCProgressMonitorFunc func, void* ptr) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetProgressMonitorFunction);
    RTCORE_VERIFY_HANDLE(scene);
    ((Scene*)scene)->setProgressMonitorFunction(func,ptr);
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcCommit (RTCScene scene) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommit);
    RTCORE_VERIFY_HANDLE(scene);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.dumpGeometry(scene);
#endif

    /* perform scene build */
    ((Scene*)scene)->build(0,0);
    
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcCommitThread(RTCScene scene, unsigned int threadID, unsigned int numThreads) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcCommitThread);
    RTCORE_VERIFY_HANDLE(scene);

    if (unlikely(numThreads == 0)) 
      throw_RTCError(RTC_INVALID_OPERATION,"invalid number of threads specified");

#if defined(__MIC__)
    if (unlikely(numThreads % 4 != 0 && numThreads != 1)) 
      throw_RTCError(RTC_INVALID_OPERATION,"MIC requires numThreads % 4 == 0 in rtcCommitThread");
#endif
    
    /* for best performance set FTZ and DAZ flags in the MXCSR control and status register */
#if !defined(__MIC__)
    unsigned int mxcsr = _mm_getcsr();
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif
    
     /* perform scene build */
    ((Scene*)scene)->build(threadID,numThreads);

 /* reset MXCSR register again */
#if !defined(__MIC__)
    _mm_setcsr(mxcsr);
#endif

    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcIntersect (RTCScene scene, RTCRay& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay old_ray = ray;
#endif

    STAT3(normal.travs,1,1,1);
    ((Scene*)scene)->intersect(ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay1Intersect(scene,old_ray,ray);
#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcIntersect4 (const void* valid, RTCScene scene, RTCRay4& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect4);
#if !defined(__TARGET_SIMD4__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect4 not supported");    
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,4);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay4 old_ray = ray;
#endif

    ((Scene*)scene)->intersect4(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay4Intersect(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcIntersect8 (const void* valid, RTCScene scene, RTCRay8& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect8);
#if !defined(__TARGET_SIMD8__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect8 not supported");                                    
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,8);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay8 old_ray = ray;
#endif

    ((Scene*)scene)->intersect8(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay8Intersect(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcIntersect16 (const void* valid, RTCScene scene, RTCRay16& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcIntersect16);
#if !defined(__TARGET_SIMD16__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcIntersect16 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(normal.travs,1,cnt,16);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay16 old_ray = ray;
#endif

    ((Scene*)scene)->intersect16(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay16Intersect(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcOccluded (RTCScene scene, RTCRay& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded);
    STAT3(shadow.travs,1,1,1);
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)&ray) & 0x0F        ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay old_ray = ray;
#endif

    ((Scene*)scene)->occluded(ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay1Occluded(scene,old_ray,ray);
#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcOccluded4 (const void* valid, RTCScene scene, RTCRay4& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded4);
#if !defined(__TARGET_SIMD4__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded4 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 16 bytes");   
    if (((size_t)&ray ) & 0x0F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 16 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<4; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,4);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay4 old_ray = ray;
#endif

    ((Scene*)scene)->occluded4(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay4Occluded(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcOccluded8 (const void* valid, RTCScene scene, RTCRay8& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded8);
#if !defined(__TARGET_SIMD8__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded8 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 32 bytes");   
    if (((size_t)&ray ) & 0x1F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 32 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<8; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,8);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay8 old_ray = ray;
#endif

    ((Scene*)scene)->occluded8(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RayStreamLogger::rayStreamLogger.logRay8Occluded(valid,scene,old_ray,ray);
#endif

#endif
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcOccluded16 (const void* valid, RTCScene scene, RTCRay16& ray) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcOccluded16);
#if !defined(__TARGET_SIMD16__)
    throw_RTCError(RTC_INVALID_OPERATION,"rtcOccluded16 not supported");
#else
#if defined(DEBUG)
    RTCORE_VERIFY_HANDLE(scene);
    if (((Scene*)scene)->isModified()) throw_RTCError(RTC_INVALID_OPERATION,"scene got not committed");
    if (((size_t)valid) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "mask not aligned to 64 bytes");   
    if (((size_t)&ray ) & 0x3F       ) throw_RTCError(RTC_INVALID_ARGUMENT, "ray not aligned to 64 bytes");   
#endif
    STAT(size_t cnt=0; for (size_t i=0; i<16; i++) cnt += ((int*)valid)[i] == -1;);
    STAT3(shadow.travs,1,cnt,16);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
    RTCRay16 old_ray = ray;
#endif

    ((Scene*)scene)->occluded16(valid,ray);

#if defined(RTCORE_ENABLE_RAYSTREAM_LOGGER)
  RayStreamLogger::rayStreamLogger.logRay16Occluded(valid,scene,old_ray,ray);
#endif

#endif
  RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcDeleteScene (RTCScene scene) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteScene);
    RTCORE_VERIFY_HANDLE(scene);
    delete (Scene*) scene;
    RTCORE_CATCH_END;
  }

  RTCORE_API unsigned rtcNewInstance (RTCScene target, RTCScene source) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewInstance);
    RTCORE_VERIFY_HANDLE(target);
    RTCORE_VERIFY_HANDLE(source);
    return ((Scene*) target)->newInstance((Scene*) source);
    RTCORE_CATCH_END;
    return -1;
  }

  RTCORE_API void rtcSetTransform (RTCScene scene, unsigned geomID, RTCMatrixType layout, const float* xfm) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetTransform);
    RTCORE_VERIFY_HANDLE(scene);
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

    RTCORE_CATCH_END;
  }

  RTCORE_API unsigned rtcNewUserGeometry (RTCScene scene, size_t numItems) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewUserGeometry);
    RTCORE_VERIFY_HANDLE(scene);
    return ((Scene*)scene)->newUserGeometry(numItems);
    RTCORE_CATCH_END;
    return -1;
  }

  RTCORE_API unsigned rtcNewTriangleMesh (RTCScene scene, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewTriangleMesh);
    RTCORE_VERIFY_HANDLE(scene);
    return ((Scene*)scene)->newTriangleMesh(flags,numTriangles,numVertices,numTimeSteps);
    RTCORE_CATCH_END;
    return -1;
  }

  RTCORE_API unsigned rtcNewHairGeometry (RTCScene scene, RTCGeometryFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewHairGeometry);
    RTCORE_VERIFY_HANDLE(scene);
    return ((Scene*)scene)->newBezierCurves(flags,numCurves,numVertices,numTimeSteps);
    RTCORE_CATCH_END;
    return -1;
  }

  RTCORE_API unsigned rtcNewSubdivisionMesh (RTCScene scene, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices, 
                                             size_t numEdgeCreases, size_t numVertexCreases, size_t numHoles, size_t numTimeSteps) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcNewSubdivisionMesh);
    RTCORE_VERIFY_HANDLE(scene);
    return ((Scene*)scene)->newSubdivisionMesh(flags,numFaces,numEdges,numVertices,numEdgeCreases,numVertexCreases,numHoles,numTimeSteps);
    RTCORE_CATCH_END;
    return -1;
  }

  RTCORE_API void rtcSetMask (RTCScene scene, unsigned geomID, int mask) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetMask);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setMask(mask);
    RTCORE_CATCH_END;
  }

  RTCORE_API void* rtcMapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcMapBuffer);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    return ((Scene*)scene)->get_locked(geomID)->map(type);
    RTCORE_CATCH_END;
    return nullptr;
  }

  RTCORE_API void rtcUnmapBuffer(RTCScene scene, unsigned geomID, RTCBufferType type) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUnmapBuffer);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->unmap(type);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetBuffer(RTCScene scene, unsigned geomID, RTCBufferType type, void* ptr, size_t offset, size_t stride)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBuffer);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setBuffer(type,ptr,offset,stride);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcEnable (RTCScene scene, unsigned geomID) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcEnable);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->enable();
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcUpdate (RTCScene scene, unsigned geomID) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdate);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->update();
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcUpdateBuffer (RTCScene scene, unsigned geomID, RTCBufferType type) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcUpdateBuffer);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->updateBuffer(type);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcDisable (RTCScene scene, unsigned geomID) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDisable);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->disable();
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcDeleteGeometry (RTCScene scene, unsigned geomID) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcDeleteGeometry);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->erase();
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetUserData (RTCScene scene, unsigned geomID, void* ptr) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetUserData);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setUserData(ptr);
    RTCORE_CATCH_END;
  }

  RTCORE_API void* rtcGetUserData (RTCScene scene, unsigned geomID)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcGetUserData);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    return ((Scene*)scene)->get(geomID)->getUserData(); // this call is on purpose not thread safe
    RTCORE_CATCH_END;
    return nullptr;
  }

  RTCORE_API void rtcSetBoundsFunction (RTCScene scene, unsigned geomID, RTCBoundsFunc bounds)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetBoundsFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setBoundsFunction(bounds);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetDisplacementFunction (RTCScene scene, unsigned geomID, RTCDisplacementFunc func, RTCBounds* bounds)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetDisplacementFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setDisplacementFunction(func,bounds);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetIntersectFunction (RTCScene scene, unsigned geomID, RTCIntersectFunc intersect) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction(intersect);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetIntersectFunction4 (RTCScene scene, unsigned geomID, RTCIntersectFunc4 intersect4) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction4(intersect4);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetIntersectFunction8 (RTCScene scene, unsigned geomID, RTCIntersectFunc8 intersect8) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction8(intersect8);
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcSetIntersectFunction16 (RTCScene scene, unsigned geomID, RTCIntersectFunc16 intersect16) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectFunction16(intersect16);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction (RTCScene scene, unsigned geomID, RTCOccludedFunc occluded) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction(occluded);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction4 (RTCScene scene, unsigned geomID, RTCOccludedFunc4 occluded4) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction4(occluded4);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction8 (RTCScene scene, unsigned geomID, RTCOccludedFunc8 occluded8) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction8(occluded8);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOccludedFunction16 (RTCScene scene, unsigned geomID, RTCOccludedFunc16 occluded16) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOccludedFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOccludedFunction16(occluded16);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetIntersectionFilterFunction (RTCScene scene, unsigned geomID, RTCFilterFunc intersect) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction(intersect);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetIntersectionFilterFunction4 (RTCScene scene, unsigned geomID, RTCFilterFunc4 filter4) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction4(filter4);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetIntersectionFilterFunction8 (RTCScene scene, unsigned geomID, RTCFilterFunc8 filter8) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction8(filter8);
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcSetIntersectionFilterFunction16 (RTCScene scene, unsigned geomID, RTCFilterFunc16 filter16) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetIntersectionFilterFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setIntersectionFilterFunction16(filter16);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOcclusionFilterFunction (RTCScene scene, unsigned geomID, RTCFilterFunc intersect) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction(intersect);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOcclusionFilterFunction4 (RTCScene scene, unsigned geomID, RTCFilterFunc4 filter4) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction4);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction4(filter4);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcSetOcclusionFilterFunction8 (RTCScene scene, unsigned geomID, RTCFilterFunc8 filter8) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction8);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction8(filter8);
    RTCORE_CATCH_END;
  }
  
  RTCORE_API void rtcSetOcclusionFilterFunction16 (RTCScene scene, unsigned geomID, RTCFilterFunc16 filter16) 
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcSetOcclusionFilterFunction16);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get_locked(geomID)->setOcclusionFilterFunction16(filter16);
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcInterpolate(RTCScene scene, unsigned geomID, unsigned primID, float u, float v, const float* src, size_t byteStride, float* dst, size_t numFloats)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolate);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    ((Scene*)scene)->get(geomID)->interpolate(primID,u,v,src,byteStride,dst,numFloats); // this call is on purpose not thread safe
    RTCORE_CATCH_END;
  }

  RTCORE_API void rtcInterpolateN(RTCScene scene, unsigned geomID, unsigned primID,
                                  const void* valid_i, const float* u, const float* v, size_t numUVs, 
                                  const float* src, size_t byteStride, float* dst, size_t numFloats)
  {
    RTCORE_CATCH_BEGIN;
    RTCORE_TRACE(rtcInterpolateN);
    RTCORE_VERIFY_HANDLE(scene);
    RTCORE_VERIFY_GEOMID(geomID);
    if (numFloats > 256) throw_RTCError(RTC_INVALID_OPERATION,"maximally 256 floating point values can be interpolated per vertex");
    const int* valid = (const int*) valid_i;
    for (size_t i=0; i<numFloats; i++) // FIXME: implement fast path for packet queries
    {
      if (valid && !valid[i]) continue;
      float dst1[256];
      rtcInterpolate(scene,geomID,primID,u[i],v[i],src,byteStride,dst1,numFloats);
      for (size_t j=0; j<numFloats; j++)
        dst[j*numFloats+i] = dst1[j];
    }
    RTCORE_CATCH_END;
  }
}
