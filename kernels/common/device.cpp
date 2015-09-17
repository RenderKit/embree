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

#include "device.h"
#include "version.h"
#include "scene_triangle_mesh.h"
#include "scene_user_geometry.h"
#include "scene_instance.h"
#include "scene_bezier_curves.h"
#include "scene_subdiv_mesh.h"

#include "subdiv/tessellation_cache.h"

#include "acceln.h"
#include "geometry.h"

#if !defined(_MM_SET_DENORMALS_ZERO_MODE)
#define _MM_DENORMALS_ZERO_ON   (0x0040)
#define _MM_DENORMALS_ZERO_OFF  (0x0000)
#define _MM_DENORMALS_ZERO_MASK (0x0040)
#define _MM_SET_DENORMALS_ZERO_MODE(x) (_mm_setcsr((_mm_getcsr() & ~_MM_DENORMALS_ZERO_MASK) | (x)))
#endif

#if defined(TASKING_LOCKSTEP)
#  include "../../common/tasking/taskscheduler_mic.h"
#elif defined(TASKING_TBB_INTERNAL)
#  include "../../common/tasking/taskscheduler_tbb.h"
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
  DECLARE_SYMBOL(RTCBoundsFunc,InstanceBoundsFunc); // FIXME: move this state to device class
  DECLARE_SYMBOL(AccelSet::Intersector1,InstanceIntersector1);
  DECLARE_SYMBOL(AccelSet::Intersector4,InstanceIntersector4);
  DECLARE_SYMBOL(AccelSet::Intersector8,InstanceIntersector8);
  DECLARE_SYMBOL(AccelSet::Intersector16,InstanceIntersector16);
  
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
#if defined (RTCORE_RAY_PACKETS)
    SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,InstanceIntersector4);
    SELECT_SYMBOL_AVX_AVX2(features,InstanceIntersector8);
#endif
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

#if defined(TASKING_TBB)

  template <typename R, typename S>
  R tbb_pi( S num_steps )
  {
    const R step = R(1) / num_steps;
    return step * tbb::parallel_reduce( tbb::blocked_range<S>( 0, num_steps ), R(0),
                                        [step] ( const tbb::blocked_range<S> r, R local_sum ) -> R {
                                          for ( S i = r.begin(); i < r.end(); ++i ) {
                                            R x = (i + R(0.5)) * step;
                                            local_sum += R(4) / (R(1) + x*x);
                                          }
                                          return local_sum;
                                        },
                                        std::plus<R>()
      );
  }

  bool g_tbb_threads_initialized = false;
  tbb::task_scheduler_init tbb_threads(tbb::task_scheduler_init::deferred);

  class TBBAffinity: public tbb::task_scheduler_observer
  {
    tbb::atomic<int> num_threads;

    void on_scheduler_entry( bool ) {
      ++num_threads;
      setAffinity(TaskSchedulerTBB::threadIndex());
      //PING;
      //PRINT(num_threads);
    }

    void on_scheduler_exit( bool ) { 
      --num_threads; 
      //PING;
      //PRINT(num_threads);
    }
  public:
    
    TBBAffinity() { num_threads = 0; }

    int  get_concurrency()      { return num_threads; }
    void set_concurrency(int i) { num_threads = i; }

  } tbb_affinity;
#endif

  static MutexSys g_cache_size_mutex;
  static std::map<Device*,size_t> g_cache_size_map;

  Device::Device (const char* cfg, bool singledevice)
  {
    /* initialize global state */
    init_globals();
    State::clear(singledevice);
    State::parseString(cfg);
    State::parseFile(FileName::executableFolder()+FileName(".embree" TOSTRING(__EMBREE_VERSION_MAJOR__)));
    if (FileName::homeFolder() != FileName("")) // home folder is not available on KNC
      State::parseFile(FileName::homeFolder()+FileName(".embree" TOSTRING(__EMBREE_VERSION_MAJOR__)));
    
    /*! set tessellation cache size */
    setCacheSize( State::tessellation_cache_size );

    /*! enable some floating point exceptions to catch bugs */
    if (State::float_exceptions)
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
    if (State::verbosity(1))
      print();

    /* CPU has to support at least SSE2 */
#if !defined (__MIC__)
    if (!hasISA(SSE2)) 
      throw_RTCError(RTC_UNSUPPORTED_CPU,"CPU does not support SSE2");
#endif

    /* verify that calculations stay in range */
    assert(rcp(min_rcp_input)*FLT_LARGE+FLT_LARGE < 0.01f*FLT_MAX);

    /* here we verify that CPP files compiled for a specific ISA only
     * call that same or lower ISA version of non-inlined class member
     * functions */
#if !defined (__MIC__) && defined(DEBUG)
    assert(isa::getISA() == ISA);
#if defined(__TARGET_SSE41__)
    assert(sse41::getISA() <= SSE41);
#endif
#if defined(__TARGET_SSE42__)
    assert(sse42::getISA() <= SSE42);
#endif
#if defined(__TARGET_AVX__)
    assert(avx::getISA() <= AVX);
#endif
#if defined(__TARGET_AVX2__)
    assert(avx2::getISA() <= AVX2);
#endif
#if defined (__TARGET_AVX512__)
    assert(avx512::getISA() <= AVX512KNL);
#endif
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

    if (State::verbosity(2)) 
      State::print();

#if defined(TASKING_LOCKSTEP)
    TaskScheduler::create(g_numThreads,State::set_affinity); // FIXME: prevents creation of two devices
#endif

#if defined(TASKING_TBB_INTERNAL)
    TaskSchedulerTBB::create(g_numThreads,State::set_affinity); // FIXME: prevents creation of two devices
#endif

#if defined(TASKING_TBB)

    /* only set affinity of requested by the user */
    if (State::set_affinity) {
      tbb_affinity.set_concurrency(0);
      tbb_affinity.observe(true); 
    }
    
    if (g_numThreads == 0) {
      g_tbb_threads_initialized = false;
      g_numThreads = tbb::task_scheduler_init::default_num_threads();
    } else {
      g_tbb_threads_initialized = true;
      tbb_threads.initialize(g_numThreads);

#if 0
      const size_t N = 1024*1024;
      //PRINT(g_numThreads );
      while (tbb_affinity.get_concurrency() < g_numThreads /*tbb::task_scheduler_init::default_num_threads()*/) 
        tbb_pi<double> (N);
      PRINT( tbb_affinity.get_concurrency() );
#endif
    }
#if USE_TASK_ARENA
    arena = new tbb::task_arena(g_numThreads);
#endif
#endif

    /* execute regression tests */
    if (State::regression_testing) 
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
  }

  Device::~Device ()
  {
    setCacheSize(0);
   
#if defined(TASKING_LOCKSTEP)
    TaskScheduler::destroy(); // FIXME: prevents creation of two devices
#endif

#if defined(TASKING_TBB_INTERNAL)
    TaskSchedulerTBB::destroy(); // FIXME: prevents creation of two devices
#endif

#if defined(TASKING_TBB)
#if USE_TASK_ARENA
    delete arena; arena = nullptr;
#endif
    if (g_tbb_threads_initialized)
      tbb_threads.terminate();
#endif
  }

  void Device::print()
  {
    std::cout << "Embree Ray Tracing Kernels " << __EMBREE_VERSION__ << " (" << __DATE__ << ")" << std::endl;
    std::cout << "  Compiler : " << getCompilerName() << std::endl;
    std::cout << "  Platform : " << getPlatformName() << std::endl;
    std::cout << "  CPU      : " << stringOfCPUModel(getCPUModel()) << " (" << getCPUVendor() << ")" << std::endl;
    std::cout << "  ISA      : " << stringOfCPUFeatures(getCPUFeatures()) << std::endl;
    std::cout << "  Threads  : " << getNumberOfLogicalThreads() << std::endl;
#if !defined(__MIC__)
    const bool hasFTZ = _mm_getcsr() & _MM_FLUSH_ZERO_ON;
    const bool hasDAZ = _mm_getcsr() & _MM_DENORMALS_ZERO_ON;
    std::cout << "  MXCSR    : " << "FTZ=" << hasFTZ << ", DAZ=" << hasDAZ << std::endl;
#endif
    std::cout << "  Config   : ";
#if defined(DEBUG)
    std::cout << "Debug ";
#else
    std::cout << "Release ";
#endif
#if defined(TASKING_TBB)
    std::cout << "TBB" << TBB_VERSION_MAJOR << "." << TBB_VERSION_MINOR << " ";
    std::cout << "TBB_header_interface_" << TBB_INTERFACE_VERSION << " TBB_lib_interface_" << tbb::TBB_runtime_interface_version() << " ";
#endif
    // FIXME: should also print SSE2, SSSE3, MIC
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
      if (State::verbosity(1)) 
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
    if (State::verbosity(1))
      std::cout << "  WARNING: enabled 'bufferstride' support will lower BVH build performance" << std::endl;
#endif
  }

  void Device::process_error(RTCError error, const char* str)
  { 
    /* print error when in verbose mode */
    if (State::verbosity(1)) 
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
    if (State::error_function) 
      State::error_function(error,str); 

    /* record error code */
    RTCError* stored_error = State::error();
    if (*stored_error == RTC_NO_ERROR)
      *stored_error = error;
  }

  void Device::memoryMonitor(ssize_t bytes, bool post)
  {
    if (State::memory_monitor_function && bytes != 0) {
      if (!State::memory_monitor_function(bytes,post)) {
#if !defined(TASKING_LOCKSTEP) && !defined(TASKING_TBB_INTERNAL)
        if (bytes > 0) { // only throw exception when we allocate memory to never throw inside a destructor
          throw_RTCError(RTC_OUT_OF_MEMORY,"memory monitor forced termination");
        }
#endif
      }
    }
  }
 
  void Device::setCacheSize(size_t bytes) 
  {
    Lock<MutexSys> lock(g_cache_size_mutex);
    if (bytes == 0) g_cache_size_map.erase(this);
    else            g_cache_size_map[this] = bytes;
    updateCacheSize();
  }

  void Device::updateCacheSize()
  {
    size_t maxCacheSize = 0;
    for (std::map<Device*,size_t>::iterator i=g_cache_size_map.begin(); i!= g_cache_size_map.end(); i++)
      maxCacheSize = max(maxCacheSize, (*i).second);
    
    resizeTessellationCache(max(size_t(1024*1024),maxCacheSize));
  }

  void Device::setParameter1i(const RTCParameter parm, ssize_t val)
  {
    switch (parm) {
    case RTC_SOFTWARE_CACHE_SIZE: setCacheSize(val); break;
    default: throw_RTCError(RTC_INVALID_ARGUMENT, "unknown parameter"); break;
    };
  }
}
